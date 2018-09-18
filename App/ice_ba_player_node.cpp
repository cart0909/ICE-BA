#include "System.h"
#include "basic_datatype.h"
#include "ros_utility.h"
#include "tracer.h"
#include "utility.h"
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <thread>

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

class Node {
public:
    using Measurements = vector<pair<pair<ImageConstPtr, ImageConstPtr>, vector<ImuConstPtr>>>;

    Node() {}
    ~Node() {}

    void ImageCallback(const sensor_msgs::ImageConstPtr& img_msg,
        const sensor_msgs::ImageConstPtr& img_r_msg)
    {
        ScopedTrace st("image_c");
        unique_lock<mutex> lock(m_buf);
        img_buf.emplace(img_msg, img_r_msg);
        cv_system.notify_one();
    }

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
    {
        ScopedTrace st("imu_c");
        unique_lock<mutex> lock(m_buf);
        imu_buf.emplace(imu_msg);
        cv_system.notify_one();
    }

    void ReadFromNodeHandle(ros::NodeHandle& nh)
    {
        std::string config_file;
        config_file = readParam<std::string>(nh, "config_file");

        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        fs["imu_topic"] >> imu_topic;
        fs["image_topic"] >> img_topic;
        fs["image_r_topic"] >> img_r_topic;
        fs.release();

        mSystem.ReadConfigYaml(config_file);
        t_system = std::thread(&Node::SystemThread, this);
    }

    Measurements GetMeasurements()
    {
        // The buffer mutex is locked before this function be called.
        Measurements measurements;

        while (1) {
            if (imu_buf.empty() || img_buf.empty())
                return measurements;

            double img_ts = img_buf.front().first->header.stamp.toSec();
            // catch the imu data before image_timestamp
            // ---------------^-----------^ image
            //                f           f+1
            // --x--x--x--x--x--x--x--x--x- imu
            //   f                       b
            // --o--o--o--o--o^-?---------- collect data in frame f

            // if ts(imu(b)) < ts(img(f)), wait imu data
            if (imu_buf.back()->header.stamp.toSec() < img_ts) {
                return measurements;
            }
            // if ts(imu(f)) > ts(img(f)), img data faster than imu data, drop the img(f)
            if (imu_buf.front()->header.stamp.toSec() > img_ts) {
                img_buf.pop();
                continue;
            }

            pair<ImageConstPtr, ImageConstPtr> img_msg = img_buf.front();
            img_buf.pop();

            vector<ImuConstPtr> IMUs;
            while (imu_buf.front()->header.stamp.toSec() < img_ts) {
                IMUs.emplace_back(imu_buf.front());
                imu_buf.pop();
            }
//            IMUs.emplace_back(imu_buf.front()); // ??
            measurements.emplace_back(img_msg, IMUs);
        }

        return measurements;
    }

    void SystemThread()
    {
        while (1) {
            Measurements measurements;
            std::unique_lock<std::mutex> lock(m_buf);
            cv_system.wait(lock, [&] {
                return (measurements = GetMeasurements()).size() != 0;
            });
            lock.unlock();

            for(auto& meas : measurements) {
                auto& img_msg = meas.first.first;
                auto& img_msg_right = meas.first.second;
                double timestamp = img_msg->header.stamp.toSec();

                if(mbImageFirstEnqueue) {
                    mdImageFirstEnqueueTimeStamp = timestamp;
                    mbImageFirstEnqueue = false;
                }

                cv::Mat img_left, img_right;
                img_left = cv_bridge::toCvShare(img_msg, "mono8")->image;
                img_right = cv_bridge::toCvShare(img_msg_right, "mono8")->image;

                auto& v_imu_msg = meas.second;
                std::vector<XP::ImuData> imu_data;
                for(auto& imu_msg : v_imu_msg) {
                    XP::ImuData temp_imu;
                    temp_imu.accel(0) = imu_msg->angular_velocity.x;
                    temp_imu.accel(1) = imu_msg->angular_velocity.y;
                    temp_imu.accel(2) = imu_msg->angular_velocity.z;

                    temp_imu.ang_v(0) = imu_msg->angular_velocity.x;
                    temp_imu.ang_v(1) = imu_msg->angular_velocity.y;
                    temp_imu.ang_v(2) = imu_msg->angular_velocity.z;

                    temp_imu.time_stamp = imu_msg->header.stamp.toSec() - mdImageFirstEnqueueTimeStamp;
                    imu_data.emplace_back(temp_imu);
                }

                mSystem.TrackStereoVIO(img_left, img_right, timestamp - mdImageFirstEnqueueTimeStamp, imu_data);
            }
        }
    }

    string imu_topic;
    string img_r_topic;
    string img_topic;

    mutex m_buf;
    queue<ImuConstPtr> imu_buf;
    queue<pair<ImageConstPtr, ImageConstPtr>> img_buf;

    System mSystem;
    std::condition_variable cv_system;
    std::thread t_system;

    bool mbImageFirstEnqueue = true;
    double mdImageFirstEnqueueTimeStamp;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ice_ba_player");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Info);
    Node node;
    node.ReadFromNodeHandle(nh);

    //    node.sub_img = nh.subscribe(node.img_topic, 100, &Node::ImageCallback, &node);
    //    node.sub_img_r = nh.subscribe(node.img_r_topic, 100, &Node::ImageRCallback, &node);

    message_filters::Subscriber<Image> sub_image(nh, node.img_topic, 100);
    message_filters::Subscriber<Image> sub_image_r(nh, node.img_r_topic, 100);
    TimeSynchronizer<Image, Image> sync(sub_image, sub_image_r, 100);
    sync.registerCallback(boost::bind(&Node::ImageCallback, &node, _1, _2));

    ros::Subscriber sub_imu = nh.subscribe(node.imu_topic, 2000, &Node::ImuCallback, &node,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("ICE-BA Node is ready.");

    ros::spin();
    return 0;
}
