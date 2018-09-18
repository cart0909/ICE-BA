#pragma once
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "CameraPoseVisualization.h"

class Viewer {
public:
    Viewer();
    ~Viewer();

    static Viewer& Instance() {
        static Viewer instance;
        return instance;
    }

    void RisgisterPub(ros::NodeHandle &nh);
    void PubFeautreTracking(const cv::Mat& img,
                            const std::vector<cv::KeyPoint>& kps);
    void PubOdometry(const Eigen::Matrix4f& Tws, const Eigen::Vector3f& velocity);
    void PubKeyFramePose(std::vector<Eigen::Vector3f,
                         Eigen::aligned_allocator<Eigen::Vector3f>>& vec_p);
private:
    ros::Publisher pub_feature_tracking;
    ros::Publisher pub_odometry;
    ros::Publisher pub_path;
    ros::Publisher pub_camera_pose_visual, pub_kf_poses;
    nav_msgs::Path path;
    CameraPoseVisualization camera_pose_visual;
};
