#include "Viewer.h"

Viewer::Viewer() : camera_pose_visual(1, 0, 0, 1) {
    camera_pose_visual.setScale(0.3);
}
Viewer::~Viewer() {}

void Viewer::RisgisterPub(ros::NodeHandle &nh) {
    pub_feature_tracking = nh.advertise<sensor_msgs::Image>("feature_img", 100);
    pub_path = nh.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = nh.advertise<nav_msgs::Odometry>("odometry", 100);
    pub_camera_pose_visual = nh.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 100);
    pub_kf_poses = nh.advertise<visualization_msgs::Marker>("kf_poses", 100);
}

void Viewer::PubFeautreTracking(const cv::Mat& img,
                                const std::vector<cv::KeyPoint>& kps) {
//    std_msgs::Header header_msg;

    cv::Mat show_img;
    cv::cvtColor(img, show_img, CV_GRAY2BGR);

    for(auto& pt : kps) {
        cv::circle(show_img, pt.pt, 4, cv::Scalar(0, 255, 0), -1);
    }

    cv_bridge::CvImage img_msg;
    img_msg.header.frame_id = "world";
    img_msg.image = show_img;
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    pub_feature_tracking.publish(img_msg.toImageMsg());
}

void Viewer::PubOdometry(const Eigen::Matrix4f& Tws, const Eigen::Vector3f& velocity) {
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Eigen::Quaternionf tmp_q(Tws.block<3, 3>(0, 0));
    odometry.pose.pose.position.x = Tws(0, 3);
    odometry.pose.pose.position.y = Tws(1, 3);
    odometry.pose.pose.position.z = Tws(2, 3);
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    odometry.twist.twist.linear.x = velocity(0);
    odometry.twist.twist.linear.y = velocity(1);
    odometry.twist.twist.linear.z = velocity(2);
//    pub_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    path.header.frame_id = "world";
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    camera_pose_visual.reset();
    camera_pose_visual.add_pose(Tws.block<3, 1>(0, 3).cast<double>(), tmp_q.cast<double>());
    camera_pose_visual.publish_by(pub_camera_pose_visual, odometry.header);
}

void Viewer::PubKeyFramePose(std::vector<Eigen::Vector3f,
                             Eigen::aligned_allocator<Eigen::Vector3f>>& vec_p) {
    visualization_msgs::Marker key_poses;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (auto& p : vec_p)
    {
        geometry_msgs::Point pose_marker;
        pose_marker.x = p.x();
        pose_marker.y = p.y();
        pose_marker.z = p.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_kf_poses.publish(key_poses);
}
