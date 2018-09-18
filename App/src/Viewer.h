#pragma once
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

class Viewer {
public:
    Viewer();
    ~Viewer();

    static Viewer& Instance() {
        static Viewer instance;
        return instance;
    }

    void RisgisterPub(ros::NodeHandle &nh);

private:
    ros::Publisher pub_feature_tracking;
};
