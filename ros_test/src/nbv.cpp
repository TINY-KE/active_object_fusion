#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


int i = 0;
#include <Eigen/Core>
// #include <Eigen>
#include <System.h>
#include "Global.h"
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

// imu的相关工具
#include "tf/transform_datatypes.h"//转换函数头文件
#include <sensor_msgs/Imu.h>//imu数据信息
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
std::string WORK_SPACE_PATH = "";
ORB_SLAM2::System* system_ptr;

//std::string DatasetType;
Eigen::Matrix4d INIT_POSE = Eigen::Matrix4d::Identity();
std::vector<BoxSE> darknetRosMsgToBoxSE(std::vector<darknet_ros_msgs::BoundingBox>& boxes);

int main(int argc, char **argv)
{
    WORK_SPACE_PATH = "/home/zhjd/ws_active/src/kinect/EAO-Fusion/";

    //(1) ROS
    ros::init(argc, argv, "EllipsoidSLAM");
    ros::NodeHandle nh;

    std::cout << "Waiting for generating frame ..." << std::endl;

    //(2) 生成物体及内部物体




    
    ros::spin();
    ros::shutdown();

    return 0;
}

