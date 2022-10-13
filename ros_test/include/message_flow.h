/*
 * @Author: Chen Jiahao
 * @Date: 2021-11-04 14:29:27
 * @LastEditors: Chen Jiahao
 * @LastEditTime: 2021-12-23 21:54:13
 * @Description: file content
 * @FilePath: /catkin_ws/src/EAO-SLAM/ros_test/include/message_flow.h
 */
#ifndef _MESSAGE_FLOW_H_
#define _MESSAGE_FLOW_H_

#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <System.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "Global.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"

#include "imu_subscriber.h"
#include "image_subscriber.h"
// IMU相关
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <eigen3/Eigen/Core>

#include "tool.h"

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
    int nFrames = 0;
    int lostFrames = 0;
    int idx = 0;
    int similarity = -1;
    // int trackerIdx = -1;
};




//class RGBDIMessageFlow
//{
//public:
//    // 图像数据指针
//    std::shared_ptr<IMGSubscriber> image_sub_ptr_;
//    // IMU数据指针
//    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
//    // ORB-SLAM指针
//    std::shared_ptr<ORB_SLAM2::System> slam_ptr_;
//
//private:
//    // 判断是否在线的标志
//    bool semanticOnline;
//    bool initIMUFlag;
//    bool rosBagFlag;
//    int gravity_aixs = 3;
//    int count = 0;
//    // 判断传感器类型的标志
//    std::string sensor;
//    // 与图像进行同步的IMU数据缓存队列
//    std::deque<sensor_msgs::Imu> synced_imu_data_buff_;
//    // 原始的IMU数据缓存队列
//    std::deque<sensor_msgs::Imu> unsynced_imu_data_buff_;
//
//    std::deque<sensor_msgs::Image> image_color_data_buff_;
//    std::deque<sensor_msgs::Image> image_depth_data_buff_;
//
//    std::deque<sensor_msgs::Imu> unsynced_imu_data_;
//
//    sensor_msgs::Imu synced_imu_data_;
//    sensor_msgs::Image current_image_color_data_;
//    sensor_msgs::Image current_image_depth_data_;
//    cv::Mat cvColorImgMat, cvDepthMat;
//    // cv::Mat CurrentGray;
//
//public:
//    RGBDIMessageFlow(ros::NodeHandle &nh);
//
//    ~RGBDIMessageFlow();
//
//    void Run();
//
//    bool ReadData();
//
//    bool HasData();
//
//    bool ValidData();
//
//    bool InitIMU();
//
//    bool IMUSyncData(
//        std::deque<sensor_msgs::Imu> &UnsyncedDataBuff,
//        std::deque<sensor_msgs::Imu> &UnsyncedData,
//        std::deque<sensor_msgs::Imu> &SyncedData,
//        ros::Time sync_time);
//
//    void SaveTrajectory();
//};

class RGBDMessageFlow
{
public:
    // 图像数据指针
    std::shared_ptr<IMGSubscriber> image_sub_ptr_;
    // ORB-SLAM指针
    std::shared_ptr<ORB_SLAM2::System> slam_ptr_;

private:
    // ros topic
    std::string rgb_topic, depth_topic, bbox_topic ,yamlfile;
    // 判断是否在线的标志
    bool semanticOnline;
    bool rosBagFlag;
    int count = 0;
    // 判断传感器类型的标志
    std::string sensor;

    std::deque<sensor_msgs::Image> image_color_data_buff_;
    std::deque<sensor_msgs::Image> image_depth_data_buff_;
    std::deque<darknet_ros_msgs::BoundingBoxes> image_bbox_data_buff_;
    sensor_msgs::Image current_image_color_data_;
    sensor_msgs::Image current_image_depth_data_;
    darknet_ros_msgs::BoundingBoxes current_image_bbox_data_;
    cv::Mat cvColorImgMat, cvDepthMat;
    Eigen::MatrixXd cvBboxMat;  //没用到
    std::vector<BoxSE> cvBboxVector;
    double timestamp;

    // cv::Mat CurrentGray;

public:
    RGBDMessageFlow(ros::NodeHandle &nh);

    ~RGBDMessageFlow();

    void Run();

    bool ReadData();

    bool HasData();

    bool ValidData();

    void SaveTrajectory();

    Eigen::MatrixXd darknetRosMsgToMat(std::vector<darknet_ros_msgs::BoundingBox>& boxes)
    {
        //修改
        if (boxes.size() == 0)
        {
            std::cout << "[WARNNING] OBJECTS SIZE IS ZERO" << std::endl;
        }
        /* darknet_ros_msgs::BoundingBox
         * float64 probability
         * int64 xmin
         * int64 ymin
         * int64 xmax
         * int64 ymax
         * int16 id
         * string Class
         * */
        std::vector<BoxSE> boxes_online;
        for (auto &objInfo : boxes)
        {
            if (objInfo.probability < 0.5)
                continue;
            // TODO: 检测和yolo3ros 相同.
            // 0: person; 24: handbag?24应该是backpack背包,26是handbag手提包; 28: suitcase; 39: bottle; 56: chair;
            // 57: couch; 58:potted plant; 59: bed; 60: dining table; 62: tv;
            // 63: laptop; 66: keyboard; 67: phone; 73: book;
            if (objInfo.id != 0 && objInfo.id != 24 && objInfo.id != 28 && objInfo.id != 39 && objInfo.id != 56 && objInfo.id != 57 && objInfo.id != 58 && objInfo.id != 59 && objInfo.id != 60 && objInfo.id != 62 && objInfo.id != 63 && objInfo.id != 66 && objInfo.id != 67 && objInfo.id != 73)
                continue;
            BoxSE box;
            box.m_class = objInfo.id;
            box.m_score = objInfo.probability;
            box.x = objInfo.xmin;
            box.y = objInfo.ymin;
            box.width = (objInfo.xmax - objInfo.xmin);
            box.height = (objInfo.ymax - objInfo.ymin );
            // box.m_class_name = "";
            boxes_online.push_back(box);
        }
        std::sort(boxes_online.begin(), boxes_online.end(), [](BoxSE a, BoxSE b) -> bool { return a.m_score > b.m_score; });

        // std::vector<BoxSE> --> Eigen::MatrixXd.
        int i = 0;
        Eigen::MatrixXd eigenMat;

        eigenMat.resize((int)boxes_online.size(), 5);
        for (auto &box : boxes_online)
        {
            eigenMat(i, 0) = box.x;
            eigenMat(i, 1) = box.y;
            eigenMat(i, 2) = box.width;
            eigenMat(i, 3) = box.height;
            eigenMat(i, 4) = box.m_score;
            i++;
        }
        return eigenMat;
//    liaoziwei版本
//    Eigen::MatrixXd boxMat; boxMat.resize(boxes.size(), 7);
//    for( int i=0;i<boxMat.rows();i++ )
//    {
//        auto box = boxes[i];
//        Eigen::Matrix<double, 7, 1> boxVec;
//        boxVec << i,box.xmin,box.ymin,box.xmax,box.ymax,box.id,box.probability;
//        boxMat.row(i) = boxVec.transpose();
//    }
//    return boxMat;
    }


    std::vector<BoxSE> darknetRosMsgToBoxSE(std::vector<darknet_ros_msgs::BoundingBox>& boxes)
    {
        //修改
        if (boxes.size() == 0)
        {
            std::cout << "[WARNNING] OBJECTS SIZE IS ZERO" << std::endl;
        }
        /* darknet_ros_msgs::BoundingBox
         * float64 probability
         * int64 xmin
         * int64 ymin
         * int64 xmax
         * int64 ymax
         * int16 id
         * string Class
         * */
        std::vector<BoxSE> boxes_online;
        for (auto &objInfo : boxes)
        {
            if (objInfo.probability < 0.5)
                continue;
            // TODO: 检测和yolo3ros 相同.
            // 0: person; 24: handbag?24应该是backpack背包,26是handbag手提包; 28: suitcase; 39: bottle; 56: chair;
            // 57: couch; 58:potted plant; 59: bed; 60: dining table; 62: tv;
            // 63: laptop; 66: keyboard; 67: phone; 73: book;
            if (objInfo.id != 0 && objInfo.id != 24 && objInfo.id != 28 && objInfo.id != 39 && objInfo.id != 56 && objInfo.id != 57 && objInfo.id != 58 && objInfo.id != 59 && objInfo.id != 60 && objInfo.id != 62 && objInfo.id != 63 && objInfo.id != 66 && objInfo.id != 67 && objInfo.id != 73)
                continue;
            BoxSE box;
            box.m_class = objInfo.id;
            box.m_score = objInfo.probability;
            box.x = objInfo.xmin;
            box.y = objInfo.ymin;
            box.width = (objInfo.xmax - objInfo.xmin);
            box.height = (objInfo.ymax - objInfo.ymin );
            // box.m_class_name = "";
            boxes_online.push_back(box);
        }
        std::sort(boxes_online.begin(), boxes_online.end(), [](BoxSE a, BoxSE b) -> bool { return a.m_score > b.m_score; });
        return boxes_online;
//    liaoziwei版本
//    Eigen::MatrixXd boxMat; boxMat.resize(boxes.size(), 7);
//    for( int i=0;i<boxMat.rows();i++ )
//    {
//        auto box = boxes[i];
//        Eigen::Matrix<double, 7, 1> boxVec;
//        boxVec << i,box.xmin,box.ymin,box.xmax,box.ymax,box.id,box.probability;
//        boxMat.row(i) = boxVec.transpose();
//    }
//    return boxMat;
    }
};


//class RGBDMessageFlow_test
//{
//public:
//    // 图像数据指针
//    std::shared_ptr<IMGSubscriber> image_sub_ptr_;
//    // ORB-SLAM指针
//    std::shared_ptr<ORB_SLAM2::System> slam_ptr_;
//
//private:
//    // ros topic
//    std::string rgb_topic, depth_topic, bbox_topic ,yamlfile;
//    // 判断是否在线的标志
//    bool semanticOnline;
//    bool rosBagFlag;
//    int count = 0;
//    // 判断传感器类型的标志
//    std::string sensor;
//
//    std::deque<sensor_msgs::Image> image_color_data_buff_;
//    std::deque<sensor_msgs::Image> image_depth_data_buff_;
//    std::deque<darknet_ros_msgs::BoundingBoxes> image_bbox_data_buff_;
//    sensor_msgs::Image current_image_color_data_;
//    sensor_msgs::Image current_image_depth_data_;
//    darknet_ros_msgs::BoundingBoxes current_image_bbox_data_;
//    cv::Mat cvColorImgMat, cvDepthMat;
//    Eigen::MatrixXd cvBboxMat;  //没用到
//    std::vector<BoxSE> cvBboxVector;
//    double timestamp;
//
//    // cv::Mat CurrentGray;
//
//public:
//    RGBDMessageFlow_test(ros::NodeHandle &nh)
//    {
//        // 初始化图像
//        ros::param::param<std::string>("~rgb_topic", rgb_topic, "/camera/color/image_raw");
//        ros::param::param<std::string>("~depth_topic", depth_topic, "/camera/aligned_depth_to_color/image_raw");
//        ros::param::param<std::string>("~bbox_topic", bbox_topic, "/darknet_ros/bounding_boxes");
//        image_sub_ptr_ = std::make_shared<IMGSubscriber>(nh, rgb_topic, depth_topic, bbox_topic, 1000);
//        // image_sub_ptr_ = std::make_shared<IMGSubscriber>(nh, "/camera/rgb/image_color", "/camera/depth/image", 1000);
//
//        // 读取参数文件
//        const std::string VocFile = WORK_SPACE_PATH + "/Vocabulary/ORBvoc.bin";
//        // const std::string YamlFile = WORK_SPACE_PATH + "/ros_test/config/D435i.yaml";
//        ros::param::param<std::string>("~yamlfile", yamlfile, "TUM3.yaml");
//        const std::string YamlFile = WORK_SPACE_PATH + "/ros_test/config/" + yamlfile;
//
//        // 读取launch文件中的参数
//        ros::param::param<std::string>("~sensor", sensor, "RGBD");
//        ros::param::param<bool>("~online", semanticOnline, "true");
//        ros::param::param<bool>("~rosbag", rosBagFlag, "false");  //  这是做什么的???/
//
//        ROS_INFO("SUCCESS TO READ PARAM!");
//    }
//
//    ~RGBDMessageFlow_test(){}
//
//    void Run(){
//        if (!ReadData())
//            return;
//
//        while (HasData()) //针对image_color_data_buff_等队列中的图像,进行循环.
//        {
//            if (!ValidData())   //读取
//                continue;
//            double current_time = ros::Time::now().toSec();   //TODO:  这个时间是不对的.应该是从图像中的ros时间戳e获得.因为这个时间会被用在motion track中. 例如:double timestamp = msgImage->header.stamp.toSec();
//            // double current_time = current_image_color_data_.header.stamp.toSec();
//
//            if (sensor == "RGBD")
//                slam_ptr_->TrackRGBD(cvColorImgMat, cvDepthMat, current_time, cvBboxVector);
//            else if (sensor == "MONO")
//                slam_ptr_->TrackMonocular(cvColorImgMat, /* TODO: */current_time);
//        }
//    }
//
//    bool ReadData()
//    {
//        image_sub_ptr_->ParseData(image_color_data_buff_, image_depth_data_buff_, image_bbox_data_buff_);   //这两个变量是双端队列,使用方法与vector相似
//        if (image_color_data_buff_.size() == 0 || image_depth_data_buff_.size() == 0 || image_bbox_data_buff_.size() == 0)
//            return false;
//        return true;
//    }
//
//    bool HasData()
//    {
//        if (image_color_data_buff_.size() == 0)
//            return false;
//        if (image_depth_data_buff_.size() == 0)
//            return false;
//        if (image_bbox_data_buff_.size() == 0)
//            return false;
//        return true;
//    }
//
//    bool ValidData()
//    {
//        double image_time;
//        if (rosBagFlag)
//        {
//            current_image_color_data_ = image_color_data_buff_.front();
//            current_image_depth_data_ = image_depth_data_buff_.front();
//            current_image_bbox_data_ = image_bbox_data_buff_.front();
//            image_color_data_buff_.pop_front();
//            image_depth_data_buff_.pop_front();
//            image_bbox_data_buff_.pop_front();
//        }
//        else
//        {   TODO:
//            current_image_color_data_ = image_color_data_buff_.back();
//            current_image_depth_data_ = image_depth_data_buff_.back();
//            image_color_data_buff_.clear();
//            image_depth_data_buff_.clear();
//        }
//
//        // // 初始时刻不准，扔掉
//        // if (count < 30){
//        //     count++;
//        //     return false;
//        // }
//
//        cv_bridge::CvImagePtr cvImgPtr, cvDepthPtr;
//        std::vector<darknet_ros_msgs::BoundingBox> bboxes;
//        try
//        {
//            cvImgPtr = cv_bridge::toCvCopy(current_image_color_data_, sensor_msgs::image_encodings::BGR8);
//            cvDepthPtr = cv_bridge::toCvCopy(current_image_depth_data_, sensor_msgs::image_encodings::TYPE_16UC1);
//            bboxes = current_image_bbox_data_.bounding_boxes;  //此时还是ros_vector版本,还需要转化为eigen mat版本
//        }
//        catch (cv_bridge::Exception e)
//        {
//            ROS_ERROR_STREAM("CV_bridge Exception:" << e.what());
//            return false;
//        }
//
//        // cv::cvtColor(cvColorImgMat,CurrentGray,CV_BGR2GRAY);
//
//        cvColorImgMat = cvImgPtr->image;
//        cvDepthMat = cvDepthPtr->image;
//
//        //semantic
////    cvBboxMat.resize(bboxes.size(), 5); cvBboxMat = darknetRosMsgToMat(bboxes);
//        cvBboxVector = darknetRosMsgToBoxSE(bboxes);
//        timestamp = current_image_color_data_.header.stamp.toSec();
//        return true;
//    }
//
//    void SaveTrajectory()
//    {
//        slam_ptr_->SaveTrajectoryTUM("CameraTrajectory.txt");
//        slam_ptr_->Shutdown();
//    }
//};
#endif
