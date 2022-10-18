// Update: 2021-4-30 by Lzw
// 为 demo 而修改，融合 ORBSLAM2与YOLO的实时演示

// Update: 20-1-8 by lzw
// 适应真实数据集. 

// Update: 19-12-24 by lzw
// 从ros话题读取检测结果测试椭球体重建效果.

// 该文件作为基本ros模板.

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
using namespace sensor_msgs;

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// 多帧数据同步
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace message_filters;

#include <geometry_msgs/PoseStamped.h>
using namespace geometry_msgs;
int i = 0;
#include <Eigen/Core>

#include <System.h>
#include "Global.h"
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

using namespace std;
std::string WORK_SPACE_PATH = "";
ORB_SLAM2::System* system_ptr;

//std::string DatasetType;

std::vector<BoxSE> darknetRosMsgToBoxSE(std::vector<darknet_ros_msgs::BoundingBox>& boxes){
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
            if (objInfo.id != 0 && objInfo.id != 24 && objInfo.id != 28 && objInfo.id != 39 && objInfo.id != 56 && objInfo.id != 57 && objInfo.id != 58 && objInfo.id != 59 && objInfo.id != 60 && objInfo.id != 62 && objInfo.id != 63 && objInfo.id != 66 && objInfo.id != 67 && objInfo.id != 73
            /* 自己添加 */ && objInfo.id != 72 /* refrigerator */  && objInfo.id != 11 /* stop sign */
            )
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


void GrabImage(const ImageConstPtr& image, const ImageConstPtr& depth, const darknet_ros_msgs::BoundingBoxesConstPtr& bbox);

int main(int argc, char **argv)
{
    WORK_SPACE_PATH = "/home/zhjd/ws_active/src/kinect/EAO-Fusion/";//ros::package::getPath("ros_evo") + "/../";

//(1) ROS
    ros::init(argc, argv, "EllipsoidSLAM");
    ros::NodeHandle nh;

    
//tum_bag:     /camera/rgb/image_color    /camera/depth/image
//kinect_dk和rosbag_correct:  /rgb/image_raw   /depth_to_rgb/image_raw
    message_filters::Subscriber<Image> image_sub(nh, "/rgb/image_raw", 1);
    message_filters::Subscriber<Image> depth_sub(nh, "/depth_to_rgb/image_raw", 1);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbox_sub(nh, "/darknet_ros/bounding_boxes", 1);
//    ros::Subscriber sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>( "/darknet_ros/bounding_boxes" , 10 , GrabBbox );
    typedef message_filters::sync_policies::ApproximateTime
            <Image, Image, darknet_ros_msgs::BoundingBoxes> sync_pol;
    typedef message_filters::sync_policies::ApproximateTime
            <Image, Image> sync_pol_only_image;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), image_sub, depth_sub, bbox_sub);
    message_filters::Synchronizer<sync_pol_only_image> sync_only_image(sync_pol_only_image (10), image_sub, depth_sub);
    

//(2)SYSTEM
    string yamlfile, sensor; bool semanticOnline, rosBagFlag;
    const std::string VocFile = WORK_SPACE_PATH + "/Vocabulary/ORBvoc.bin";
    // const std::string YamlFile = WORK_SPACE_PATH + "/ros_test/config/D435i.yaml";
    ros::param::param<std::string>("~yamlfile", yamlfile, "TUM3_ros.yaml"); /*TUM3.yaml  kinectdk_720.yaml*/
    const std::string YamlFile = WORK_SPACE_PATH + "/ros_test/config/" + yamlfile;
    // 读取launch文件中的参数
    ros::param::param<std::string>("~sensor", sensor, "RGBD");
    
    ros::param::param<bool>("~rosbag", rosBagFlag, "false");  //  这是做什么的???/
    ros::param::param<bool>("~online", semanticOnline, "true");
//    slam_ptr_ = std::make_shared<ORB_SLAM2::System>(VocFile, YamlFile, "Full", ORB_SLAM2::System::RGBD, true, semanticOnline);
    system_ptr = new ORB_SLAM2::System(VocFile, YamlFile, "Full", ORB_SLAM2::System::RGBD, true, semanticOnline);                     


    sync.registerCallback(boost::bind(&GrabImage,_1,_2,_3));
    //    sync_only_image.registerCallback(boost::bind(&GrabImage,_1,_2));
    // 显示窗口
    // cv::namedWindow("rgb", cv::WINDOW_NORMAL);
    // cv::namedWindow("depth", cv::WINDOW_NORMAL);
    std::cout << "Waiting for comming frames..." << std::endl;






    
    ros::spin();
    ros::shutdown();

    return 0;
}

void GrabImage(const ImageConstPtr& msgImage, const ImageConstPtr& msgDepth, const darknet_ros_msgs::BoundingBoxes::ConstPtr & msgBbox)
{
    //  i++;
    //  std::cout << "OpenCV version: "
    //      << CV_MAJOR_VERSION << "."
    //      << CV_MINOR_VERSION << "."
    //      << CV_SUBMINOR_VERSION << "."
    //      << i
    //      << std::endl;

   cv::Mat imRGB, imDepth, imDepth32F;
    double current_time = msgImage->header.stamp.toSec();
    std::cout << std::endl << std::endl;
    std::cout << "[Get a Frame with bbox] Timestamp:" << current_time << std::endl;

    // // 数据处理 简易版
    //  cvImgPtr = cv_bridge::toCvCopy(current_image_color_data_, sensor_msgs::image_encodings::BGR8);
    // cvDepthPtr = cv_bridge::toCvCopy(current_image_depth_data_, sensor_msgs::image_encodings::TYPE_16UC1);
    // bboxes = current_image_bbox_data_.bounding_boxes;  //此时还是ros_vector版本,还需要转化为eigen mat版本


    // 彩色图像
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(msgImage);
        if(!cv_ptrRGB->image.empty())
            imRGB = cv_ptrRGB->image.clone();
        else
        {
            std::cout << "Empty RGB!" << std::endl;
            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 深度图像
    cv_bridge::CvImageConstPtr cv_ptrDepth;
    try
    {
        cv_ptrDepth = cv_bridge::toCvCopy(msgDepth);
        if(!cv_ptrDepth->image.empty())
            imDepth = cv_ptrDepth->image.clone();
        else
        {
            std::cout << "Empty Depth!" << std::endl;
            return;
        }
        // imDepth32F.convertTo(imDepth, CV_16UC1, 1000);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<darknet_ros_msgs::BoundingBox> boxes = msgBbox->bounding_boxes;
    if( boxes.size() == 0 )
    {
        std::cerr << "No detection. " << std::endl;
        // return;  //zhangjiadong  不能return,否则orbslam无法运行.
    }
    std::vector<BoxSE> BboxVector = darknetRosMsgToBoxSE(boxes);

    // mpSLAM->TrackWithObjects(timestamp , cam_pose, bboxMatFiltered, imDepth, imRGB, false);   // 其中 imRGB只用作可视化.
    system_ptr -> TrackRGBD(imRGB, imDepth, current_time, BboxVector);
}


