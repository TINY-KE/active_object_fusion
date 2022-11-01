

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


// #include <tf/Quat>
using namespace geometry_msgs;


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

#include "Object.h"
#include "MapPoint.h"
using namespace std;
std::string WORK_SPACE_PATH = "";
ORB_SLAM2::System* system_ptr;

//std::string DatasetType;
Eigen::Matrix4d INIT_POSE = Eigen::Matrix4d::Identity();

int main(int argc, char **argv)
{
    WORK_SPACE_PATH = "/home/zhjd/ws_active/src/kinect/EAO-Fusion/";//ros::package::getPath("ros_evo") + "/../";

//(1) ROS
    ros::init(argc, argv, "EllipsoidSLAM");
    ros::NodeHandle nh;
    std::cout << "Waiting for comming frames..." << std::endl;


    std::string filePath = "/home/zhjd/ws_active/src/kinect/EAO-Fusion/eval/Objects_with_points.txt";
    ifstream infile(filePath, ios::in);
    if (!infile.is_open())
    {
        cout << "open fail: "<< filePath <<" " << endl;
        exit(233);
    }
    else
    {
        std::cout << "read Objects_with_points.txt" << std::endl;
    }

    vector<double> row;

    cv::Mat cam_pose_mat;
    int mnid_current = -1;
//    string s0;
//    getline(infile, s0);  注销掉无用的line
    std::vector<ORB_SLAM2::Object_Map*> Objects;
    double tmp;
    string line;
    int object_num = -1;
    int type = 1;
    while (getline(infile, line))
    {
        istringstream istr(line);
        istr >> type;
        std::cout<<line<<std::endl;
        if( type == 1){

            ORB_SLAM2::Object_Map obj;
            object_num ++;
//            std::cout<<"物体"<<object_num<<std::endl;
            double temp;
            istr >> temp;    obj.mnId = temp;
            istr >> temp;    obj.mnClass = temp;
            istr >> temp;    obj.mnConfidence = temp;
            istr >> temp ;  //物体中特征点的数量

            Eigen::MatrixXd object_poses(1, 8); ;
            istr >> temp;  object_poses(0) = temp;
            istr >> temp;  object_poses(1) = temp;
            istr >> temp;  object_poses(2) = temp;
            istr >> temp;  object_poses(3) = temp;
            istr >> temp;  object_poses(4) = temp;
            istr >> temp;  object_poses(5) = temp;
            istr >> temp;  object_poses(6) = temp;
            g2o::SE3Quat cam_pose_se3(object_poses.row(0).head(7));

            obj.mCuboid3D.pose = cam_pose_se3;
            istr >> temp;   obj.mCuboid3D.lenth = temp;
            istr >> temp;   obj.mCuboid3D.width = temp;
            istr >> temp;   obj.mCuboid3D.height = temp;

            Objects.push_back( &obj );
        }
        else if( type == 0)
        {
//            std::cout<<"特征点"<<object_num<<std::endl;
            bool notbad; istr >> notbad;
            int id;      istr >> id;
            ORB_SLAM2::MapPoint point;
            double temp, x_p, y_p, z_p;
            istr >> temp;  x_p = temp;
            istr >> temp;  y_p = temp;
            istr >> temp;  z_p = temp;
            std::vector<double> vec{x_p, y_p, z_p};
            cv::Mat WorldPos(vec);
//            std::cout<<  WorldPos  <<std::endl;
            point.SetWorldPos(WorldPos) ;
            Objects[ object_num ]->mvpMapObjectMappoints.push_back( &point );
        }


        row.clear();
        type = -1;
        istr.clear();
        line.clear();
    }



    infile.close();

    ros::spin();
    ros::shutdown();
    // Save camera trajectory

    return 0;
}
