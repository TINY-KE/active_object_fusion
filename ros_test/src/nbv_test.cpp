

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

//数值计算
#include <numeric>
#include <math.h>
#include <assert.h>

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

namespace ORB_SLAM2
{
class InformationEntroy {
    public:
        InformationEntroy() {}
        InformationEntroy(Object_Map* object) {
            mCuboid3D = object->mCuboid3D;
            mvpMapObjectMappoints = object->mvpMapObjectMappoints;

            vector<double> InforEntroy_single(18, 0.693147) ;
            vector<vector<double> > InforEntroy( 18, InforEntroy_single);   vInforEntroy = InforEntroy;

            vector<double> grid_prob_single(18, 0.5) ;
            vector<vector<double> > grid_prob( 18, grid_prob_single);   vgrid_prob = grid_prob;

            vector<int> pointnum_eachgrid_single(18, 0.0) ;
            vector<vector<int> >  pointnum_eachgrid(18, pointnum_eachgrid_single);        vpointnum_eachgrid = pointnum_eachgrid;
        }

        vector< MapPoint*> mvpMapObjectMappoints;
        Cuboid3D mCuboid3D;

        int threshold = 3;  //用于判定, 某列grid是否有效
        double P_occ=0.6;
        double P_free=0.4;
        double P_prior=0.5;

        vector<vector<double> > vInforEntroy;  // 没用.  用于存储18个栅格的信息熵
        vector<vector<double> > vgrid_prob;  //用于存储18*18个栅格的占据概率
        vector<vector<int> > vpointnum_eachgrid;   //存储18*18个栅格中,各自的grid数量

        void compute_pointnum_eachgrid(){
            double center_x = mCuboid3D.cuboidCenter(0);
            double center_y = mCuboid3D.cuboidCenter(1);
            double center_z = mCuboid3D.cuboidCenter(2);
            //g2o::SE3Quat pose = mCuboid3D.pose;
            //Eigen::Isometry3d T_w2o = fromSE3Quat(mCuboid3D.pose);
            cv::Mat T_w2o_mat = Converter::toCvMat(mCuboid3D.pose);
            //Eigen::MatrixXd T_w2o_eigen = Eigen::toEigenMatrixXd(T_w2o_mat);
            Eigen::Isometry3d T_w2o = ORB_SLAM2::Converter::toSE3Quat(T_w2o_mat);
            Vector3d zero_vec( 1,0,0);
            zero_vec = T_w2o* zero_vec;
            for (int i = 0; i < mvpMapObjectMappoints.size(); ++i) {
                cv::Mat point_pose = mvpMapObjectMappoints[i]->GetWorldPos();
                Vector3d point_vec( point_pose.at<float>(0)-center_x, point_pose.at<float>(1)-center_y, point_pose.at<float>(2)-center_z);
                int x = -1 , y = -1;
                grid_index(zero_vec, point_vec,  x,  y);
                if( x>=0 && x<=18 && y>=0 && y<=18 ) {
                    vpointnum_eachgrid[x][y] += 1;
                }
                else{
                    std::cout<<"compute grid index: ERROR"<<std::endl;
                }
            }
        }

        void compute_occupied_prob(){

            for(int x=0; x<18; x++){
                int num = accumulate(vpointnum_eachgrid[x].begin(), vpointnum_eachgrid[x].end(), 0);
                if(num > threshold){
                    //当前列的观测到认为有效，即当前列的grid，认为是free或occupied
                    for(int y=0; y<18; y++){
                        double lnv_p ;
                        if(vpointnum_eachgrid[x][y] == 0){
                            //free
                            //lnv_p = log(vgrid_prob[x][y]) + log(P_free) -log(0.5);
                            // 当前只更新一次，之后对物体内的point进行“是否为新添加的更新”，再进行增量更新
                            lnv_p = log(0.5) + log(P_free) -log(0.5);
                        }
                        else{
                            //occupied
                            //lnv_p = log(vgrid_prob[x][y]) + log(P_occ) -log(0.5);
                            // 当前只更新一次，之后对物体内的point进行“是否为新添加的更新”，再进行增量更新
                            lnv_p = log(0.5) + log(P_occ) -log(0.5);
                        }
                        vgrid_prob[x][y] = exp(lnv_p);
                    }
                }
                else{
                    ///当前列的观测到认为无效，即当前列的grid，认为是unknown
                     for(int y=0; y<18; y++)
                         //unkonwn
                         vgrid_prob[x][y] = 0.5;
                }

            }
        }

        void compute_information_entroy(){
            for(int x=0; x<18; x++)
                for(int y=0; y<18; y++)
                    vInforEntroy[x][y] = information_entroy(vgrid_prob[x][y]);
        }

        double information_entroy(const double &p){
            return -1*p*log(p) - (1-p)*log(1-p);
        }

        double get_information_entroy(){
            double entroy = 0;
            for(int x=0; x<18; x++)
                for(int y=0; y<18; y++){
                    entroy += vInforEntroy[x][y];
                }
            return entroy/(18.0*18.0);
        }

        void grid_index(const Vector3d &zero_vec, const Vector3d &point_vec, int& x, int& y){
            Vector3d v1(zero_vec(0),zero_vec(1),0.0), v2(point_vec(0),point_vec(1),0.0);
            double cosValNew = v1.dot(v2) / (v1.norm()*v2.norm()); //通过向量的点乘, 计算角度cos值
            double angleNew = acos(cosValNew) * 180 / M_PI;     //弧度角
            if(point_vec(1) >= 0)
                x = floor(angleNew/20.0);
            else
                x = floor((360.0-angleNew)/20.0);

            y = floor(((point_vec(2)-(-1/2*mCuboid3D.height))/mCuboid3D.height) * 18 );
        }
    };
}
//std::string DatasetType;
Eigen::Matrix4d INIT_POSE = Eigen::Matrix4d::Identity();


void cmpute_corner(ORB_SLAM2::Object_Map* object) {

    float x_min_obj = (-0.5)*object->mCuboid3D.lenth;
    float x_max_obj = (0.5)*object->mCuboid3D.lenth;
    float y_min_obj = (-0.5)*object->mCuboid3D.width;
    float y_max_obj = (0.5)*object->mCuboid3D.width;
    float z_min_obj = (-0.5)*object->mCuboid3D.height;
    float z_max_obj = (0.5)*object->mCuboid3D.height;

    object->mCuboid3D.corner_1 = object->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    object->mCuboid3D.corner_2 = object->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    object->mCuboid3D.corner_3 = object->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    object->mCuboid3D.corner_4 = object->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    object->mCuboid3D.corner_5 = object->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    object->mCuboid3D.corner_6 = object->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    object->mCuboid3D.corner_7 = object->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    object->mCuboid3D.corner_8 = object->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);
}

void compute_information_entropy(ORB_SLAM2::Object_Map* object){

}
int main(int argc, char **argv)
{
    WORK_SPACE_PATH = "/home/zhjd/ws_active/src/kinect/EAO-Fusion/";//ros::package::getPath("ros_evo") + "/../";

//(1) ROS
    ros::init(argc, argv, "EllipsoidSLAM");
    ros::NodeHandle nh;

// (2)读取本地文件
//    std::string filePath = "/home/zhjd/ws_active/src/kinect/EAO-Fusion/eval/Objects_with_points_for_read.txt";
//    ifstream infile(filePath, ios::in);
//    if (!infile.is_open())
//    {
//        cout << "open fail: "<< filePath <<" " << endl;
//        exit(233);
//    }
//    else
//    {
//        std::cout << "read Objects_with_points.txt" << std::endl;
//    }
//
//    vector<double> row;
//
//    cv::Mat cam_pose_mat;
//    int mnid_current = -1;
////    string s0;
////    getline(infile, s0);  注销掉无用的line
//    std::vector<ORB_SLAM2::Object_Map*> vObjects;
//    vObjects.clear();
//    string line;
//    int object_num = -1;
//    int type = 1;
//    while (getline(infile, line))
//    {   //std::cout<<line<<std::endl;
//        istringstream istr(line);
//        istr >> type;
//
//        if( type == 1){
//            ORB_SLAM2::Object_Map *obj = new ORB_SLAM2::Object_Map();
//            object_num ++;
////            std::cout<<"物体"<<object_num<<std::endl;
//            double temp;
//            istr >> temp;    obj->mnId = temp;
//            istr >> temp;    obj->mnClass = temp;
//            istr >> temp;    obj->mnConfidence = temp;
//            istr >> temp ;  //物体中特征点的数量
//
//            Eigen::MatrixXd object_poses(1, 8); ;
//            istr >> temp;  object_poses(0) = temp;
//            istr >> temp;  object_poses(1) = temp;
//            istr >> temp;  object_poses(2) = temp;
//            istr >> temp;  object_poses(3) = temp;
//            istr >> temp;  object_poses(4) = temp;
//            istr >> temp;  object_poses(5) = temp;
//            istr >> temp;  object_poses(6) = temp;
//            g2o::SE3Quat cam_pose_se3(object_poses.row(0).head(7));
//
//            obj->mCuboid3D.pose = cam_pose_se3;
//            istr >> temp;   obj->mCuboid3D.lenth = temp;
//            istr >> temp;   obj->mCuboid3D.width = temp;
//            istr >> temp;   obj->mCuboid3D.height = temp;
//
//            cmpute_corner(obj);
//
//            vObjects.push_back( obj );
//
//            std::cout<<  "mnId: "<<vObjects[ object_num ]->mnId
//                     <<  ", Class: " << vObjects[ object_num ]->mnClass <<std::endl;
//
//        }
//        else if( type == 0)
//        {
////            std::cout<<"特征点"<<object_num<<std::endl;
//            double temp;
//            istr >> temp;
//            istr >> temp;
//
//            ORB_SLAM2::MapPoint* point = new ORB_SLAM2::MapPoint();
//            float x_p, y_p, z_p;
//            istr >> temp;  x_p = temp;
//            istr >> temp;  y_p = temp;
//            istr >> temp;  z_p = temp;
//            std::vector<float> vec{x_p, y_p, z_p};
//            cv::Mat WorldPos(vec);
//
//            point->SetWorldPos(WorldPos) ;
//            vObjects[ object_num ]-> mvpMapObjectMappoints.push_back( point );
////            mpMapPub -> mpMap->mvObjectMap[ object_num ]->mvpMapObjectMappoints.push_back( &point );
//        }
//
//
//        row.clear();
//        type = -1;
//        istr.clear();
//        line.clear();
//    }




    ros::spin();
    ros::shutdown();
    // Save camera trajectory

    return 0;
}
