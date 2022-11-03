/*
 * =============================================
 *      Filename:  Object.h
 *
 *      Description:
 *      Version: 1.0
 *      Created: 09/19/2019
 *      Author: Yanmin Wu
 *      E-mail: wuyanminmax@gmail.com
 * ==============================================
 */

#ifndef OBJECT_H
#define OBJECT_H

#include "System.h"
#include "bitset"
#include "MapPoint.h"
#include <mutex>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
typedef Eigen::Matrix<float,5,1> Vector5f;

//数值计算
#include <numeric>
#include <math.h>
#include <assert.h>
#include <iostream>

namespace ORB_SLAM2
{
    class Frame;
    class MapPoint;
    class KeyFrame;
    class Map;

    // BRIEF the object in current frame.
    class Object_2D
    {
        public:
            int _class_id;      // class id.
            float mScore;       // Probability.

            float left;         // size.
            float right;
            float top;
            float bottom;
            float mWidth;
            float mHeight;

            cv::Rect mBoxRect;              // cv::Rect format.
            cv::Rect mRectFeaturePoints;    // the bounding box constructed by object feature points.
            BoxSE mBox;                     // BoxSE
            cv::Point2f box_center_2d;      // 2D center.

            vector< MapPoint*>  Obj_c_MapPonits;        // object points in current frame.
            cv::Mat _Pos;                               // current object center (3d, world).
            cv::Mat mAssMapObjCenter;                   // map object center.
            float mStandar_x, mStandar_y, mStandar_z;   // standard deviation
            int mCountMappoint;                         // = Obj_c_MapPonits.size().

            int mnId;               // object ID.
            int mnWhichTime;

            int LastAddId;
            cv::Point2f point_center_2d;
            bool mbHaveCube = false;
            // cuboid* mDetectedCube;          // cube slam.

            bool few_mappoint;
            bool bOnEdge;           // on the edge of the image.

            bool First_obj;
            int confidence;
            int add_id;
            bool bad = false;
            bool current = false;

            int nMayRepeat = 0;
            std::map<int, float> mReIdAndIou;   // potential objects.

            vector< MapPoint*>  Obj_k_MapPonits;  // not used.
            vector< MapPoint*>  co_MapPonits;     // not used.
            // vector< MapPoint*>  pro_MapPonits;
            // vector<cv::Mat> pro_MapPoints_camera;

            cv::Mat sum_pos_3d;         // Summation of points observed in the current frame.
            cv::Mat sum_pos_3d_map;     // Summation of points observed in the map.


            void CopyBoxes(const BoxSE &box);           // copy box to object_2d.
            void ComputeMeanAndStandardFrame();         // compute the mean and standard deviation of object points in current frame.
            void RemoveOutliersByBoxPlot(Frame &mCurrentFrame); // remove outliers by boxplot.
            void ObjectDataAssociation(Map* mpMap, Frame &mCurrentFrame, cv::Mat &image, string &flag);    // data association.
            int  NoParaDataAssociation(Object_Map* ObjectMapSingle, Frame &mCurrentFrame, cv::Mat &image); // NP.
            void MergeTwoFrameObj(Object_2D* ObjLastFrame);

        protected:
            std::mutex mMutexFrameObjMapPoints;
    };

    // brief
    struct Cuboid3D
    {
        //     7------6
        //    /|     /|
        //   / |    / |
        //  4------5  |
        //  |  3---|--2
        //  | /    | /
        //  0------1
        // lenth ：corner_2[0] - corner_1[0]
        // width ：corner_2[1] - corner_3[1]
        // height：corner_2[2] - corner_6[2]

        // 8 vertices.
        Eigen::Vector3d corner_1;
        Eigen::Vector3d corner_2;
        Eigen::Vector3d corner_3;
        Eigen::Vector3d corner_4;
        Eigen::Vector3d corner_5;
        Eigen::Vector3d corner_6;
        Eigen::Vector3d corner_7;
        Eigen::Vector3d corner_8;

        // 8 vertices (without rotation).
        Eigen::Vector3d corner_1_w;
        Eigen::Vector3d corner_2_w;
        Eigen::Vector3d corner_3_w;
        Eigen::Vector3d corner_4_w;
        Eigen::Vector3d corner_5_w;
        Eigen::Vector3d corner_6_w;
        Eigen::Vector3d corner_7_w;
        Eigen::Vector3d corner_8_w;

        float x_min, x_max, y_min, y_max, z_min, z_max;     // the boundary in XYZ direction.
        Eigen::Vector3d cuboidCenter;       // the center of the Cube, not the center of mass of the object

        float lenth;
        float width;
        float height;

        g2o::SE3Quat pose;                      // 6 dof pose.
        cv::Mat pose_mat = cv::Mat::eye(4, 4, CV_32F);   //cv::mat形式的 物体在世界坐标系下的位姿
        g2o::SE3Quat pose_without_yaw;          // 6 dof pose without rotation.

        // angle.
        float rotY = 0.0;
        float rotP = 0.0;
        float rotR = 0.0;

        float mfRMax;

        // line.
        float mfErrorParallel;
        float mfErroeYaw;
    };

    // BRIEF 3d object in the map.
    class Object_Map
    {
        public:
            Object_Map(){
                init_information_entroy();
            }   //zhangjiadong 用于nbv test
            std::vector<Object_2D*> mObjectFrame;
            cv::Rect mLastRect;
            cv::Rect mLastLastRect;
            cv::Rect mPredictRect;
            cv::Rect mRectProject;
            int mnId;   //全局的id
            int mnClass;
            int mnConfidence; // zhangjiaddong:  等于 被多少帧看到.  那岂不是等于mObjectFrame.size()
            bool mbFirstObserve;
            int mnAddedID;
            int mnLastAddID;
            int mnLastLastAddID;
            std::set<int> msFrameId;
            vector< MapPoint*> mvpMapObjectMappoints;
            vector< MapPoint*> mvpMapCurrentNewMappoints;

            cv::Mat mSumPointsPos;
            cv::Mat mCenter3D;
            float mStandar_x, mStandar_y, mStandar_z;
            float mCenterStandar_x, mCenterStandar_y, mCenterStandar_z;
            float mCenterStandar;

            int nMayRepeat = 0;                 // maybe a repeat object.
            std::map<int, int> mReObj;          // potential associated objects.
            std::map<int, int> mmAppearSametime;// object id and times simultaneous appearances .

            bool bBadErase = false;    //zhangjiadong  用途：（1）如果为true，则不view  （2）在localMapping、 等地方，应用

            Cuboid3D mCuboid3D;                  // cuboid.
            vector<cv::Mat> mvPointsEllipsoid;   // not used.

            std::vector<Vector5f> mvAngleTimesAndScore;    // Score of sampling angle.

            void ComputeMeanAndStandard();
            void IsolationForestDeleteOutliers();
            bool DataAssociateUpdate(   Object_2D* ObjectFrame,
                                        Frame &mCurrentFrame,
                                        cv::Mat &image,
                                        int Flag);

            void ComputeProjectRectFrame(cv::Mat &image, Frame &mCurrentFrame);
            void WhetherMergeTwoMapObjs(Map *mpMap);
            void MergeTwoMapObjs(Object_Map *RepeatObj);
            bool DoubleSampleTtest(Object_Map *RepeatObj);
            void DealTwoOverlapObjs(Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
            bool WhetherOverlap(Object_Map *CompareObj);
            void BigToSmall(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z);
            void DivideEquallyTwoObjs(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z);

            // void UpdateObjScale(Eigen::Vector3d Scale);    // for optimization.
            void UpdateObjPose();      // update object pose.

        protected:
            std::mutex mMutexMapPoints;
            std::mutex mMutex;


        public:
            void init_information_entroy() {
                //mCuboid3D = object->mCuboid3D;
                //mvpMapObjectMappoints = object->mvpMapObjectMappoints;

                vector<double> InforEntroy_single(18, 0.693147) ;
                vector<vector<double> > InforEntroy( 18, InforEntroy_single);   vInforEntroy = InforEntroy;

                vector<double> grid_prob_single(18, 0.5) ;
                vector<vector<double> > grid_prob( 18, grid_prob_single);   vgrid_prob = grid_prob;

                vector<int> pointnum_eachgrid_single(18, 0.0) ;
                vector<vector<int> >  pointnum_eachgrid(18, pointnum_eachgrid_single);        vpointnum_eachgrid = pointnum_eachgrid;
            }

            //vector< MapPoint*> mvpMapObjectMappoints;
            //Cuboid3D mCuboid3D;

            int threshold = 3;  //用于判定, 某列grid是否有效

            Vector3d mMainDirection;  //通过特征点计算的主方向,用于view的 ie

            //传感器模型
            double P_occ=0.6;
            double P_free=0.4;
            double P_prior=0.5;

            //占据状态值

            //
            vector<vector<double> > vInforEntroy;  // 没用.  用于存储18*18个栅格的信息熵
            vector<vector<double> > vgrid_prob;  //用于存储18*18个栅格的占据概率
            vector<vector<int> > vpointnum_eachgrid;   //存储18*18个栅格中,各自的grid数量

            void grid_index(const Vector3d &zero_vec, const Vector3d &point_vec, int& x, int& y){
                Vector3d v1(zero_vec(0),zero_vec(1),0.0), v2(point_vec(0),point_vec(1),0.0);
                double cosValNew = v1.dot(v2) / (v1.norm()*v2.norm()); //通过向量的点乘, 计算角度cos值
                double angleNew = acos(cosValNew) * 180 / M_PI;     //弧度角
                if(point_vec(1) >= 0)
                    x = floor(angleNew/20.0);
                else
                    x = floor((360.0-angleNew)/20.0);

                y = floor(
                            (  (point_vec(2)  + mCuboid3D.height/2 ) /mCuboid3D.height) * 18
                        );
                std::cout<<"[计算y]"  <<point_vec(2)   <<", 中心z " <<  mCuboid3D.cuboidCenter[2] <<",  cube高度 "<< mCuboid3D.height/2 <<std::endl;
            }

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
                        std::cout<<"compute grid index: ERROR:i "<<i<<", x "<<x<<", y "<<y<<std::endl;
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

            double information_entroy(const double &p){
                return -1*p*log(p) - (1-p)*log(1-p);
            }

            void compute_information_entroy(){
                compute_pointnum_eachgrid();

                compute_occupied_prob();

                //计算各grid的信息熵
                for(int x=0; x<18; x++)
                    for(int y=0; y<18; y++)
                        vInforEntroy[x][y] = information_entroy(vgrid_prob[x][y]);

                //计算主向量
                double main_x, main_y, main_z;
                for (int i = 0; i < mvpMapObjectMappoints.size(); ++i) {
                    cv::Mat point_pose = mvpMapObjectMappoints[i]->GetWorldPos();
                    main_x +=  point_pose.at<float>(0);
                    main_y +=  point_pose.at<float>(1);
                    main_z +=  point_pose.at<float>(2);
                }
                double normalize = sqrt( main_x*main_x + main_y*main_y + main_z*main_z );
                main_x = main_x/normalize;
                main_y = main_y/normalize;
                main_z = main_z/normalize;
                mMainDirection =  Vector3d(main_x, main_y, main_z);

                //记录栅格的状态

            }


            double get_information_entroy(){
                double entroy = 0;
                for(int x=0; x<18; x++)
                    for(int y=0; y<18; y++){
                        entroy += vInforEntroy[x][y];
                    }
                return entroy/(18.0*18.0);
            }




    };
}
#endif //OBJECT_H
