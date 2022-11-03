/**
* This file is part of ORB-SLAM2.
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* Modification: EAO-SLAM
* Version: 1.0
* Created: 05/21/2019
* Author: Yanmin Wu
* E-mail: wuyanminmax@gmail.com
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer( System* pSystem, FrameDrawer *pFrameDrawer,
                MapDrawer *pMapDrawer, Tracking *pTracking,
                const string &strSettingPath, const string &flag):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false), mflag(flag)
{
    mpMapPub = new MapPublisher(pMapDrawer -> mpMap); //[rviz]
    mpMapPub -> CurrentCameraPose = &(pMapDrawer->mCameraPose); //[rviz]  TODO: 很丑陋的实现方法，最好是把MapPublisher的指针传入到tracker中，直接修改mpMapPub -> CurrentCameraPose【在init和track时】
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    mfx = fSettings["Camera.fx"];
    mfy = fSettings["Camera.fy"];
    mcx = fSettings["Camera.cx"];
    mcy = fSettings["Camera.cy"];

    run_pangolin = fSettings["Viewer.pangolin"];
    run_rviz = fSettings["Viewer.rviz"];

    read_local_object = fSettings["Viewer.readlocalobject"];
}

void Viewer::Run()
{
    mbFinished = false;

    if(run_pangolin)
        pangolin::CreateWindowAndBind("Object Map Viewer",1200, 900);   // 1920,1080.
    else
        pangolin::CreateWindowAndBind("Object Map Viewer",12, 9);
        // pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",mImageWidth+175,mImageHeight);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowCamera("menu.Show Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<double> menuSigmaTH("menu.Sigma",0.02,1e-10,0.05,false);
    pangolin::Var<bool> menuCameraView("menu.Camera View",true,true);

    pangolin::Var<bool> menuShowQuadricObj("menu.Show QuadricObj",true,true);

    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);



    // add plane
    pangolin::Var<bool> menuShowPlanes("menu.Show Planes", true, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            // carv: using calibrated camera center and focal length
            pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.1,1000),
            pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
//            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -mImageWidth/mImageHeight)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("Raw Image");
    cv::moveWindow("Raw Image", 40, 20);
    cv::namedWindow("Point, Line and Object Detection");
    cv::moveWindow("Point, Line and Object Detection", 40, 360);
    cv::namedWindow("Quadric Projection");
    cv::moveWindow("Quadric Projection", 40, 710);

    bool bFollow = true;
    bool bLocalizationMode = false;

    // carv: camera close up view
    bool bCameraView = true;
    pangolin::OpenGlMatrix projectionAbove = pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mViewpointF,mViewpointF,
                                                                        mImageWidth/2,mImageHeight/2,0.1,1000);
    pangolin::OpenGlMatrix projectionCamera = pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.1,1000);
    pangolin::OpenGlMatrix viewAbove = pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0);
    pangolin::OpenGlMatrix viewCamera = pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0.0,-1.0, 0.0);

    if(read_local_object)
        read_local_object_file();
    while(1)
    {

        if(run_rviz)
            mpMapPub -> Refresh();   //[rviz]

        if(read_local_object)
        {
            //std::cout<<"[nbv debug] publish local object"<<std::endl;
            mpMapPub ->PublishObject(vObjects);
            mpMapPub ->PublishIE(vObjects);
        }

        if(run_pangolin){
            //pangolin
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);    //[rviz]  Twc是怎么获得的??  可用在rviz的当前帧显示中.

            if(menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if(menuFollowCamera && !bFollow)
            {
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if(menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if(!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            // carv: setup viewpoint to see model
            if(menuCameraView && !bCameraView)
            {
                s_cam.SetProjectionMatrix(projectionCamera);
                s_cam.SetModelViewMatrix(viewCamera);
                bCameraView = true;
            }
            else if(!menuCameraView && bCameraView)
            {
                s_cam.SetProjectionMatrix(projectionAbove);
                s_cam.SetModelViewMatrix(viewAbove);
                bCameraView = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            if(menuShowCamera)
                mpMapDrawer->DrawCurrentCamera(Twc);
            if(menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
            if(menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            // step draw objects.
            if( menuShowQuadricObj)
            {
                mpMapDrawer->DrawObject(menuShowQuadricObj,
                                        mflag);
            }
            // add plane
            if (menuShowPlanes)
                mpMapDrawer->DrawMapPlanesOld();
            // add plane end

            pangolin::FinishFrame();
        }


        // gray image.
        cv::Mat im = mpFrameDrawer->DrawFrame();
        if(!im.empty())
        {
            cv::Mat resizeimg;
            cv::resize(im, resizeimg, cv::Size(640*0.7, 480*0.7), 0, 0, cv::INTER_CUBIC);
            cv::imshow("Point, Line and Object Detection", resizeimg);
        }

        // color image.
        cv::Mat RawImage = mpFrameDrawer->GetRawColorImage();
        if(!RawImage.empty())
        {
            cv::Mat resizeimg;
            cv::resize(RawImage, resizeimg, cv::Size(640*0.7, 480*0.7), 0, 0, cv::INTER_CUBIC);
            cv::imshow("Raw Image", resizeimg);
        }

        // quadric image.
        cv::Mat QuadricImage = mpFrameDrawer->GetQuadricImage();
        if(!QuadricImage.empty())
        {
            cv::Mat resizeimg;
            cv::resize(QuadricImage, resizeimg, cv::Size(640*0.7, 480*0.7), 0, 0, cv::INTER_CUBIC);
            cv::imshow("Quadric Projection", resizeimg);
        }



        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            // add plane --------------------
            menuShowPlanes = true;
            // add plane end -------------------
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;

            menuCameraView = true;


            mpSystem->Reset();
            menuReset = false;
        }


        cv::waitKey(mT);

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::read_local_object_file() {
    std::string filePath = "/home/zhjd/ws_active/src/kinect/EAO-Fusion/eval/Objects_with_points_for_read.txt";
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
    vObjects.clear();
    string line;
    int object_num = -1;
    int type = 1;
    while (getline(infile, line))
    {   //std::cout<<line<<std::endl;
        istringstream istr(line);
        istr >> type;

        if( type == 1){
            Object_Map *obj = new Object_Map();
            object_num ++;
//            std::cout<<"物体"<<object_num<<std::endl;
            double temp;
            istr >> temp;    obj->mnId = temp;
            istr >> temp;    obj->mnClass = temp;
            istr >> temp;    obj->mnConfidence = temp;
            istr >> temp ;  //物体中特征点的数量

            Eigen::MatrixXd object_poses(1, 8); ;
            istr >> temp;  object_poses(0) = temp;  obj->mCuboid3D.cuboidCenter(0) = temp;
            istr >> temp;  object_poses(1) = temp;  obj->mCuboid3D.cuboidCenter(1) = temp;
            istr >> temp;  object_poses(2) = temp;  obj->mCuboid3D.cuboidCenter(2) = temp;
            istr >> temp;  object_poses(3) = temp;
            istr >> temp;  object_poses(4) = temp;
            istr >> temp;  object_poses(5) = temp;
            istr >> temp;  object_poses(6) = temp;
            g2o::SE3Quat cam_pose_se3(object_poses.row(0).head(7));

            obj->mCuboid3D.pose = cam_pose_se3;
            istr >> temp;   obj->mCuboid3D.lenth = temp;
            istr >> temp;   obj->mCuboid3D.width = temp;
            istr >> temp;   obj->mCuboid3D.height = temp;

            cmpute_corner(obj);

            vObjects.push_back( obj );

            std::cout<<  "mnId: "<<vObjects[ object_num ]->mnId
                    <<  ", Class: " << vObjects[ object_num ]->mnClass <<std::endl;

        }
        else if( type == 0)
        {
//            std::cout<<"特征点"<<object_num<<std::endl;
            double temp;
            istr >> temp;
            istr >> temp;

            MapPoint* point = new MapPoint();
            float x_p, y_p, z_p;
            istr >> temp;  x_p = temp;
            istr >> temp;  y_p = temp;
            istr >> temp;  z_p = temp;
            std::vector<float> vec{x_p, y_p, z_p};
            cv::Mat WorldPos(vec);

            point->SetWorldPos(WorldPos) ;
            vObjects[ object_num ]-> mvpMapObjectMappoints.push_back( point );
//            mpMapPub -> mpMap->mvObjectMap[ object_num ]->mvpMapObjectMappoints.push_back( &point );
        }


        row.clear();
        type = -1;
        istr.clear();
        line.clear();
    }

    for(int i=0; i<vObjects.size(); i++){
        vObjects[i]->compute_information_entroy();
    }

}

    void Viewer::cmpute_corner(Object_Map* object) {

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

}
