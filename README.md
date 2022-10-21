# EAO-Fusion
**目标：试图调整 EAO-SLAM ，建立物体级active slam**


### 1 过程记录
``
**TODO：**

+ 在实际场景中尝试:鼠标,笔记本,桌子,椅子,
+ 雷达地图和roseao的融合
+ 如何提取,物体中的point
+ 编写gazebo的自主仿真模型.
+ 如何去除的真值约束? 对比加上和不加垂直方向的初始约束的效果；
+ 重写plane的优化.平面的系数转到世界坐标系下是H^{-T}``
+ virtual void setToOriginImpl() { _estimate = Plane3D() ;}   //?????

``
**已完成部分：**

+ 将yolo改为ros版本（darknet ros）,并通过ros message_filters::Synchronizer进行输入.
+ 编写了rosbag timestamp correct程序以修正,相机rosbag和darknet_net的时间戳
+ darknet_ros未识别到物体时，也输出/darknet_ros/bounding_boxes。在darknet_ros源码的YoloObjectDetector.cpp的publishInThread()函数中,修改if (num > 0 && num <= 100)未if (num >= 0 && num <= 100)
+ 在config文件中,ConstraintType为0时,设定初始帧的位姿为重力方向。
+ 在config文件中,ConstraintType为2时,根据kinect中的IMU生成真实的重力方向. 注意:IMU输出的位姿orientation,是imu坐标系在世界坐标系下的姿态, 即:世界坐标系到imu坐标系的旋转变换关系.
+ 时间测试: 可知yolo处理一帧图像是0.07秒（14hz），ros_eao处理一帧(提取特征点和特征面)是0.076秒。
+ 


**未来想进行的修改 和 学习的内容：**

+ 运行tum数据集中的long_office，会报错。还没找到问题所在，是否与amd的cpu有关（在旧intel笔记本上没有这个bug）。可以尝试从gcc编译flag、c++版本、eigen版本等方面，试一试。
+ 有时候相机会卡死, 推测可能是message_filters::Synchronizer的问题. 重启相机后,可修复.
+ 怎么感觉frame.h中多生命了一个构造函数??  c文件里面还重复写了一个rgbd的frame的构造函数
+ KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);  关键帧里的词袋是怎么生成和使用的?
+ plane在loop close中是怎么起作用的??  发现和point是一样的,连变量名字都是一样的.  https://blog.csdn.net/YGG12022/article/details/124958961 . 
+ plane在回环检测和优化中,起到了重要作用, 因此我们把这个平面也作为一个重要的观测对象.
+ PEAC方法中,深度信息只用了0.2~4之间.是否妥当
+ mspMapPlanes中的平面 是什么时候添加进去的?
+ 特征点的UpdateNormalAndDepth 是什么用法??
+ 平面的误差函数  是怎么构建的????

### 2 知识学习
+ PEAC平面提取方法介绍 --- ComputePlanesFromPEAC(const cv::Mat &imDepth)
  + 对深度图降采样: imDepth --> iCloud
  + ahc::PlaneFitter 从iCloud中提取平面. 平面在程序中的应用方法是:
    + plane_vertices_ :  plane_vertices_.at(i) 存储了i平面中各point, 在icloud(深度图像indepth间隔采样的结果)中的像素的索引.
    + extractedPlanes :  extractedPlanes.at(i) 存储了i平面的法向量,中心坐标.
  + 根据plane_vertices_中的像素索引, 从icloud中提取点云:   icloud --> inputCloud
  + 利用pcl::VoxelGrid对inputCloud,进行体素化的下采样:  inputCloud -->  dcoarseCloud
  + 将平面的点云信息dcoarseCloud, 添加进frame的mvPlanePoints中.   平面的法向量和坐标coef,添加进frame的mvPlaneCoefficients中
+ 平面参与优化 --- https://blog.csdn.net/supengufo/article/details/105846129
  + 读取地图或者临近帧的平面: vector<MapPlane *> vpMPl = pMap->GetAllMapPlanes();
  + 误差函数的构建:
    + 
  + 构建g2o的因子图:
    + 将vpMPl[i],添加为构建g2o中的点 VertexPlane.
    + 将vpMPl[i]->GetObservations()->first ,即观测到此平面的关键帧, 添加为g2o中的点. [这一步其实已经在BundleAdjustment的关键帧处理部分,已经完成了]
    + 添加边: optimizer.addEdge(edge);
    + 正式开始优化 optimizer.optimize(nIterations);
    + 将优化的结果, 赋值给plane. 
  + 
+ 虚函数: 指向基类的指针在操作它的多态类对象时，会根据不同的类对象，调用其相应的函数，这个函数就是虚函数。

### 3 环境构建

```yaml
Ubuntu: 18.04
opencv: 3.4.10
Eigen: 3.2.1 ?? 
darknet_net: 
rapidjson: 1.1.0 
boost: 1.65.1
```
