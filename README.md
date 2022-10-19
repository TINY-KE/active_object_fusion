# EAO-Fusion
**目标：试图调整 EAO-SLAM ，建立物体级active slam**


### 1 过程记录

**TODO：**

+ 在kinect dk IMU上使用. 利用ros姿态的imu tool求解imu_link的位姿，并根据imu_link和rgb_camera_link的变换关系。（利用ros tf变换为rgb_camera_link的位姿。错）
+ 在实际场景中尝试:鼠标,笔记本,桌子,椅子,
+ 


**已完成部分：**

+ 将yolo改为ros版本（darknet ros）.并
+ 编写了rosbag timestamp correct程序以修正,相机rosbag和darknet_net的时间戳
+ darknet_ros未识别到物体时，也输出/darknet_ros/bounding_boxes。在darknet_ros源码的YoloObjectDetector.cpp的publishInThread()函数中,修改if (num > 0 && num <= 100)未if (num >= 0 && num <= 100)
+ ConstraintType赋值为0时,设定初始帧的位姿为重力方向。 0: no constraint, first frame is vertical to ground; 1: use ground truth; 2: use imu.
+ 


**正在进行的修改：**

+ 运行tum数据集中的long_office，会报错。还没找到问题所在，是否与amd的cpu有关（在旧intel笔记本上没有这个bug）。可以尝试从gcc编译flag、c++版本、eigen版本等方面，试一试。
+ 利用imu_tools中的imu_complementary_filter_node软件包,滤波kinect dk的imu, 融合出相机的位姿.并作为相机的初始位姿.
+ 
+ 

**未完成部分：**

+ 

### 2 环境构建

```yaml
Ubuntu: 18.04
opencv: 3.4.10
Eigen: 3.2.1 ?? 
darknet_net: 
rapidjson: 1.1.0 
boost: 1.65.1
```
