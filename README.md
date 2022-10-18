# EAO-Fusion
**目标：试图调整 EAO-SLAM ，建立物体级active slam**


### 1 过程记录

**已完成部分：**

+ 将yolo改为ros版本（darknet ros）.并
+ 编写了rosbag timestamp correct程序以修正,相机rosbag和darknet_net的时间戳
+ darknet_ros未识别到物体时，也输出/darknet_ros/bounding_boxes。在darknet_ros源码的YoloObjectDetector.cpp的publishInThread()函数中,修改if (num > 0 && num <= 100)未if (num >= 0 && num <= 100)
+ 


**正在进行的修改：**

+ 运行tum数据集中的long_office，会报错。还没找到问题所在，是否与amd的cpu有关（在旧intel笔记本上没有这个bug）。可以尝试从gcc编译flag、c++版本、eigen版本等方面，试一试。
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
```
