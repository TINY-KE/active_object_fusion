/*
 * @Author: Chen Jiahao
 * @Date: 2021-11-04 14:59:42
 * @LastEditors: Chen Jiahao
 * @LastEditTime: 2021-12-23 21:54:18
 * @Description: file content
 * @FilePath: /catkin_ws/src/EAO-SLAM/ros_test/src/image_subscriber.cc
 */
#include "image_subscriber.h"


IMGSubscriber::IMGSubscriber(ros::NodeHandle &nh, std::string color_topic_name, std::string depth_topic_name, std::string bbox_topic_name, size_t buff_size)
    : it_(nh)
{
    image_sub_color_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image> >(nh, color_topic_name, buff_size);
    image_sub_depth_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image> >(nh, depth_topic_name, buff_size);
    image_sub_bbox_ptr  = std::make_shared<message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> >(nh, bbox_topic_name, buff_size);

    // 备注: 预定义 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    sync_ptr = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(3 * buff_size), *image_sub_color_ptr, *image_sub_depth_ptr, *image_sub_bbox_ptr);

    sync_ptr->registerCallback(boost::bind(&IMGSubscriber::image_callback, this, _1, _2, _3));
    // subscriber_ = nh_.subscribe(topic_name, buff_size, &IMGSubscriber::imu_callback, this);
}

// 回调函数
// 输入：RBG彩色图像和深度图像
void IMGSubscriber::image_callback(const sensor_msgs::ImageConstPtr &msgImg, const sensor_msgs::ImageConstPtr &msgDepth, const darknet_ros_msgs::BoundingBoxes &msgBbox)
{
    new_color_data_.push_back(*msgImg);
    new_depth_data_.push_back(*msgDepth);
    new_bbox_data_.push_back(*msgBbox);
    
    buff_mutex_.unlock();
}

void IMGSubscriber::ParseData(std::deque<sensor_msgs::Image> &image_color_data_buff, std::deque<sensor_msgs::Image> &image_depth_data_buff)
{
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_color_data_.size() > 0)
    {
        image_color_data_buff.insert(image_color_data_buff.end(), new_color_data_.begin(), new_color_data_.end());
        new_color_data_.clear();

        image_depth_data_buff.insert(image_depth_data_buff.end(), new_depth_data_.begin(), new_depth_data_.end());
        new_depth_data_.clear();
    }

    buff_mutex_.unlock();
}
