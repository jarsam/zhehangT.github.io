---
title: VINS-Mono中的Feature Tracker
date: 2018-04-18 10:11:25
tags:
- VINS-Mono
- 源码学习
categories:
- 机器人事业
- SLAM
description: VINS-Mono中的Feature Tracker相关代码学习
---
<!-- more -->

# 摘要
VINS-Mono中采用光流法作为视觉前端，从实验效果上来看，追踪的成功率非常的高，虽然牺牲了一定的精确度，但不会像ORB-SLAM那样容易丢失。本篇博客将对VINS-Mono的前端代码做简单的梳理。


# 数据结构
VINS-Mono以 ROS 作为代码开发的基础，与前端相关的代码都在feature_tracker目录下。最主要的流程就是通过订阅图像的Topic，获取图像后进行处理。处理得到了图像与图像之间的匹配关系，然后通过消息发布。这里涉及到一个很重要的数据结构就是`sensor_msgs::PointCloud`。其定义如下：
```
std_msgs/Header header
geometry_msgs/Point32[] points
sensor_msgs/ChannelFloat32[] channels
```
在points中存储了所有当前图像中追踪到的角点的图像归一化坐标。
在channels中存储了关于该角点的相关信息，这里共存储了5种信息。
1.角点的id
2.角点像素坐标的横坐标
3.角点像素坐标的纵坐标
4.角点在x方向的速度
5.角点在y方向的速度
`sensor_msgs::PointCloud`中所有的信息都是在`void img_callback(const sensor_msgs::ImageConstPtr &img_msg)`中处理得到的，实际依赖的是`FeatureTracker`类，处理图像的入口是`void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)`。`FeatureTracker`类中几个比较重要的数据结构，如下：
```
camodocal::CameraPtr m_camera; //相机模型，保存了相机的内参和相机投影方程

cv::Mat prev_img, cur_img, forw_img; //原始图像数据。prev_img好像没啥用，cur_img保存了上一帧图像，forw_img保存了当前帧。
vector<cv::Point2f> prev_pts, cur_pts, forw_pts; //图像中的角点坐标
vector<int> track_cnt; //保存了当前追踪到的角点一共被多少帧图像追踪到
vector<int> ids; //保存了当前追踪到的角点的ID，这个ID非常关键，保存了帧与帧之间角点的匹配关系。


```


# 处理流程
整个视觉前端最关键的代码就在`void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)`中。
上一帧的图像和图像中的角点分别保存在`cur_img`和`cur_pts`中，利用`cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);`函数可以得到当前图像中能通过光流追踪到的角点的坐标，保存在`forw_pts`中。
由于前一帧保存的角点并不一定全部能够被当前帧追踪到，追踪的成功与否保存在`status`中。因此利用`status`和`void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)`函数对`track_cnt，ids`等进行更新。
经过更新之后`track_cnt`里面保存都是能够追踪到的角点的追踪次数，执行+1操作。

由于光流匹配会导致追踪到的角点越来与少，因此需要根据`MAX_CNT`和在当前帧中追踪到的角点数目差，进行新的角点的提取。依赖的函数为`   cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);`新提取的角点坐标保存在`n_pts`中。值得注意的是这里有个参数`MIN_DIST`，这个参数保证2个相邻角点之间的最小距离，通过这个参数可以在一定程度上保证角点的均匀分布。
这里`mask`的设置也是有讲究的，因为在`forw_img`中已经通过光流追踪到了一部分角点，当再次从`forw_img`中提取角点的时候，通过设置相应的`mask`,来保证角点的提取不会重复。设置`mask`是通过`void FeatureTracker::setMask()`实现的。`void FeatureTracker::setMask()`里面有个很有趣的操作就是会根据角点被追踪的次数进行排序，即`track_cnt, ids, forw_pts`都是按照角点被追踪的次数排序的。
之后会把新追踪到的角点`n_pts`加入到`forw_pts`和`ids`中去。
最后调用`void FeatureTracker::undistortedPoints()`对角点图像坐标做去畸变处理，并计算每个角点的速度。

至此，所有`sensor_msgs::PointCloud`里面的数据都已经计算得到了。

# 小结
Feature Tracker其实仅仅做了视觉前端的数据关联，只是视觉前端的第一步。与ORB通过描述子进行关联匹配不同的是，VINS采用了KLT光流的方式对角点进行关联匹配，并通过id的方式记录关联的结果。总的来说，速度快，抗干扰强，但牺牲了一定的精确度。











