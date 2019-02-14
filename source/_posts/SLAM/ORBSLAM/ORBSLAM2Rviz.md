---
title: ORB-SLAM2的Rviz可视化方案
date: 2017-04-27 10:11:25
tags:
- ORB-SLAM
categories:
- 机器人事业
- SLAM
description: 关于如何让ORB-SLAM2能在Rviz中进行可视化的以及融合Turtlebot坐标系的解决方案
---
<!-- more -->

# 前言
ORB-SLAM的地图和定位可视化是通过Rviz进行展示的，而在ORB-SLAM2中，为了不依赖于ROS，ORB-SLAM2的可视化采用了pangolin库。而我的硕士课题有个需求，就是要使ORB-SLAM2的结果和Cartographer的结果同时显示在同一个可视化工具中，而Cartographer是采用Rviz显示的，还定制了专用的Rviz插件。思考再三，决定为ORB-SLAM2重新添加基于Rviz的可视化模块。

# ORB-SLAM2的Rviz可视化
简单分析一下ORB-SLAM中关于Rviz的可视化，总结了以下几点：
1.ORB-SLAM的Rviz视化用了单独的一个类来完成可视化信息的发布：MapPublisher类
2.所有的可视化信息都是Rviz的Mark类型，根据发布的地图点，关键帧，Covisibility Graph，Spanning Tree和相机轨迹，设计了不同的Mark类型。
3.所有的可视化信息，包或地图，轨迹等都是从ORB-SLAM中的Map类中获取的。
4.每次获得一帧图像，进行Track后，利用MapPublisher类发布可视化信息。
5.在配置相应的Rviz，使其可以接收可视化信息。

明白了这几点之后，在ORB-SLAM2中添加Rviz可视化模块就很简单了，主要对源代码做两个改动。
1.添加MapPublisher类和配置Rviz，并在每次Track之后利用MapPublisher类发布可视化信息，这里可以直接复用ORB-SLAM中的MapPublisher类和Rviz。
2.为Map类添加返回相关信息的接口。
3.特别要注意ORB-SLAM2的坐标系下，z轴是朝前的，而Rviz的坐标系下，z轴是朝上的，因此要做相应的转换。

通过以上几个修改，就能在Rviz中显示ORB-SLAM2的地图构建结果和相机位姿了。简单到让我很佩服ORB-SLAM2代码的简洁和松耦合。

# ORB-SLAM2的坐标系转换
在对ORB-SLAM2进行实验的过程中，我采用的喜闻乐见的Turtlebot。我们知道ORB-SLAM2的地图构建和定位都是基于自己的map坐标系的，与Turtlebot的坐标系并不产生任何关联。但是我希望将map坐标系融入到Turtlebot的坐标系中，方便之后针对Turtlebot对ORB-SLAM2进行修改和扩展。
首先简单了解下Turtlebot的坐标系。如下图所示。
![](1.png)
Turtlebot的坐标系遵循**odom --- base_link --- other_link**的标准格式。**other_link**中包含相机的坐标系**camera_rgb_frame**。

因此要将ORB-SLAM2的map坐标系融合到Turtlebot坐标系中，即形成**map --- odom --- base_link --- other_link**的标准格式，需要计算**map --- odom**之间的转换关系。
ORB-SLAM2得到的是在map坐标系下相机的位姿，也就是说**map --- camera_rgb_frame**之间的转换是已知的， 在配合上**odom --- camera_rgb_frame**之间的转换关系，**map --- odom**之间的转换关系就不难求解了。具体计算方法可以看看以下的伪代码。
```c++
    //根据世界坐标系map下相机的位姿和里程计坐标系odom下相机的位姿，计算相机坐标系map和里程计坐标系odom之间的旋转矩阵和平移向量
    
    r_map_camera = r_map_odom * r_odom_camera
    --> r_map_odom = r_map_camera * r_odom_camera^(-1);
    
    // C_m和C_o表示相机在map坐标系下和odom坐标系下的坐标
    C_m = r_map_odom * C_o + t_map_odom
    C_m = r_map_camera * [0,0,0] + t_map_camera  = t_map_camera
    C_o = r_odom_camera * [0,0,0] + t_odom_camera  = t_odom_camera
    --> t_map_odom = t_map_camera - r_map_odom * t_odom_camera;
    // r_map_odom 和 t_map_odom就是map坐标系和odom坐标系之间的转换关系
```

根据以上的思路，在配合ROS中的TF包，很快就能完成map坐标系和Turtlebot坐标系之间的融合。以后如果要查找map坐标系和Turtlebot其他坐标系之间的转换关系，比如深度相机坐标系，IMU的坐标系等，就可能借助TF完成非常快速和方便的查询。

# 总结
感觉没啥能总结的。












