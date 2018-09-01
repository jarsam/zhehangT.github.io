---
title: VINS-Mono中的回环检测
date: 2018-04-25 10:11:25
tags:
- VINS-Mono
- 源码学习
categories:
- 机器人事业
- SLAM
description: VINS-Mono中的 回环检测相关代码学习
---
<!-- more -->

# 摘要
回环检测的重要性对于任何 SLAM 系统都是不言而喻的。VIO 即使精确度非常高，仍然会有累积误差。此时就需要回环检测来消除累积误差。我们来看看 VINS-Mono 中是如何进行回环检测的。

# 快速重定位
首先理一理 VINS-Mono 进行回环检测的流程。
VINS-Mono 中关于回环检测的入口在 pose_graph 包中`pose_graph_node.cpp`，pose_graph 的输入主要依赖三个数据，第一是原始图像，第二是地图点，第三是关键帧。除了原始图像，其他输入主要来自于 vins_estimator。
```c
queue<sensor_msgs::ImageConstPtr> image_buf;  //原始图像数据
queue<sensor_msgs::PointCloudConstPtr> point_buf; //世界坐标系下的地图点坐标，该地图点在最新图像帧中的归一化坐标，图像坐标和feature id
queue<nav_msgs::Odometry::ConstPtr> pose_buf; //当前帧的 pose
```

pose_graph 的依靠这三个数据进行回环检测。回环帧的检测用的还是词袋模型，如何得到回环约束，我们放到下一小节。这里我们先关注快速重定位功能。
这里vins提供了一种快速重定位功能，当检测到回环约束时，直接将回环检测的约束返回给 vins_estimator 进行快速的重定位。相关代码在函数`bool KeyFrame::findConnection(KeyFrame* old_kf)`。

```c
if(FAST_RELOCALIZATION)
{
    //msg_match_points 是发送给 estimator 的匹配信息
    //points 里是匹配到的角点的归一化坐标和该地图点的id
    //channels 是回环帧的 pose
    sensor_msgs::PointCloud msg_match_points;
    msg_match_points.header.stamp = ros::Time(time_stamp);
    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
    {
        geometry_msgs::Point32 p;
        p.x = matched_2d_old_norm[i].x;
        p.y = matched_2d_old_norm[i].y;
        p.z = matched_id[i];
        msg_match_points.points.push_back(p);
    }
    Eigen::Vector3d T = old_kf->T_w_i; 
    Eigen::Matrix3d R = old_kf->R_w_i;
    Quaterniond Q(R);
    sensor_msgs::ChannelFloat32 t_q_index;
    t_q_index.values.push_back(T.x());
    t_q_index.values.push_back(T.y());
    t_q_index.values.push_back(T.z());
    t_q_index.values.push_back(Q.w());
    t_q_index.values.push_back(Q.x());
    t_q_index.values.push_back(Q.y());
    t_q_index.values.push_back(Q.z());
    t_q_index.values.push_back(index);
    msg_match_points.channels.push_back(t_q_index);
    pub_match_points.publish(msg_match_points);
}
```

vins_estimator利用回环检测的约束其实并非直接优化滑动窗口内的所有状态量，但是通过回环约束计算出把当前帧转换到正确位姿所需要的位姿修正量，具体的做法如下。
1.在回环约束中已知回环帧匹配到了哪些地图点，以id的形式记录。
2.在滑动窗口内搜索具有相同id的地图点，建立重投影误差，当前帧的位姿作为初始位姿。
3.优化重投影误差，得到回环帧在当前滑动窗口具有的地图点下，应该所处的位姿。
4.此时得到的回环帧位姿和回环约束中回环帧的位姿产生了一个相对位姿变换量，利用这个相对位姿变换量，就可以把当前帧修正到比较准确的位姿。

> 其实通过这种方式修正的位姿，只利用了一帧约束，并不是非常准确，所以称之为快速重定位。这部分内容跟论文是对应不上的，应该是VINS-Mono新版本里面的改动。


# 全局位姿优化

真正执行全局回环优化的是函数`void PoseGraph::optimize4DoF()`。
基本的思路很简单，通过`bool KeyFrame::findConnection(KeyFrame* old_kf)`我们得到了当前帧和回环帧之间的回环约束，更具体的讲就是通过计算当前帧角点描述子和回环帧的角点描述子之间的汉明距离，得到最佳的角点匹配。匹配信息在以下三个局部变量中。这三个变量中保存的信息是一一对应的。
```c
vector<cv::Point2f> matched_2d_cur, matched_2d_old; //像素坐标
vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm; //归一化坐标 
vector<cv::Point3f> matched_3d; //地图点坐标
```
接下来进行了通过pnp删除一些误匹配。如果此时得到当前帧和回环帧之间的角点匹配数目大于`MIN_LOOP_NUM`，则表示当前回环帧检测是有效的。最后把当前帧与回环帧之间的位姿转换关系保存在`Eigen::Matrix<double, 8, 1 > loop_info`中。
有了当前帧和回环帧之间的位姿转换关系，就具备了进行全局位姿优化的条件。
全局位姿优化有两种约束关系。第一种是通过 VIO 得到的，相邻两帧之间的位姿约束。第二种是通过刚刚描述的当前帧与回环帧之间的位姿约束。最后要注意的是，对 VIO 来说，pitch角和rolling角是不会产生累积误差的，因此此时的全局位姿优化只在4自由度上进行。
基于 VIO 位姿约束构造的残差项定义在`struct FourDOFError`中。这里有两点需要注意。**首先**，全局位姿优化只会优化回环帧之后的关键帧，回环帧自身保持固定，回环帧之间的帧不参与优化。**第二**，在构造 VIO 位姿约束时，不仅仅构造了相邻两帧之间的位姿约束，而是够造了与前5帧之间的位姿约束。
基于回环约束构造的残差项定义在 struct FourDOFWeightError，可以通过weight参数来放大这一部分的误差值。但 VINS-Mono 代码中将 weight设为了1。
简单看一下误差项，不难发现这里的误差项只有平移三个自由度，yaw角一个自由度，一共四个自由度。这部分的优化求解采用了自动求导的方式。

```c
bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
{
	T t_w_ij[3]; //因为ti和tj都是世界系下的平移，所以ti-tj是世界系下的平移差
	t_w_ij[0] = tj[0] - ti[0];
	t_w_ij[1] = tj[1] - ti[1];
	t_w_ij[2] = tj[2] - ti[2]; 
	// 得到Riw，Riw(ti-tj)即为i坐标系下的平移
	T w_R_i[9];
	YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
	// rotation transpose
	T i_R_w[9];
	RotationMatrixTranspose(w_R_i, i_R_w);
	// rotation matrix rotate point
	T t_i_ij[3];
	RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
	residuals[0] = (t_i_ij[0] - T(t_x));
	residuals[1] = (t_i_ij[1] - T(t_y));
	residuals[2] = (t_i_ij[2] - T(t_z));
	residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));
	return true;
}
```

# 地图融合
假设加载了之前建好的地图，称之为地图1。如果 VINS-Mono 在启动之后并没有检测到与地图1的回环，即没有重定位成功，此时会新建一个地图，称为地图2。而在 VINS-Mono 运行过程中，当检测到与地图1中的回环约束之后，此时的全局位姿优化会将地图1和地图2合并。如下图所示。这部分内容其实与回环检测没有本质上的区别。

![](1.png)













