---
title: ORB-SLAM2:an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras 论文笔记
date: 2017-04-24 10:11:25
tags:
- ORB-SLAM
categories:
- 机器人事业
- SLAM
description: 关于ORB-SLAM2论文的笔记
---
<!-- more -->

# 前言
针对单目相机无法直接获得景深信息的缺点，作者在ORB-SLAM只支持单目相机的基础上添加了双目相机和RGBD相机的支持，形成了ORB-SLAM2。这篇博客是ORB-SLAM2的论文笔记。


# Introduction
单目相机具有成本低，设置简单的优点，但是同样存在尺度不确定、初始化需要足够视差，最终造成尺度漂移、无法处理纯旋转等问题。而通过双目相机和RGBD相机，以上问题都可以得到很好的解决。
ORB-SLAM2为SLAM的发展作出了以下几个**贡献**：
1.这是第一个同时提供单目，双目和RGB-D接口的SLAM开源系统，并且包含回环检测，重定位和地图重用。
2.通过BA对RBG-D进行优化，效果优于state-of-the-art的ICP或photometric and depth error minimization
3.通过使用近距离和远距离的双目点以及单目观测，使得其双目的精确度要高于state-of-the-art的直接使用双目的SLAM系统
4.通过禁用建图来实现利用已有地图，进行轻量级的定位。

# ORB-SLAM2
ORB-SLAM2的框架和ORB-SLAM的框架几乎是一模一样的，包含了三个线程：Tracking，Local Mapping和Loop Closing。主要的区别还是在对于双目相机和RBG-D相机前端，是如何进行追踪并构建后端优化问题的。因此与ORB-SLAM相同的部分就不再赘述，详细可参考上一篇关于ORB-SLAM论文的博客。

**A.Monocular, Close Stereo and Far Stereo Keypoints**
ORB-SLAM2是基于特征的SLAM系统，因此当从输入的图像中提取特征之后，图像不需要被保存而是直接丢弃，因此可以说ORB-SLAM2与传感器之间是相互独立的，重要的还是特征提取的过程，如图所示。
![](1.png)

**Steroe keypoints**
Steroe Keypoints用三维坐标$X\_s=(u\_L, v\_L, v\_R)$来定义。其中$(u\_L, v\_L)$表示左边图像的像素点坐标，$v\_R$表示右边图像的横坐标。
对于**双目相机**，首先从左边图像中提取ORB特征，然后在右边图像中寻找匹配，通过这种方式可以快速的找到一系列的Steroe keypoints。
对于**RGB-D相机**，从RGB图像中提取ORB特征，此时提取的ORB特征的像素坐标作为左边图像的像素坐标，然后根据每个特征对应的深度信息恢复出该ORB特征在右边图像的像素坐标，从而得到Steroe Keypoints。基线设置为8cm。
一个Keypoint会被分为far和close两种。如果Steroe Keypoints对应的深度小于基线的40倍则为close，反之则为far。close的关键点可以从一帧图像中直接三角化得到准确的scale，translation和rotation信息，因为其深度信息是可靠的。而far的关键点可以提供准确的rotation信息，而scale和translation信息并不可靠，因此需要通过多视图方式来对远距离的关键点进行三角化。
**Monocular keypoints**
Monocular keypoints用二维坐标$X_m=(v_L,v_L)$来定义。对于无法提供深度信息的关键点，只能通过多视图的方式进行三角化，但无法恢复scale信息，只能恢复出rotation和translation信息。

**B.System Bootstrapping**
使用双目或者RGB-D相机不需要像单目相机那样进行复杂的初始化，而只需要把第一帧作为关键帧，得到初始地图。

**C.Bundle Adjustment with Monocular and Stereo Constraints**
论文中提到了三种BA方式：motion-only BA，local BA和full BA。与ORB-SLAM中提到的BA方式也是一一对应的。
ORB-SLAM2中的BA增加了从地图点到Steroe keypoint的投影方式，从而使Steroe keypoint可以直接作为误差计算对象出现在BA中。

**D. Loop Closing and Full BA**
使用双目或者RGB-D相机不会出现尺度漂移的问题，因此在对回环候选帧进行位姿优化时，不再需要使用sim3相似变换，使用so3刚体变换就可以了。而进行full BA的闭环优化时，鉴于其计算量可能会比较大，因此在这里会新开一个线程专门处理full BA，这里会涉及到full BA过程中如果有新的回环被检测到该怎么办的问题。论文阐述的做法是直接停止当前正在进行的full BA，闭合最新的回环和full BA。新开线程处理full BA还会存在full BA完成时，如何把full BA的结果与full BA过程中信添加的关键帧和地图点融合，论文提到的做法是full BA执行过程中，先不把新的关键帧和地图点加进来，而是等full BA完成时，根据优化结果对新的关键帧和地图点做一次矫正，然后再进行关键帧和地图点的添加。

**E. Keyframe Insertion**
ORB-SLAM2的关键帧添加策略与ORB-SLAM基本保持一致。唯一的区别就是基于Steroe Keypoint添加了一个条件。在大尺寸的场景中，只有存在足够多的close Steroe Keypoint才能保证位姿评估的准确性，因此在Tracking过程中，两帧之间匹配的close Steroe Keypoint少于100时，并且当前帧可以提取多余70个close Steroe Keypoint时，当前帧就可以作为关键帧进行添加。这种策略对于大场景下的定位非常关键。


**F. Localization Mode**
ORB-SLAM2在ORB-SLAM的基础上添加了定位模式，使其可以利用已有的地图进行高效的定位。在这种模式下Local Mapping和Loop Closing线程被停止，只通过Tracking线程对相机位姿进行追踪。用到的就是用两帧之间的特征匹配做motion-only BA，以及根据已有地图做local BA。


# 总结
可以说ORB-SLAM2是非常优秀并且易于学习的SLAM系统，它涉及了基于视觉特征的SLAM的方方面面。作者最后也提到了基于ORB-SLAM2的未来工作，包括多相机融合，鱼眼相机的支持，生成大尺度的稠密地图以及提高系统鲁棒性等。最后我希望自己未来能够在这些方面作出贡献。


# 参考文献
[1] Mur-Artal, Raul, and Juan D. Tardos. "ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras." arXiv preprint arXiv:1610.06475 (2016).APA	






