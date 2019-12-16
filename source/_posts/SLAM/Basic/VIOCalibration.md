---
title: VIO标定相关Paper简单梳理
date: 2019-03-23 10:11:25
tags: 
- SLAM基础 
categories:
- 机器人事业
- SLAM
description: VIO标定相关Paper的梳理
---
<!-- more -->


1.Continuous-Time Batch Estimation using Temporal Basis Functions （ICRA 2012）
大致思路：IMU频率很高，如果建立所有IMU时刻和图像时刻的状态量（6 pose、6 bias），参数规模会很大。因此用大概200-300个knots的样条曲线来代表IMU时刻和图像时刻的状态量，此时可以极大降低状态量规模。观测有视觉特征点以及IMU的加速度和角速度。对于每个图像时刻建立视觉观测残差项，对于每个IMU时刻将6 pose微分得到角速度和加速度，和IMU观测建立残差项

2.Unified Temporal and Spatial Calibration for Multi-Sensor Systems （IROS 2013）
在1的基础上加入时间延迟的估计

3.Extending kalibr:Calibrating the Extrinsics of Multiple IMUs and of Individual Axes
在2的基础上，对IMU进行了更精细的建模。建立 input reference axes（IRA），估计加速度计与IRA的旋转，陀螺仪与IRA的旋转，IMU与IRA之间的平移，加计尺度因子，加计旋转修正，陀螺仪尺度因子，陀螺仪旋转修正，以及加计对陀螺仪的影响因子 （ICRA 2016）
