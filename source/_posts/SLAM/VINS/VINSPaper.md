---
title: A Robust and Versatile Monocular Visual-Inertial State Estimator 论文笔记
date: 2017-11-11 10:11:25
tags:
- VINS-Mono
- VIO
categories:
- 机器人事业
- SLAM
description: 关于开源SLAM系统VINS-Mono的论文笔记
---
<!-- more -->

# 摘要
VINS-Mono是最新的基于单目与IMU融合的SLAM系统，整篇文章详细描述了VINS系统的各个部分，与ORB-SLAM-Inertial进行对比学习，进一步加深IMU预积分如何应用到一个单目SLAM系统中去。

# 概述
![](1.png)

VINS主要分为4个部分：
（1）观测数据的预处理
负责图像特征提取，IMU测量值的预积分
（2）初始化
提供所有初始信息，包括位姿、速度、重力向量、陀螺仪偏差和三维地图点
（3）VIO
将IMU预积分信息与视觉特征融合，形成紧耦合的VIO，并且带有重定位和回环检测功能
（4）位姿优化
验证重定位结果，执行全局优化来消除累积误差


# 观测数据预处理

1）图像数据预处理
VINS中，单目图像的追踪是通过 KLT sparse optical flow 算法进行的，提取的图像特征为角点特征，检测器（detector）通过设定像两个角点特征之间的最小像素间隔，来保证所提取的角点特征在图像中的分布。
在这一阶段还会执行关键帧的选择，需要满足两个条件。第一，当前帧和上一个关键帧之间要有足够的视差。第二，如果当前帧所追踪的特帧数目低于某个值，当前帧会被作为关键帧，以避免特征点追踪丢失的情况。

2）IMU预积分
这篇论文的IMU预积分在速度和平移上的与之前的两篇类似，但是在旋转上采用了四元数的方式。四元数的形式感觉上不如李代数来的简洁直观，采用四元数的方式的原因大概是跟作者之前的工作保持一致。

这篇论文还简单地给出了如何推导出预积分模型，主要思路就是将参考系从世界坐标系转换到预积分开始时刻的IMU坐标系下，从而可以将IMU的观测值分离出来。
公式（3）是世界坐标系下的计算公式。
![](2.png)
公式（5）是以积分开始时刻IMU坐标系下的计算公式。$\alpha,\beta,\gamma$为只与imu测量值相关的部分，如公式（6）
![](3.png)
![](4.png)
公式（7）为公式（6）的差分形式，要计算两个关键帧之间的预积分，即对每个观测值进行累加。
![](5.png)

公式（9）给出了欧拉积分公式下系统状态的误差方程。得到协方差矩阵$P\_{b\_{k+1}}^{b\_k}$。
![](6.png)

公式（10）给出了协方差矩阵的迭代求解方式。
![](7.png)


同样对于系统状态的雅可比矩阵也可以用迭代的方式求解（11）。
![](8.png)

当得到新的IMU偏差量之后，为了减少计算量，可以通过公式（12）的方式更新IMU预积分量。
![](9.png)

至此，得到了IMU的观测模型（13），即可建立基于IMU预积分的残差项。
![](10.png)

>总得来说，IMU预积分模型都大同小异，对于雅可比矩阵和协方差矩阵的计算，还需要进一步的分析，但是感觉应该和前两篇论文是一样的。


# 估计初始化
与ORB-SLAM-Inertial不同的是，在本篇文章的初始化中，作者选择忽视加速度偏差，其理由是加速度偏差与重力在量级上相差比较大，加速度偏差很难被观测。

1）视觉初始化
与ORB-SLAM-Inertial类似，初始化过程将从纯视觉SLAM开始。当帧之间有足够的视差，就利用5点法恢复出旋转和平移，并对特征点进行三角化。根据当前的最后一帧和三角化的地图点，利用PnP方法对其他帧进行位姿估计。最后利用full-BA进行优化。另外，将第一帧图像作为世界坐标系的原点。

2）陀螺仪偏差初始化
陀螺仪的偏差估计原理上与ORB-SLAM-Inertial是相同的，只不过这里使用了四元数。如式（15）所示。
![](11.png)

基于新得到的陀螺仪偏差，利用式（12）对IMU预积分项进行更新。

3）速度，重力向量和尺度初始化
把速度，重力向量和尺度作为估计量：
![](12.png)

这部分的求解与ORB-SLAM-Inertial初始化的第二步是类似的，即将待估计的变量构造成一个超定方程，然后求其最小二乘解。但是在ORB-SLAM-Inertial作者曾指出，如果直接引入速度变量，会增加超定方程是病态的可能性。这篇文章中，并没有这样的考虑。

4）重力修正
![](16.png)
其基本原理就是利用上图所示的切线空间中的正交基$b\_1$和$b\_2$，计算其修正量$w1$和$w2$，修正后的重力向量为$g\cdot \bar{\hat{g}} + w\_1b\_1 + w\_2b\_2$。将其代入式（17），将$w\_1$和$w\_2$与（16）中的变量一起求解。正交基$b\_1$和$b\_2$可以通过如下的方式得到。
![](17.png)
通过不断的迭代上述过程，直到$\hat{g}$收敛。

通过修正重力向量的方向，可以得到世界坐标系$(\cdot)^w$与参考坐标系$(\cdot)^{c\_0}$之间的旋转$q\_{c\_0}^w$。利用$q\_{c\_0}^w$即可将之前得到的所有的速度修正到世界坐标系下。

> 对比ORB-SLAM-Inertial中的重力修正部分，此处的重力修正部分虽然不如ORB-SLAM-Inertial中来得简洁明了，但是其利用了重力向量只有两个自由度需要估计（pitch，roll）的优点，在计算速度和准确性上可能会存在优势。


# 紧耦合VIO
对于VIO，本文采用了基于滑动窗口的紧耦合的形式，在滑动窗口内所有要估计的系统状态如下：
![](13.png)
其中$x\_k$表示IMU在第$k$张图片时的状态，$\lambda\_l$表示第$l$个特征点的反向深度。
我们采取 visual-inertial bundle adjustment 的形式，来对系统状态进行优化：
![](14.png)
分别包含了先验残差项，IMU残差项和视觉残差项。

1）IMU残差项
基于式（13）的可以建立式（24）的IMU残差项，
![](15.png)
很奇怪的是，式（24）中的IMU预积分并没有包含偏差IMU偏差。

2）视觉残差项
与之前两篇论文中不同的是，VINS中所用的视觉残差项是基于球面的，而不是针孔的。这样做的好处是视觉模型可以兼容几乎所有类型的相机，包括广角相机和鱼眼相机。式（25）概括了整个视觉残差项的构造过程。
![](18.png)
需要注意的是，这里的反投影是利用图像中的像素点坐标$[u,v]$得到三维空间中的特征点坐标。首先利用第$i$帧图像，该计算特征点坐标在当前相机坐标系下的三维坐标。然后利用第$j$帧图像对相同特帧点的观测，以及第$i$帧图像和第$j$帧图像之间的外参，同样计算出该特征点在第$i$帧图像相机坐标系下的三维坐标，在将两者的误差投影到正切空间，即可得到一个二维的残差值。其中$b\_1$和$b\_2$的计算与重力修正时用到的算法相同。

3）边缘化
为了保证滑动窗口内的关键帧数目保持在一个可以快速求解的数目，VINS中使用了 marginalization 方法，关键帧的选择方法如图所示：
![](19.png)

边缘化方法使用了 Schur complement 方法，并将边缘化的观测数据与要被去除的状态，转化为先验信息，加入到整个优化问题中。

4）Motion-only Bundle Adjustment
为了进一步减小计算量，引入另一种BA的形式。
此处依旧沿用式（22），但不再优化像（21）中所示的所有变量，只优化位姿、速度和固定数目的IMU状态，而把特征点深度、外参、偏差和旧的IMU状态作为固定量。使用依靠所有视觉和IMU观测量的motion-only BA，而不是依靠单张图像帧的PnP方法，可以使得状态估计的结果更加平滑。
这里有点像ORB-SLAM中的Tracking线程干的事。

文中还提到了IMU-Rate的状态估计、失败检测及恢复。

# 重定位
重定位首先从回环检测开始。与ORB-SLAM类似，回环检测使用了词袋模型DBoW2，依赖于corner角点和BRIEF描述子。文中提到由于VIO可以观测roll和pitch，因此不需要依赖像ORB这种具备旋转不变性的特征。

检测到回环之后，要建立当前帧和回环帧之间的特征关联，通过BRIEF得到的特征关联会有很多的异常匹配，因此文中采用了基于对极几何和PnP的RANSAC来剔除异常匹配。

建立匹配之后，将利用匹配关系建立约束，如式（26）所示：
![](20.png)
与式（22）相比，式（26）多了一个回环误差项。

# 全局位姿优化
重定位只是将当前滑动窗口与之前的位姿信息进行了对齐，为了获得全局一致的位姿信息，还需要进行全局位姿优化。之前也提到过，对于VIO来说，roll和pitch是客观测的，因此全局位姿的优化是一个四自由度的优化问题。

当关键帧被边缘化时，它会被加入到全局位姿图中。全局位姿图中的关联关系有两种，第一种称为 Sequential Edges, 是当前关键帧$i$在滑动窗口中与之前关键帧$j$之间的相对位置关系。如式（27）。
![](21.png)

如果被边缘化的关键帧被检测到是回环，则会存在第二种关联关系，称为Loop Closure Edge，即关键帧与回环帧之间相对位置关系。其约束关系与式（27）相同。

有了关联关系之后，即可定义进行位姿图优化的残差项，如式（28）。
![](22.png)
$\hat{\theta\_i},\hat{\phi\_i}$ 是可以从VIO中直接得到的 roll 和 pitch 的角度。

全局位姿优化问题即可构造为：
![](23.png)
其中第一项是Sequential Edges，第二项为Loop Closure Edge。

为了保证VINS可以长时间的运行，防止位姿图过于庞大，在运行过程中采用了一种降采样的方式，来限制位姿图的尺寸。包含 Loop Closure Edge 的关键帧将会被保留，而某些与相邻关键帧很接近或者关联关系很类似的关键帧，就会被去除。


# 总结
实验部分暂时先跳过了。
初次学习VIO，通读整篇文章还是有些吃力，很多地方理解的还有些羞涩。希望能够结合VINS的代码，进一步加深理解。
下一步，好好撸代码吧！


# 参考文献
VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator
http://www.cnblogs.com/buxiaoyi/p/7353353.html













