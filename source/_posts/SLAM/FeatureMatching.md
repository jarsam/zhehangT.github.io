---
title: 图像特征匹配
date: 2017-03-3 10:11:25
tags:
- SLAM基础 
- 特征匹配
categories:
- 机器人事业
- SLAM
description: 关于学习视觉SLAM问题中图像特征匹配的简单记录。
---
<!-- more -->

# 摘要
视觉SLAM是指通过摄像头采集周围环境的图像，根据图像来进行定位和建图。也就是说可以通过一系列的图像，来计算机器人的位姿。
基于特征点法的视觉SLAM是目前比较成熟的解决方案。这篇博客将简单聊一聊什么是特征点，以及特征点的匹配问题。

# 特征点
我们知道图像在计算机中用像素矩阵来表示，而特帧是图像信息的另一种数字表达形式。那有了像素矩阵为什么还要搞出一套特征表示出来？主要出于以下几个考虑：
1）我们希望在环境选取一些比较有代表性的点，这些点在相机视角发生少量变化后，依旧可以从各个图像中找到。如果用像素矩阵直接表示这些点，不同图像之间变化会非常大，不容易找到。图像特征就是来解决这个问题，这些点被称为特征点。
2）用特征点表示可以降低变量的维度，有利于大规模的计算。

在计算机视觉领域，大牛们经过长期的研究，设计了很多很牛逼的特征点，如SIFT，SURF，ORB等等。为什么说这些特征点牛逼？主要有以下几个方面：
1）可重复性：环境中相同的区域，可以在不同的图像中被找到
2）可区别性：环境中不同的区域，计算得到的特征表示不同
3）高效性：在同一图像中，特征点的数量远小于像素的数量
4）本地性：特征仅与一小片图像区域有关

特征点由关键点和描述子两部分组成，比如当我们谈论ORB特征是，是指提取ORB关键点，并计算ORB描述子两件事情。关键点是值该特征点在图像里的位置，有些特征点还具有朝向、大小等信息。描述子描述了该关键点周围像素的信息，两个特征点的描述子在向量空间上的距离可以用来表示其相似程度，距离越小越相似。

在目前的SLAM方案中，ORB是质量和性能之间较好的折中，因此将着重介绍ORB特征。

# ORB特征
ORB特征的关键点称为“Oriented Fast”，是一种改进的FAST角点。描述子称为BRIEF。

先简单介绍一下什么是FAST角点。FAST角点的主要思想是：如果一个像素与它邻域的像素差别较大，那它可能是角点。相比于其他角点检测算法，FAST只需要比较像素亮度的大小，因此计算速度非常快。

FAST角点的**计算方法**：
对于图像中某个像素$p$，亮度为$I\_p$，设置一个阈值$T$。以像素$p$为中心，选取半径为3的圆上的16个像素点。如果有连续$N$个点的亮度大于$I\_p+T$或小于$I\_p-T$，那么像素$p$可认为是FAST角点。

FAST角点的**缺点**：
数量大且不确定。ORB做了改进。首先对原始的FAST角点分别计算Harris响应值，然后选取前N个具有最大响应值的角点，作为最终的角点集合。
FAST角点不具有方向信息，并且由于它固定取半径为3的圆，因此存在尺度问题：远看像角点的地方，接近以后看可能就不是角点了。ORB在FAST角点的基础上添加了尺度和旋转的描述。尺度不变性由构造图像金字塔，并在金字塔的每一层上检测角点来实现，金字塔是指对图像进行不同层次的降采样，以获得不同分辨率的图像。而特征的旋转是由灰度质心法实现的。其方向的定义就是几何中心$O$与质心$C$。

有了关键点之后就是为每个关键点计算其描述子。ORB采用的描述子是为BRIEF添加了方向信息，因此称为Steer BRIEF。简单的来说就是随机的挑选关键点周围的两个像素，对比其大小，根据大小关系设置为0或1。因此得到的描述子是由许多个0和1组成的描述向量。具体的计算方法不展开说了。

# 特征匹配
考虑到两个时刻的图像，如果在图像$I\_t$中提取到特征点$x\_t^m$，在图像$I\_{t+1}$提取到特征点$x\_{t+1}^n$，如何寻找这两个集合元素的对应关系呢？之前我们提到描述子在向量空间上的距离可以用来表示其相似程度，距离越小越相似。因此最简单的办法就是对每个特征点$x\_t^m$，计算它与所有的$x\_{t+1}^n$之间距离，然后选择其中距离最小，作为匹配点。这种做法称为暴力匹配。
但是当特征点数量很大时，暴力匹配法的运算量将变得很大，因此通常用快速近似最近邻(FLANN)方法来进行匹配。
当完成了图像之间的特征匹配，就可以利用这些匹配来计算，为了拍摄这两张图像相机经过了怎样的位姿变换。这部分内容，下回分解。


# 参考文献
1.《视觉SLAM十四讲》第七讲




