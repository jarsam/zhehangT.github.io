---
title: ORB-SLAM:a Versatile and Accurate Monocular SLAM System 论文笔记
date: 2017-04-20 10:11:25
tags:
- ORB-SLAM
categories:
- 机器人事业
- SLAM
description: 关于ORB-SLAM论文的笔记
---
<!-- more -->

# 前言
关于ORB-SLAM，想必每个SLAM领域的入门者都会学习一番。过去几个月，我也在基于ORB-SLAM2的开源代码做硕士课题。ORB-SLAM的论文[1]因此也看了很多遍。网上已经有关于这篇论文的中文翻译，但有些地方由于每个人的表达方式不同，并不是很好理解。因此在此也用自己的表达方式对ORB-SLAM的论文进行简单记录。
当然也很有可能，这篇博客只有我自己可以看懂。


# Introduction
论文是从Bundle Adjustment(BA)对于SLAM的作用展开的。通过BA可以为相机定位和地图构建提供更精确的评估。关于BA在博客[3D-2D的运动估计](https://zhehangt.github.io/2017/03/07/SLAM/PnP/)里做过一次简单的介绍。以前BA由于计算量比较大，因此被认为无法用于实时SLAM，特别是视觉SLAM，但是随着移动计算能力的快速发展，BA已经可以在视觉SLAM中达到实时的效果。要运用BA，相应的视觉SLAM算法必须满足以下几个方面：
1.要对关键帧之间的特征进行匹配以及地图点和特征之间进行关联
2.为了防止计算量过大，要去除冗余的关键帧
3.关键帧和地图点之间的匹配和关联要尽可能的稠密，也就是说关键帧中的观察到的地图点能够提供足够多的视差以及足够的回环
4.要提供一个尽可能准确的初始值用于BA的非线性优化
5.对于局部地图，优化只在局部进行，与全局尺寸无关，从而具备更好的可扩展性
6.对于全局地图，要有回环检测和全局优化

其实从ORB-SLAM的很多设计方式上看，都是奔着满足上面这几个方面去的，也算BA运用到视觉SLAM中的一个简单的指导。


ORB-SLAM的成功离不开第一个基于特征的视觉SLAM：PTAM[2]。ORB-SLAM基本延续了PTAM的基本思路，在PTAM的基础上做了很多方面的改进。
ORB-SLAM主要有以下几点优点：
1.在所有的模块中都采用相同的图像特征。这些模块包括追踪，建图，重定位，回环，因此使得我们的系统更加的简单，高效和可靠。通过使用ORB特征，使得可以在不采用GPU的情况下实时运行，并在不同的视角和光照下都具有良好的不变性。
2.能够实时运行在大场景的环境中。通过使用Covisibility Graph，追踪和局部地图构建都在Covisibility Graph的一部分中处理，因此与全局的地图尺寸无关。
3.实时的回环检测的优化是基于Essential Graph进行的，Essential Graph是通过生成树树进行维护，是基于Covisibility Graph强关联边和回环边构建的。
4.实时的重定位，能够从追踪丢失中恢复追踪以及在已有地图中进行定位。
5.地图初始化时可以根据是否为平面自动选择相应的模型。
6.对于地图点和关键帧的选择采用一种survival of the fittest的方法，生成时放宽尺度但选择时提高要求。这种策略可以剔除冗余的关键帧，从而增强追踪的鲁棒性以及长时间运行的能力。


# System Overview
**A.Feature Choice**
我们设计的主要思想之一就是用于建图和追踪的特征可以同时用于重定位和回环检测。这使得我们的系统更有效率。选择ORB特征[3]是因为其计算和匹配速度快，具备旋转不变性。ORB特征已经在位置识别中取得了很好的性能[4]。
**B. Three Threads: Tracking, Local Mapping and Loop Closing**
{% qnimg ORBSLAMPaper-01.png title: ... alt:... %}

如图1所示，系统共有Tracking，Local Mapping和Loop Closing三个线程在同时运行。
Tracking：基于每一帧来对地图进行定位，并会决定什么时候插入新的关键帧。通过与前一帧进行特征匹配来对当前帧的位姿进行初始化估计，然后用motion-only BA对位姿作进一步优化。当Tracking丢失时通过位置识别来进行全局的定位。完成了与前一帧的匹配和位姿估计之后，可以通过得到的位姿将与其他关键帧进行特征匹配，也就是从一个局部地图里面进行搜索匹配使相机位置得到进一步优化。最后Tracking线程来判断当前帧是不是关键帧。

Local Mapping：通过处理关键帧和执行local BA，从而得到一个较优的地图。对于新的关键帧，会Covisibility Graph寻找更多的特征匹配，并三角化得到新的地图点。对于新的地图点会进行严格筛选，Local Mapping也同样负责删除一些冗余帧。

Loop Closing：对每个新的关键帧都要进行闭环检测，以确认是否有闭环。如果闭环被检测到，通过计算相似变换来得到闭环的累积误差。然后对齐闭环处的两帧并融合重复的地图点。最后，执行有相似约束的位姿图优化[5]，消除回环误差，确保全局地图的一致性。这里的优化是基于Essential Graph进行，它是Covisibility Graph的子图。

所有的优化都是基于g2o实现的 Levenberg-Marquardt 算法。

**C.Map Points, KeyFrames and their Selection**
每个地图点$p\_i$包含：
1.世界坐标系下的3D坐标$X\_{w,i}$
2.视图方向$n\_i$，对于每个能观察到该地图点的关键帧的中心到地图点有一个视图方向，$n\_i$是所有能观察到该地图点的关键帧的视图方向的平均值，用单位向量表示
3.ORB特征描述子$D\_i$，对于所有能够观察到该地图点的关键帧都计算一个ORB特征描述子，$D\_i$是其中汉明距离最小的。
4.根据ORB特征尺度不变性的约束，计算出能观测到该点的最大距离$d\_{max}$和最小距离$d\_{min}$


每个关键帧$K\_i$包含：
1.相机的位姿$T\_{i,w}$，也是从世界坐标系转换到相机的坐标系的刚体变换矩阵
2.相机内参，包括焦距和相机主点
3.所有从图像帧提取的ORB特征，不管是否已经关联了地图点，都通过畸变模型进行矫正

地图点和关键帧通过宽松策略创建，而剔除机制会非常严格，剔除冗余的关键帧和错误匹配或不可追踪的地图点。从而使运行期间地图具备扩展性，增强在极端环境下追踪的鲁棒性。


**D.Covisibility Graph and Essential Graph**

关键帧之间的视图内容关联信息(Covisibility information)在系统的几个模块上都非常有用,关联信息用权重图像间接地表示。每个节点都是一个关键帧，如果两个关键帧可以观察到相同的地图云点，根据地图云点数目赋予边权重值$\theta$。

我们通过位姿图优化来纠正回环。为了防止Covisibility Graph非常稠密，我们通过建立Essential Graph来保留所有的关键帧节点，但是只保留少量的边，从而保留足够的信息来产生精确的优化结果。系统从第一个关键帧开始增量式地构建一个生成树，它是Covisibility Graph中边数最少的连通子图。当新的关键帧插入时，它加入树中，并连接到能够观察到相同地图点最多的关键帧上。当一个关键帧通过筛选策略被删除时，系统会根据关键帧所在的位置更新生成树。Essential Graph包含了生成树，是保留了Covisibility Graph中高度关联(θmin = 100)的边和回环边的子图，构成强连通的关键帧连接图。


**E.Bags of Words Place Recognition**
基于DBoW2[7]，系统嵌入了基于图像词袋的位置识别模块，完成闭环检测和重定位功能。视觉单词是离散化的描述子空间，视觉单词组成视觉字典。视觉字典是离线创建的，创建视觉字典的ORB特征描述子是从大量图像中提取的。如果图像足够多，同一个视觉字典可以用于多个不同的环境。系统增量式的构建一个包含倒序索引的数据库，用来存储视觉单词，从而可以高效地检索关键帧。当关键帧通过筛选机制被删除时，数据库也会更新。

关键帧在视图上可能会存在重叠，检索数据库时，可能不止一个高分值的关键帧。DBoW2认为是图像重叠的问题，就将时间上接近的所有图像的分值相加。但这样并没有考虑同一个地方的关键帧在不同时刻被观察到的情况。因此我们依据关键帧是否是视图关联的进行分类。另外，我们的数据库返回的是分值高于最好分值75%的所有关键帧。

利用词袋模型还可以加速特征匹配。详见[4,7]。


# Automatic Map Initialization
由于单目相机无法直接得到景深信息，因此需要通过多张图像来恢复环境的三维结构，这个过程通常被称为地图初始化。地图初始化需要两帧之间有足够的视差，基于视差计算出两帧之间的位姿变换，并通过三角化得到初始的地图点。依据观察到的场景是否是平面的，单目相机的初始化通常分为两种方法：基于单应矩阵的平面初始化和基于基础矩阵的非平面初始化。ORB-SLAM的自动初始化就是同时计算这两种模型，然后用启发式的方法选择最合适的模型来完成初始化。主要的计算流程如下。

**1.Find initial correspondences**
从当前帧$F\_c$中提取ORB特征，并与参考帧$F\_r$做特征匹配$x\_c \leftrightarrow x\_r$。如果没有找到足够多的匹配特征，就重设参考帧。

**2.Parallel computation of the two models**
在两个线程中并行的计算单应矩阵$H\_{cr}$和基础矩阵$F\_{cr}$:
$$
x\_c = H\_{cr}x\_r  \qquad x\_c^TF\_{cr}x\_r=0
$$
单应矩阵的计算方法是normalized DLT，基础矩阵的计算方法是8点法，具体计算方法见神书多视图几何[8]。

为了公平比较单应矩阵和基础矩阵哪个更好，论文设计了一些技巧，比如求解时设定相同的迭代次数，并且用于且解的特征匹配点也是相同的。
对于每一次迭代，都计算一次symmetric transfer errors，基于这个误差求得单应矩阵和基础矩阵的最终得分$S\_H$和$S\_F$。具体计算过程可以参见原文。

**3.Model selection**
如果在平面场景选择了基础矩阵进行初始化，得到初始化的地图往往是错误。为了能够在平面场景时选择单应矩阵，在非平面场景时选择基础矩阵，通过计算$R\_H = \dfrac{S\_H}{S\_H+S\_F}$,当$R\_H > 0.45$时,选择单应矩阵，反之选择基础矩阵。这是一种启发式的方式。

**4.Motion and Structure from Motion recovery**
当模型被选定后，就可以通过得到的矩阵计算相关的运动假设。对于单应矩阵可以恢复出8种假设。对于每种假设匹配好的特征点进行验证。只有当三角化得到的地图点都在相机的前方并且重投影误差足够小，才认为本次的地图初始化是成功的。对于基础矩阵也是如此。

**5.Bundle adjustment**
执行一次full BA。除了第一帧不变之外，所有关键帧和地图点都会称为BA优化的对象。


# Tracking
Tracking线程用来处理从相机中得到的每一帧图像。对于每一帧图像，Tracking线程会通过如下几个步骤进行处理。

**1.ORB Extraction**
提取ORB特征涉及到几个参数。第一个参数是scale levels，第二个参数是scale factor。这两个参数应该是用来建立图像金字塔，提高ORB特征的尺度不变性。第三个参数是提取ORB的特征数。对于512 × 384 到 752 × 480分辨率，提取1000个。对于更大的分辨率，提取2000个。为了保证提取的ORB特征能够均匀分布，通过将每个scale level的图像进行网格划分，并在每个网格中提取至少5个特征点作为阈值。根据每个网格中实际提取的特征点数，会对阈值进行相应的调整。最后会计算ORB描述子。

**2.Initial Pose Estimation from Previous Frame**
如果上一帧的追踪成功，就用相同的速率运动模型进行位姿的估计，然后从前一帧看到的地图点云与当前帧做匹配，根据匹配结果进行位姿优化。这里被称为motion BA。如果匹配失败，就加大搜索范围。

**3.Initial Pose Estimation via Global Relocalization**
如果追踪失败，则将当前帧转化为词袋，然后检索词袋数据库，在所有关键帧中查找ORB特征的匹配。找到对应的匹配之后，进行PnP算法求解当前的位姿。得到一个位姿估计值之后，可以通过搜索当前帧与其他关键帧之间的匹配，从而进一步优化位姿。

**4.Track Local Map**
当有了对当前相机位姿的估计之后，可以通过建立局部地图的方式来提高位姿评估的精确度。局部地图$K\_1$包含一系列关键帧，这些关键帧可以和当前帧观察到相同的地图点，局部地图$K\_2$也是一系列关键帧，这些关键帧与$K\_1$中的关键帧是相邻的关联关系。$K\_1$中和当前帧观察到相同的地图点数目最多的关键帧称为$k\_{ref}$。所有在$K\_1$和$K\_2$可以看到的地图点都会在当前帧中进行一次搜索匹配：
1）直接计算地图点到当前帧的投影$x$，如果投影在图像边界之外，则直接剔除。
2）计算当前帧的方向$v$和地图点的视图方向$n$的乘积，如果$v\cdot n < cos(60^{\circ})$，则剔除。
3）计算地图点到相机中心的距离d，如果$d\notin[d\_{\min},d\_{\max}]$,则剔除。
4）计算当前帧的尺度$d/d_{\min}$。
5）比较地图点ORB描述子与当前帧中还未匹配的ORB特征，如果匹配到的ORB特征在尺度和位置上都接近，则找到了地图点和当前帧的一组匹配。
相机位姿会根据匹配的地图点进行进一步优化。

PS:步骤2中提到的motion BA和步骤3,4中提到的位姿优化都是指固定当前帧中能看到的地图点，然后优化当前帧的位姿。

**5.New Keyframe Decision**
最后Tracking线程会判断当前帧是不是满足关键帧的条件，只有满足以下所有条件才能称为关键帧：
1）距离上一次全局重定位已经超过了20帧图像。（保证好的重定位）
2）Local Mapping处于空闲状态，或者距离上一次插入关键帧已经超过了20帧图像
3）当前帧可以观察到超过50个地图点。（保证好的追踪）
4）当前帧与参考帧同时可以观察到的地图点少于90%。参考帧是$K_1$中与当前帧同时可以观察到的地图点最多的关键帧。（增加视差变化）


# Local Mapping
Local Mapping处理每一个被Tracking线程判断为关键帧的新关键帧$k\_i$。主要有以下几个处理步骤：

**1.KeyFrame Insertion**
首先更新Covisibility Graph，图中插入节点$k\_i$以及更新与$k\_i$相关联的其他关键帧之间的边。
其次更新生成树，将关键帧连接到参考帧上。
最后计算关键帧的词袋表达，有利于为三角化新的地图点提供数据关联。

**2.Recent Map Points Culling**
地图点能够保留下来必须经过严格的筛选。在创建了三个关键帧之后，所有新创建的地图点都必须满足以下两个条件：
1）能观察到该点的关键帧不应该少于理论上能观测到该点的关键帧数目的1/4。
2）地图点被创建之后，接下来应该至少有3个关键帧可以观察到这个地图点
在任何时刻，如果观察到地图点的关键帧少于3个，那么该地图点会被剔除。这种情况会发生在进行局部BA时，一些异常的关键帧被剔除了。

**3.New Map Point Creation**
对于$k\_i$与在Covisibility Graph中其他相连的关键帧集合$K\_c$，搜索$k\_i$中和$K\_c$未与地图点关联的ORB特征之间的匹配，然后三角化出新的地图点。新地图点的创建要满足地图点位置在相机的前方，重投影误差小以及进行尺度的检查。新的地图点是通过两个关键帧创建的，但是可以通过Tracking线程中的Track Local Map，实现地图点和其他关键帧中未匹配地图点的ORB特征进行匹配。
确定新地图点的相关属性。

**4.Local Bundle Adjustment**
局部BA针对$k\_i$,相关联的$K\_c$和所有能被观察的地图点进行优化的。其他可以看到相同的地图点但是不与$k\_i$相关联也会加入到BA中，但是只作为约束条件，是固定不变的。优化过程中或者优化结束后，异常点会被剔除。

**5.Local Keyframe Culling**
Local Mapping会剔除冗余的关键帧，从而有利于BA优化和长时间运行时关键帧的维护开销。
如果一个关键帧中观察到的地图点超过90%能被其他三个关键帧观察到，则剔除该关键帧。这个策略是[9]的启发。


# Loop Closing
当Local Mapping线程处理完一个关键帧$k\_i$之后，Loop Closing线程会尝试利用$k\_i$做回环检测。主要有以下几个步骤：

**1.Loop Candidates Detection** 
计算$k\_i$与在Covisibility Graph中相关联的关键帧($\theta\_{\min}=30$)的词袋向量的相似度,保留最小值$s\_{\min}$。然后检索位置识别数据库并剔除所有小于$s\_{\min}$的关键帧。另外，所有与当前帧相连的关键帧都会被剔除。如果连续三个候选关键帧的检索结果是一致的，则认为$k\_i$确实为闭环候选关键帧。

**2.Compute the Similarity Transformation**
单目SLAM系统有7个自由度，3个平移，3个旋转，1个尺度因子，这七个自由度上，地图都有可能发生漂移[5]。因此要闭合某个回环，我们需要计算从当前关键帧$K_i$到回环关键帧$K_l$的相似变换矩阵，以获得回环的累积误差。计算相似变换也可以作为回环的几何验证。
我们首先计算与$k\_i$中ORB特征关联的地图点和回环候选关键帧的对应关系。此时，对每个候选回环，我们有了一个3D到3D的对应关系。我们对每个候选回环执行RANSAC迭代，通过Horn方法[10]找到相似变换。如果我们用足够的有效数据找到相似变换$S\_{il}$，我们就可以优化它，并搜索更多的对应关系。如果$S\_{il}$有足够的有效数据，我们再优化它。当支持$S\_{il}$的有效数据足够多时，就可以认为$k\_l$回环被接受。这里的优化，所有的地图点都是固定的，只优化当前帧的位置。

**3.Loop Fusion**
回环矫正的第一步是融合重复的地图点，插入与回环闭合相关的相似视图的新边缘。在步骤2中通过相似变换$S\_{il}$已经矫正了当前关键帧的位姿$T\_{iw}$，现在可以将矫正扩散到与当前帧相关联的关键帧上，这样回环两端就可以对齐。所有回环关键帧和相关联的关键帧可以观察到的地图点都会被重投影到$k\_i$和与$k\_i$相关联的关键帧，类似于Tracking线程中的Track Local Map。所有匹配的地图点和用于计算$S\_{il}$会被融合。融合过程中所有的关键帧将会更新它们的边，这些新更新的边将非常有效的用于闭合回环。

**4.Essential Graph Optimization**
为了有效的闭合回环，基于Essential Graph做一次位姿优化，此时的优化只固定初始位姿，其他所有的位姿进行尺度漂移的优化，即Sim3的位姿优化。

# 总结
实验部分就暂不分析了。至此，ORB-SLAM的总体流程和框架应该都非常明了了，对于其中的具体实现细节还需要结合作者的开源代码来看。
对于回环检测的部分，特别是Sim3优化的部分的理解还比较羞涩，需要依据[5,11,12]做进一步学习。


# 参考文献
[1] Mur-Artal R, Montiel J M M, Tardos J D. ORB-SLAM: a versatile and accurate monocular SLAM system[J]. IEEE Transactions on Robotics, 2015, 31(5): 1147-1163.
[2] G. Klein and D. Murray, “Parallel tracking and mapping for small AR workspaces,” in IEEE and ACM International Symposium on Mixed and Augmented Reality (ISMAR), Nara, Japan, November 2007, pp. 225–234.
[3] E. Rublee, V. Rabaud, K. Konolige, and G. Bradski, “ORB: an efficient alternative to SIFT or SURF,” in IEEE International Conference on Computer Vision (ICCV), Barcelona, Spain, November 2011, pp. 2564–2571.
[4] R. Mur-Artal and J. D. Tardós, “Fast relocalisation and loop closing in keyframe-based SLAM,” in IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, China, June 2014, pp. 846–853.
[5] H. Strasdat, J. M. M. Montiel, and A. J. Davison, “Scale drift-aware large scale monocular SLAM.” in Robotics: Science and Systems (RSS),Zaragoza, Spain, June 2010.
[6] H. Strasdat, A. J. Davison, J. M. M. Montiel, and K. Konolige, “Double window optimisation for constant time visual SLAM,” in IEEE International Conference on Computer Vision (ICCV), Barcelona, Spain, November 2011, pp. 2352–2359.
[7] D. Gálvez-López and J. D. Tardós, “Bags of binary words for fast place recognition in image sequences,” IEEE Transactions on Robotics,vol. 28, no. 5, pp. 1188–1197, 2012.
[8] R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, 2nd ed. Cambridge University Press, 2004.
[9] W. Tan, H. Liu, Z. Dong, G. Zhang, and H. Bao, “Robust monocular SLAM in dynamic environments,” in IEEE International Symposium on Mixed and Augmented Reality (ISMAR), Adelaide, Australia, October 2013, pp. 209–218.
[10] F. Endres, J. Hess, J. Sturm, D. Cremers, and W. Burgard, “3-d mapping with an rgb-d camera,” IEEE Transactions on Robotics, vol. 30, no. 1,pp. 177–187, 2014.
[11] H. Strasdat, “Local Accuracy and Global Consistency for Efficient Visual SLAM,” Ph.D. dissertation, Imperial College, London, October 2012.
[12] B. Triggs, P. F. McLauchlan, R. I. Hartley, and A. W. Fitzgibbon, “Bundle adjustment a modern synthesis,” in Vision algorithms: theory and practice, 2000, pp. 298–372.
[13] [路游侠的博客 ORB-SLAM(5)优化](http://www.cnblogs.com/luyb/p/5447497.html) 






