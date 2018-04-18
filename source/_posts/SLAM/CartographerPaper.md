---
title: Real-Time Loop Closure in 2D LIDAR SLAM 论文笔记
date: 2017-05-1 10:11:25
tags:
- Cartographer
categories:
- 机器人事业
- SLAM
description: 关于Google开源的Cartographer SLAM论文的论文笔记
---
<!-- more -->

# 前言
这是关于Google开源的Cartographer SLAM系统的论文。一直在学习视觉SLAM的相关知识，突然上手读这篇基于激光的SLAM论文，有些措手不及。希望能通过论文笔记的方式去数理整篇论文的思路。

# Introduction
论文的贡献在于：提出了一种新颖的基于激光数据的回环检测方式，这种方式可以减少计算量，满足实时的大场景地图构建以及大规模的实时优化的性能需求。
激光数据往往被认为信息量少，难以用于回环检测，因此这个工作还是非常nice的。

# Related Work
对于scan matching方面了解较少，所以将这方面的论文都列出来，方面以后的进一步学习。
相关工作中首先介绍了scan matching的几种方法：
1.scan-to-scan matching 是基于激光的SLAM中最常用来估计相关位姿的方法。但是非常容易造成累积误差。[1,2,3,4]
2.scan-to-map matching 可以减少累积误差，因为scan-to-map每次都通过高斯-牛顿法获得局部最优的位姿，前提是要有一个比较好的初始化位姿估计。[5]
3.pixel-accurate scan matching 可以进一步减少局部误差，但是计算量比较大。这个方法同样可以用来检测回环。
4.从laser scan中提取特征，从而减少计算量[4]。histogram-based matching用于回环检测[6]。用机器学习做laser scan的特征检测[7]。

之后介绍了处理累积误差的两种方式。
1.基于粒子滤波的优化。粒子滤波在处理大场景地图时，由于粒子数的极具增长造成资源密集。粒子滤波这一块我了解不多，论文中给出了给了几个相关论文。[8,9,10]
2.基于位姿图的优化。与视觉SLAM的位姿图优化大同小异，主要是在观测方程上的区别。论文中给的相关论文[2,11,12,13]。

> 感觉作者写相关工作写的好敷衍。其实我觉得，整篇论文都好敷衍，公式多解释少。


# System Overview
Cartographer 能产生一个精确度为5cm的2D栅格地图。精确度为5cm是因为组成submap是一系列5cm×5cm的栅格。每次laser scan会通过当前的submap进行scan matching，从而将laser scan插入到submap中的最优估计位置。然后再用laser scan去更新submap。submap是通过与当前laser scan通过邻近的多个laser scan更新产生了，因此scan matching本质上也是在当前laser scan和多个邻近laser scan之间进行的。
通过scan matching得到的位姿估计在短时间内是可靠的，但是长时间会有累积误差，为了优化这个误差会有规律的执行pose optimization。当不再有新的scan插入到一个submap，就认为这个submap已经创建完成了。所有创建完成的submap以及scans都会用作回环检测的scan matching。如果有scans和submap在距离上足够的近，则scan matcher会尝试寻找回环can matching。为了减少计算量，Cartographer设置了特殊的策略来找到回环scan matching。这个策略就是根据当前的位姿估计附近设置搜索窗口，在这个搜索窗口内执行branch-and-bound方法来寻找回环scan matching，如果找到了一个足够好的match，则会将该匹配的闭环约束加入到位姿优化问题中。

论文中说回环优化的速度非常快，快到可以在加入新的submap之前就完成优化，从而保证了一种软实时。优化这么快是因为用了 branch-and-bound approach 以及submap中栅格的预计算。


# Local 2D SLAM
Cartographer通过 local 和 global 两种方式进行SLAM。
local方式就是通过submap进行scan matching在这一章。
global方式就是回环检测在下一章。

**A.Scans**
一个scans包含一个起点和很多个终点的。起点称为origin，终点称为scan points，用$H= \\{ h\_k \\}\_ {k=1,...,k}$。在scan坐标系下，origin就是坐标原点，scan points就是在scan坐标系下的坐标。当把一个scans插入到一个submap中时，假设submap坐标系到scan坐标系的坐标转换为$T\_{\xi}$，即激光传感器在submap坐标系下的位姿。则每个$h$在submap坐标系下的坐标为：$T\_{\xi}h$


**B.Submaps**
一个submap是通过几个连续scans创建，是由5cm×5cm大小的概率栅格$[p\_{min},p\_{max}]$(submap创建完成时的栅格概率如果小于$p\_{min}$表示未被占用，在$p\_{min}$和$p\_{max}$之间表示未知，大于$p\_{max}$表示占用)。每个submap的中心称为grid point，submap中的其他点称为corresponding pixel。
对于每个scan，都会产生一组称为hits的grid point和一组称为misses的grid point。如下图所示。
{% qnimg CartographerPaper-01.png title: ... alt:... %}
其中阴影带叉的是hits，阴影不带叉的是misses。
每个hits中的grid point被赋予初始值$p\_{hit}$，每个misses中的grid point被赋予初始值$p\_{miss}$
如果grid point已经有$p$值,则用下述公式更新hit。

$odds(p)=\dfrac{p}{1-p}$
$M\_{new}(x)= clamp(odds^{-1}(odds(M\_{old}(x)) \cdot odds(p\_{hit}) ))$

miss也是类似的。


**C.Ceres scan matching**
把scan插入到submap之前，需要通过scan matching对$T\_{\varepsilon}$进行优化。这里的优化问题为非线性最小二乘问题，通过Ceres库进行求解，非线性最小二乘问题构造形式如下式：
$$
\underset {\xi}{argmin} \sum\_{k=1}^K(1-M\_{smooth}(T\_{\xi}h\_k))^2 \qquad  (\mathrm{cs})
$$
其中$M\_{smooth}$函数完成了从2D到1D的平滑函数。这里用的是 bicubic interpolation 方法。
对于优化问题，一个相对精确的初始估计非常重要。因此如果通过IMU提供角度信息，可以提高优化的准确性。在缺少IMU的情况下，高频率的matching或者pixel-accurate scan matching也可以提高准确性，但会增加时间复杂度。

> 有几个问题感觉论文中没说清楚，一个submap的大小和形状是怎么样的，当scan的origin不在当前的submap内，但是points在submap中时是怎么更新和匹配submap的，这些疑问还需要结合源代码做进一步的分析。


# Closing Loops
Cartograoher通过创建大量的小submap来实现大场景的建图。为了消除小submap带来的累积误差，通过优化所有scan和submap的位姿，来提高准确度。优化方法参考了论文[2]。

A. Optimization problem
回环的优化问题与scan matching的优化问题类似，都是通过构造非线性最小二乘的方式进行的。形式如下：
$$\underset {\Xi^m,\Xi^s }{argmin} \dfrac{1}{2} \sum\_{ij} \rho( E^2({\xi}\_i^m, {\xi}\_j^s; \Sigma\_{ij}, {\xi}\_{ij} ) ) \qquad  (\mathrm{spa})
$$
其中$\Xi^m = \\{ {\xi}\_i^m  \\}\_{i=1,..,m}$是submap的位姿，$\Xi^s= \\{ {\xi}\_j^s  \\}\_{j=1,..,s}$是scan的位姿，这些位姿是在世界坐标系下的。submap位姿和scan位姿之间存在约束条件。${\xi}\_ij$表示scan在submap坐标系下的位姿，$\Sigma\_{ij}$是相应的协方差矩阵，这个协方差矩阵可以通过[14]的方式获得，也可以通过$(\mathrm{cs})$公式获得。
残差E的计算公式如下：
$$
E^2(\xi\_i^m, \xi\_j^s;\Sigma\_{ij},\xi\_{ij})=e(\xi\_i^m, \xi\_j^s;\xi\_{ij})\Sigma\_{ij}^{-1} e(\xi\_i^m, \xi\_j^s;\xi\_{ij})  \\\
e(\xi\_i^m, \xi\_j^s;\xi\_{ij}) = \xi\_{ij} - \begin{pmatrix}
R^{-1}\_{\xi\_{i}^m}(t\_{\xi\_{i}^m} - t\_{\xi\_{j}^s})  \\\ 
\xi\_{i;\theta}^m - \xi\_{j;\theta}^s
\end{pmatrix}
$$

> 论文读到这里，实在是不明白这个公式怎么就约束回环了。经过一些自己的思考，大概理了下思路：在Local 2D SLAM中所有的约束都是scans在某一个submap中得到，也就是公式CS。但是仅靠scans在某一个submap中的约束是无法进行回环的。要进行回环优化必须添加回环约束。我认为这里的回环约束就是要找到scans在多个submap中的约束。实际上当前的scans可能可以观察到多个submap，也就是说当前的scans是可以被多个submap定位的，但是为了简化，在Local 2D SLAM中，只通过当前的submap来对scans进行定位。而loop scan matching就是尝试在其他的submap对scans进行定位，如果能够在其他submap中定位成功，那么就是一个回环约束。至于到底是不是这样的，还需要分析源码来进行验证，论文是看晕了。

如果上述的解释是正确的。那么对残差公式就有了这样的解释。submap的位姿$\xi\_i^m$和scan的全局位姿$\xi\_j^s$是在Local 2D SLAM中得到的，其中$\xi\_j^s$是由Local 2D SLAM中对该scan进行定位的submap的位姿计算相应的坐标计算得到的。$\xi\_{ij}$是在回环scan matching中得到的。整个优化过程就是调整scan matching到的submap的位姿和对该scan进行Local定位的submap的位姿，从而最终得到全局一致性的地图。

B.Branch-and-bound scan matching
之前提到的回环约束关系$\xi\_{ij}$就是通过这里的方法得到的，也是整篇论文最核心的地方。
首先看一下pixel-accurate match的匹配过程。
$$
\xi^* = \underset{\xi \in W}{argmax} \sum\_{k=1}^K M\_{nearest}(T\_{\xi}h\_k) \qquad  (\mathrm{BBS})
$$
其中$W$是搜索空间，$M\_{nearest}$就是该pixel对应的grid point的M值。之后可以通过CS公式进一步提高$\xi$匹配的准确度。

搜索空间和搜索步长的选择是决定pixel-accurate match是否高效的关键。
论文给出了搜索步长的计算方式。
$$
d\_{max} = \underset{k=1,...K}{max} \left \\| h\_x \right \\|  \\\
\delta\_{\theta} = arccos (1-\dfrac{r^2}{2d\_{max}^2}) \\\
w\_x = \left \lceil \dfrac{W\_x}{r} \right \rceil \quad w\_y = \left \lceil \dfrac{W\_y}{r} \right \rceil \quad w\_{\theta} = \left \lceil \dfrac{W\_{\theta}}{\delta\_{\theta}} \right \rceil 
$$
其中$W\_x = W\_y=7m$，$W\_{\theta}$，因此搜索空间就可以确定了。此时搜索空间的大小是7m*7m。
$$
\overline{W} = \\{ -w\_x,...,w\_x \\} \times \\{ -w\_y,...,w\_y \\} \times \\{-w\_{\theta},...,w\_{\theta} \\} \\\
W = \\{ \xi\_0 + (rj\_x, rj\_y, \delta\_{\theta}j\_{\theta}):(j\_x,j\_y,j\_{\theta}) \in \overline{W}  \\}
$$
有了搜索空间和搜索步长，就可以得到最原始的暴力搜索方式。如下图所示:
{% qnimg CartographerPaper-02.png title: ... alt:... %}

为了进一步提高搜索效率，Cartograoher采用了branch and bound approach的方式。branch and bound approach是一种在问题的解空间树上搜索问题的解的方法，被Google套用在最优位姿的搜索中,从而将无法实时化暴力解优化到可以满足实时化，不得不说真是厉害。所以学好算法是多么重要。
至于branch and bound approach到底是怎么做的，还是看论文吧。



# 参考文献
[1] E. Olson, “M3RSM: Many-to-many multi-resolution scan matching,” in Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), June 2015.
[2] K. Konolige, G. Grisetti, R. Kümmerle, W. Burgard, B. Limketkai, and R. Vincent, “Sparse pose adjustment for 2D mapping,” in IROS, Taipei, Taiwan, 10/2010 2010.
[3] F. Lu and E. Milios, “Globally consistent range scan alignment for environment mapping,” Autonomous robots, vol. 4, no. 4, pp. 333–349, 1997.
[4] F. Martı́n, R. Triebel, L. Moreno, and R. Siegwart, “Two different tools for three-dimensional mapping: DE-based scan matching and feature-based loop detection,” Robotica, vol. 32, no. 01, pp. 19–41,2014.
[5] S. Kohlbrecher, J. Meyer, O. von Stryk, and U. Klingauf, “A flexible and scalable SLAM system with full 3D motion estimation,” in Proc. IEEE International Symposium on Safety, Security and Rescue Robotics (SSRR). IEEE, November 2011.
[6] M. Himstedt, J. Frost, S. Hellbach, H.-J. Böhme, and E. Maehle, “Large scale place recognition in 2D LIDAR scans using geometrical landmark relations,” in Intelligent Robots and Systems (IROS 2014),2014 IEEE/RSJ International Conference on. IEEE, 2014, pp. 5030–5035.
[7] K. Granström, T. B. Schön, J. I. Nieto, and F. T. Ramos, “Learning to close loops from range data,” The International Journal of Robotics Research, vol. 30, no. 14, pp. 1728–1754, 2011.
[8] G. Grisetti, C. Stachniss, and W. Burgard, “Improving grid-based SLAM with Rao-Blackwellized particle filters by adaptive proposals and selective resampling,” in Robotics and Automation, 2005. ICRA 2005. Proceedings of the 2005 IEEE International Conference on. IEEE, 2005, pp. 2432–2437.
[9] G. D. Tipaldi, M. Braun, and K. O. Arras, “FLIRT: Interest regions for 2D range data with applications to robot navigation,” in Experimental Robotics. Springer, 2014, pp. 695–710.
[10] J. Strom and E. Olson, “Occupancy grid rasterization in large environments for teams of robots,” in Intelligent Robots and Systems (IROS),2011 IEEE/RSJ International Conference on. IEEE, 2011, pp. 4271–
4276.
[11] R. Kümmerle, G. Grisetti, H. Strasdat, K. Konolige, and W. Burgard,“g2o: A general framework for graph optimization,” in Robotics and Automation (ICRA), 2011 IEEE International Conference on. IEEE,2011, pp. 3607–3613.
[12] L. Carlone, R. Aragues, J. A. Castellanos, and B. Bona, “A fast and accurate approximation for planar pose graph optimization,” The International Journal of Robotics Research, pp. 965–987, 2014.
[13] M. Bosse and R. Zlot, “Map matching and data association for large-scale two-dimensional laser scan-based SLAM,” The International Journal of Robotics Research, vol. 27, no. 6, pp. 667–691, 2008.
[14] E. B. Olson, “Real-time correlative scan matching,” in Robotics and Automation, 2009. ICRA’09. IEEE International Conference on. IEEE, 2009, pp. 4387–4393.












