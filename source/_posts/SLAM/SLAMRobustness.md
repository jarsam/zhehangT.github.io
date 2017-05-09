---
title: SLAM系统的鲁棒性
date: 2017-04-7 14:00:31
tags:
categories:
- 机器人事业
- SLAM
description: 一些关于构建更健壮更容错更鲁棒的SLAM系统的简单探讨。
---
<!-- more -->

# 摘要
SLAM在这两年得到了快速的发展，特别是在SLAM系统实现层面，越来越多的大牛贡献出了开源代码。基于视觉特征的ORB-SLAM，基于半稠密的LSD、DSO，基于稠密的Elastic Fusion、RGBD-SLAM，基于激光的GMapping、Cartographer等，都展示出各个思路在解决SLAM这个问题上的强大能力。
但是在实际的应用中，这些SLAM系统总是或多或少的会失效，因为在求解SLAM问题时，目前的SLAM系统总是希望环境是小范围的，长时间不发生变化的。这种对**环境静止的假设**在实际生活中往往是不成立的。因此在实际应用中，开源的SLAM系统常常由于环境的动态变化造成系统崩溃或建图和定位失败。
总得来说，SLAM在健壮、容错、鲁棒这条路上还有很长的一段路要走。
这篇博客的大部分观点来自论文[1]。

# 什么是鲁棒性
鲁棒性的中文解释是系统的健壮性，也就是说系统在异常和危险情况下，不会发生死机、崩溃等情况。但是目前的SLAM系统在环境发生动态变化时，往往会产生异常。

简单的举个特例。ORB-SLAM利用ORB特征在图像之间寻找匹配，对相机的位置进行追踪。但是当环境中动态变化的物体太多，比如人的走动，椅子的移动等，或者说环境条件发生了变化，比如光线变化，季节变化等，导致即使是相同位置的图像，其外观也发生了变化。这些变化很容易引起ORB特征的匹配失败，从而造成对相机追踪的失败。虽然ORB-SLAM添加了重定位模块，用词袋模型来解决追踪失败时相机位置丢失的问题，但是重定位模块也会依赖于视觉特征，因此ORB-SLAM在处理追踪失败这个问题上还有进一步优化的空间。
其实不仅仅是ORB-SLAM存在这种问题，所有基于视觉的SLAM系统或多或少的都存在这种问题。也就是说环境的动态变化是最让目前的SLAM系统头疼的问题，但是在实际的应用中，环境的动态变化恰恰是无法避免的。

SLAM要走向应用，往往需要长时间的运行，往往需要适应动态变化的环境，因此SLAM的鲁棒性的发展会是使其能真正走向应用的关键之一。


# 什么造成鲁棒性问题
要解决鲁棒性问题，我们首先从造成鲁棒性问题的原因说起。SLAM的鲁棒性问题主要分为两个层面：传感器层面和算法层面。
传感器层面是指通过传感器本身是否可靠，也就是通过传感器采集到的数据是否可靠。

首先**环境**会影响传感器。举个例子，在黑暗环境下，摄像头往往无法获得彩色图像，因此目前基于视觉的SLAM系统都是无法在黑暗环境下工作的，换句话中，当环境光条件发生巨大变化时或者从白天进入到黑夜时，视觉SLAM是会失效和崩溃的。还有个例子就是RGBD相机，在室外这种宽阔并且光线强度大的环境中，RGBD相机往往无法正确获取深度图像。总的来说，限于目前传感器技术并不是完美的，因此并不能保证不同的环境下，传感器都能正确工作。但从我目前的了解来看，还没有关于在不同环境下采集到的传感器数据，对于SLAM算法的求解是否会产生影响的相关研究。
其次**传感器自身**会老化，会故障，会导致传感器的精确度下降，最终使得传感器数据异常。传感器本身出现问题可能已经超出了SLAM研究本身的范围，但是一个鲁棒的SLAM系统至少可以检测出传感器数据的异常，从而将减少建图定位结果错的离谱的情况，甚至导致系统崩溃的可能性。但是SLAM系统如何检测传感器数据异常也是一个有待于研究的问题。


算法层面主要包含**数据关联**问题和**参数调整**问题。
数据关联主要是指将传感器观测与空间中的实际物体进行关联。多个传感器观测与空间中同一个物体有了关联之后，就可以对机器人的位姿进行评估。造成数据关联错误，在我看来大致可以分为两个方面。首先是**环境的动态变化**。在前文中也提到过，在动态变化的环境中，数据关联是非常容易发生错误的。明明是空间中的同一个物体，但由于一些环境条件的变化或者摆放位置的变化，导致在多次传感器观测中被认为是不同物体，这就造成了错误的数据关联。其次是**环境的特殊性**。也就是说在某些特殊场景下得到的传感器观测数据，某些数据关联方法是无法正确完成数据关联的。根据no free lunch theorem，不会存在某种传感器观测和数据关联方法可以通吃所有类型的场景。
除了短时间内的数据关联，还有长时间的数据关联问题。长时间的数据关联主要表现在回环检测问题上，提高地图和定位结果的全局一致性。错误的数据关联不仅会造成当前位姿估计的错误，还会造成回环检测和后端优化的错误。最终导致的结果就是，全是错的错的错的。
参数调整问题其实一定程度上也是数据关联的具体形式，因为很多参数设置就是为了能够得到比较好的数据关联。比如在ORB-SLAM2中对于ORB特征点的提取和匹配有专门的参数配置。一个合适的参数配置可以使SLAM系统在特定环境中更可靠的运行。从这个层面上来讲，如果SLAM能够自动调整参数，那么也是可以在一定程度上提高SLAM系统的鲁棒性。



# 相关研究进展
从当前的研究成果来看，很多大牛在致力于解决SLAM系统的鲁帮性问题。在此将论文[1]中引用的论文作简单罗列，方便以后进一步学习。

首先是回环检测中的数据关联问题。在视觉SLAM中，[2]词袋模型在回环检测问题上被广泛使用,并且展现出非常可靠的性能表现。但是词袋模型无法处理光线变化强烈的情况。
因此催生了匹配序列[3]，收集不同的视觉外观并用统一的表达方式[4]，同时使用空间信息和外观信息[5]。这方面的综述可以参考[6]。
在激光SLAM中也有类似的回环检测。比如[7,8]。
在视觉SLAM中，对于回环的验证通常用RANSAC[9]，从而减少错误的数据关联。
在激光SLAM中，对于回环的验证是将回环点，放回已经构建的地图中，计算误差。

之前提到错误的回环会严重影响定位和构图结果[10]。因此一系列方法会提出来处理错误回环问题。其中[11,12,13,14,15]验证在优化期间因为某些约束产生的误差大小来防止错误的回环。[16,17]通过在优化执行之前首先进行错误回环并剔除，来避免产生错误的结果。

而动态环境中数据关联问题，SLAM系统需要能够检测，消除或者跟踪动态变化的物体。目前主流的方法就是消除环境中的动态部分[18]。也有把动态部分作为模型的一部分的方法[19,20,21]。如果环境的变化是周期性的，SLAM系统需要对环境构建多个地图[22,23]或者基于时间参数进行地图表示[24,25]。

前面提到了很多方法来避免产生错误的数据关联和回环检测，但是现在的SLAM系统在面对错误的数据关联时依旧非常脆弱。因此一个理想的SLAM方法可以在错误发生之前进行预判，并提供恢复机制在错误发生时进行恢复和重估计。目前还没有SLAM系统能有这样的能力。在面对传感器错误时，目前的SLAM系统也缺少相应的机制和方法来检验传感器数据的精确度。
关于在地图中进行重定位来增强鲁棒性已经被一些SLAM系统采用。词袋模型是视觉SLAM进行重定位的典型代表。此外也有用轨迹匹配的方法[26]。
也有一些研究工作采用多种传感器模块或者是异构的感知信息来增强鲁棒性[27,28,29,30,31]。
也有一些研究工作针对非刚体空间的三维重建方面的[32,33,34,35]。
在参数自动调整方面，[36]可以在线调整相机的内参。


# 总结
对于SLAM鲁棒性的问题，总结起来大概从以下几个步骤展开。
1.传感器数据的精确度检测。
2.数据关联方法的进一步提高。包括回环检测下的数据关联，动态场景下的数据关联等。
3.系统在应对错误时的机制，包括对错误产生的预判，错误产生后的恢复机制等。
5.对多传感器，异构传感器，异构信息等多方面的数据关联方法的探索。
6.参数的自动调整能力。



# 参考文献
[1] Cadena C, Carlone L, Carrillo H, et al. Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age[J]. IEEE Transactions on Robotics, 2016, 32(6): 1309-1332.
[2] Gálvez-López D, Tardos J D. Bags of binary words for fast place recognition in image sequences[J]. IEEE Transactions on Robotics, 2012, 28(5): 1188-1197.
[3] M. Milford and G. Wyeth. SeqSLAM: Visual route-based navigation for sunny summer days and stormy winter nights. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA),pages 1643–1649. IEEE, 2012
[4] W. Churchill and P. Newman. Experience-based navigation for long-term localisation. The International Journal of Robotics Research(IJRR), 32(14):1645–1661, 2013.
[5] K. L. Ho and P. Newman. Loop closure detection in SLAM by combining visual and spatial appearance. Robotics and Autonomous Systems (RAS), 54(9):740–749, 2006.
[6] S. Lowry, N. Snderhauf, P. Newman, J. J. Leonard, D. Cox, P. Corke,and M. J. Milford. Visual Place Recognition: A Survey. IEEE Transactions on Robotics (TRO), 32(1):1–19, 2016.
[7] G. D. Tipaldi and K. O. Arras. Flirt-interest regions for 2D range data.In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 3616–3622. IEEE, 2010
[8] Hess W, Kohler D, Rapp H, et al. Real-time loop closure in 2D LIDAR SLAM[C]//Robotics and Automation (ICRA), 2016 IEEE International Conference on. IEEE, 2016: 1271-1278.
[9] D. Scaramuzza and F. Fraundorfer. Visual Odometry [Tutorial]. Part I:The First 30 Years and Fundamentals. IEEE Robotics and Automation Magazine, 18(4):80–92, 2011.
[10] N. Sunderhauf and P. Protzel. Towards a robust back-end for pose graph SLAM. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 1254–1261. IEEE, 2012.
[11] P. Agarwal, G. Tipaldi, L. Spinello, C. Stachniss, and W. Burgard.Robust map optimization using dynamic covariance scaling. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 62–69. IEEE, 2013.
[12] L. Carlone, A. Censi, and F. Dellaert. Selecting good measurements via l1 relaxation: a convex approach for robust estimation over graphs. In Proceedings of the IEEE/RSJ International Conference on Intelligent 21 Robots and Systems (IROS), pages 2667 –2674. IEEE, 2014.
[13] Y. Latif, C. Cadena, and J. Neira. Robust Loop Closing Over Time. In Proceedings of Robotics: Science and Systems Conference (RSS),2012.
[14] E. Olson and P. Agrawal. Inference on Networks of Mixtures for Robust Robot Mapping. In Proceedings of Robotics: Science and Systems Conference (RSS), 2012.
[15] N. Sunderhauf and P. Protzel. Towards a robust back-end for pose graph SLAM. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 1254–1261. IEEE, 2012.
[16] E. Olson, M. Walther, J. Leonard, and S. Teller. Single-Cluster Spectral Graph Partitioning for Robotics Applications. In Proceedings of Robotics: Science and Systems Conference (RSS), pages 265–272,2005.
[17] D. Sabatta, D. Scaramuzza, and R. Siegwart. Improved appearance-based matching in similar and dynamic environments using a vocabulary tree. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 1008–1013. IEEE, 2010.
[18] Neira J, Tardós J D. Data association in stochastic mapping using the joint compatibility test[J]. IEEE Transactions on robotics and automation, 2001, 17(6): 890-897.
[19] D. Rosen, J. Mason, and J. Leonard. Towards Lifelong Feature-Based Mapping in Semi-Static Environments. In Robotics: Science and Systems (RSS), workshop: The Problem of Mobile Sensors: Setting future goals and indicators of progress for SLAM, 2015.
[20] A. Walcott-Bryant, M. Kaess, H. Johannsson, and J. J. Leonard. Dynamic pose graph SLAM: Long-term mapping in low dynamic environments. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 1871–1878. IEEE,2012.
[21] C. Wang, C. Thorpe, S. Thrun, M. Hebert, and H. Durrant-Whyte. Simultaneous localization, mapping and moving object tracking. The International Journal of Robotics Research (IJRR), 26(9):889–916,2007.
[22]  F. Dayoub, G. Cielniak, and T. Duckett. Long-term experiments with an adaptive spherical view representation for navigation in changing environments. Robotics and Autonomous Systems (RAS), 59(5):285–295, 2011.
[23] K. Konolige, J. Bowman, J. Chen, P. Mihelich, M. Calonder, V. Lepetit,and P. Fua. View-based Maps. The International Journal of Robotics Research (IJRR), 29(8):941–957, 2010.
[24] T. Krajnı́k, J. P. Fentanes, O. M. Mozos, T. Duckett, J. Ekekrantz, and M. Hanheide. Long-term topological localisation for service robots in dynamic environments using spectral maps. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 4537–4542. IEEE, 2014.
[25] J. M. Santos, T. Krajnik, J. P. Fentanes, and T. Duckett. Lifelong Information-driven Exploration to Complete and Refine 4D Spatio-Temporal Maps. IEEE Robotics and Automation Letters, 1(2):684–691,2016.
[26] M. Brubaker, A. Geiger, and R. Urtasun. Lost! leveraging the crowd for probabilistic visual self-localization. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), pages 3057–3064. IEEE, 2013.
[27] R. W. Wolcott and R. M. Eustice. Visual localization within LIDAR maps for automated urban driving. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),pages 176–183. IEEE, 2014
[28] C. Forster, M. Pizzoli, and D. Scaramuzza. Air-Ground Localization and Map Augmentation Using Monocular Dense Reconstruction. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 3971–3978. IEEE, 2013
[29] D. Majdik, A. L.and Verda, Y. Albers-Schoenberg, and D. Scaramuzza. Air-ground Matching: Appearance-based GPS-denied Urban Localization of Micro Aerial Vehicles. Journal of Field Robotics (JFR),32(7):1015–1039, 2015.
[30] B. Behzadian, P. Agarwal, W. Burgard, and G. D. Tipaldi. Monte Carlo localization in hand-drawn maps. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),pages 4291–4296. IEEE, 2015.
[31] W. Winterhalter, F. Fleckenstein, B. Steder, L. Spinello, and W. Burgard. Accurate indoor localization for RGB-D smartphones and tablets given 2D floor plans. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 3138–3143. IEEE, 2015
[32] A. Agudo, F. Moreno-Noguer, B. Calvo, and J. M. M. Montiel. Sequential Non-Rigid Structure from Motion using Physical Priors. IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI),38(5):979–994, 2016.
[33] A. Agudo, F. Moreno-Noguer, B. Calvo, and J. M. M. Montiel. Real-Time 3D Reconstruction of Non-Rigid Shapes with a Single Moving Camera. Computer Vision and Image Understanding (CVIU), 2016, to appear.
[34] O. Grasa, E. Bernal, S. Casado, I. Gil, and J. M. M. Montiel. Visual SLAM for Handheld Monocular Endoscope. IEEE Transactions on Medical Imaging, 33(1):135–146, 2014.
[35] R. A. Newcombe, D. Fox, and S. M. Seitz. DynamicFusion: Reconstruction and tracking of non-rigid scenes in real-time. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), pages 343–352. IEEE, 2015.
[36] Keivan N, Sibley G. Online SLAM with any-time self-calibration and automatic change detection[C]//Robotics and Automation (ICRA), 2015 IEEE International Conference on. IEEE, 2015: 5775-5782.

