---
title: ORB_SLAM2中的Sim3优化
date: 2018-11-1 10:11:25
tags:
- ORB-SLAM
- 源码学习
categories:
- 机器人事业
- SLAM
description: ORB_SLAM2中的Sim3优化
---
<!-- more -->

# 摘要
单目SLAM在运行的过程中，不但会积累旋转和平移上的误差，还会产生尺度上的漂移。ORB-SLAM2中，对于单目的尺度漂移，回环优化的时候会做特殊的处理，即Sim3的优化。
之前对于ORB-SLAM2中的回环优化流程进行过梳理，但未针对Sim3这部分深入理解，半年后补坑。

# ComputeSim3()
在单目SLAM中，由于存在尺度漂移，在计算当前帧和回环帧之间的位姿关系时，将引入sim3变换，即在se3的基础上引入的尺度。
$$
S = \begin{bmatrix}
sR & t \\\
0 & 1
\end{bmatrix}
$$

假设在当前帧和回环帧中，我们通过词袋模型找到了一系列的匹配点，那么如何通过计算一个`sim3`的变换，把当前帧的位姿转换到一个正确的位姿？
在ORB-SLAM2中，计算这个sim3变换主要经历了如下几个步骤。

1.通过当前帧和回环帧中的匹配点$(p\_i \rightleftharpoons {p}'\_i)$。我们可以建立如下的变换关系 $ p\_i = sR{p}'\_i + t$
利用[1]中的公式,可以只用3对匹配点即可求解这个sim3的闭合解。这部分操作在``void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)``中。闭合解的求解公式就不展开讲了，有兴趣的可以查看参考文献。
但这个sim3变换并不一定是准确的，因为当前帧和回环帧之间的匹配点是有可能存在外点的。因此在进行sim3闭合解求解时，利用Ransac迭代了5次，找出其中内点最多的一次sim3变换的求解。

2.即使在第一步中通过Ransac得到了相对准确的sim3变换。但由于只用了3对匹配点，我们仍然认为这个sim3变换存在比较大的误差。因此ORB-SLAM2中又进行了一次基于sim3的BA计算。具体过程是这样的。
首先基于步骤1中得到的sim3变换，对当前帧和回环帧中的ORB特征相互投影，从而找到更多的匹配点。这部分代码在``ORBmatcher::SearchBySim3(...)``
当前帧和回环帧之间有了更多的匹配点之后，既可以建立关于sim3的当前帧和回环帧中ORB特帧点的重投影误差，优化得到更加准确的sim3变换。这部分代码在`Optimizer::OptimizeSim3()`中。
ORB-SLAM2中，这部分优化是借助于`g2o`进行的。在这个优化问题中，待优化的变量为sim3变换，对应到`g2o`中的顶点为`g2o::VertexSim3Expmap`。这是个7维的变量(3维旋转、3维平移、1维尺度)。
约束关系为匹配点的重投影误差，包括投影到当前帧和投影到回环帧两种，对应到`g2o`中的边为`g2o::EdgeSim3ProjectXYZ`和`g2o::EdgeInverseSim3ProjectXYZ`。这是个2维的重投影误差。
当前帧到回环帧投影残差的计算主要的代码为``_error = obs-v1->cam_map1(project(v1->estimate().map(v2->estimate())))``。其中`obs`为回环帧中的特征点图像坐标，`v1`为`sim3`变换，`v2`为特征点在当前帧下相机坐标系下的三维空间坐标。
回环帧到当前帧的计算方法也是类似的``_error = obs-v1->cam_map2(project(v1->estimate().inverse().map(v2->estimate())))``。只是对`sim3`变换进行了取逆操作。`sim3`的取逆公式为

$$
S = \begin{bmatrix}
sR & t \\\
0 & 1
\end{bmatrix} ^ {-1} = \begin{bmatrix}
\dfrac{1}{s} R^T &  -\dfrac{1}{s} R^T t \\\
0 & 1
\end{bmatrix} 
$$

> 其实匹配点的三维坐标也作为`g2o`的顶点`g2o::VertexSBAPointXYZ`被加入到了优化问题中，只不过这些顶点是固定不优化的，可以理解成是观测值。

3.构建了步骤2中的优化问题之后，除了直接优化，ORB-SLAM2中还有一些提高优化结果准确性的操作。
首先是对步骤2中的优化问题迭代5次得到一个优化结果，基于这个优化结果得到每对匹配点的投影误差，如果投影误差大于10个像素，则认为这个匹配是个误匹配，将其从优化问题中删除。通过这种方式减少误匹配对优化结果的影响。
再去除误匹配后会再次进行优化。
如果最终当前帧和回环帧之间存在超过20个匹配点，则认为这个当前帧和回环帧的`sim3`变换是有效的。
后续操作就是利用这个`sim3`变换和回环帧的局部视图中的所有地图点，寻找更多的地图点匹配关系。如果最终匹配点超过40个，则认为当前帧和回环帧确实存在回环关系。

这里要重点说明一个公式。给定一个当前帧到回环帧的`sim3`变换，如何把表示当前帧的`se3`通过这个`sim3`的变换得到新的`se3`。一般的做法是把的`se3`提升到`sim3`，其中`s=1`。然后在`sim3`上做变换。在`g2o`的代码中，两个`sim3`变换是写成如下的方式
```c++
    g2o::Sim3 gScm(R,t,s); //当前帧到回环帧的sim3变换
    g2o::Sim3 gSmw(CpKF->GetRotation(),pKF->GetTranslation(),1.0); //当前帧的sim3，其中s=1
    mg2oScw = gScm*gSmw;

    Sim3 operator *(const Sim3& other) const {
      Sim3 ret;
      ret.r = r*other.r;
      ret.t=s*(r*other.t)+t;
      ret.s=s*other.s;
      return ret;
    }
```
也就是说现在`mg2oScw`保存的是通过`sim3`对当前帧的位姿进行修正后的位姿，这个位姿的保存形式也为`sim3`。

# CorrectLoop()
当计算得到对当前帧需要修正的sim3变换后，后面就需要利用这个`sim3`变换进行回环优化。
首先对于当前帧局部视图中的关键帧，利用当前帧和这些关键帧之间的位姿关系，即可以得到这些关键帧通过回环帧修正的位姿，这个位姿与`mg2oScw`相同，保存形式为`sim3`，保存在`g2oCorrectedSiw`中。这部分代码为
```c++
	cv::Mat Tic = Tiw*Twc; //i是当前帧，c是回环帧
	cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
	cv::Mat tic = Tic.rowRange(0,3).col(3);
	g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
	g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
	//Pose corrected with the Sim3 of the loop closure
        CorrectedSim3[pKFi]=g2oCorrectedSiw;
```

之后即可进行`PoseGraph`优化。这里的PoseGraph优化是`Sim3`上的`PoseGraph`，即每个关键帧待优化的变量是7维变量，除了3维旋转，3维平移外，还有1维尺度。
这部分代码在`Optimizer::OptimizeEssentialGraph(...)`中。
对于当前帧局部视图中的关键帧，尺度的初始值为`ComputeSim3()`中计算得到的`s`，其他关键帧的初始值为1。
`Sim3`上的PoseGraph的残差与`SE3`上的区别不大，在`g2o`中是这样写的
```c++
    void computeError()
    {
      const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
      const VertexSim3Expmap* v2 = static_cast<const VertexSim3Expmap*>(_vertices[1]);

      Sim3 C(_measurement);
      Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
      _error = error_.log();
    }
```
要注意的是`sim3`上的求逆操作和对数映射操作，与`se3`上是不同的。

在`sim3`进行PoseGraph求解之后，得到的位姿全部用`sim3`来表示，这里就存在一个从`sim3`到`se3`的转换，
即`Sim3:[sR, t;0, 1] -> SE3:[R, t/s;0, 1]`。


# 其他部分
上述的整个sim3回环优化过程省略了两部分的内容。
1.把当前帧和其局部视图下的关键帧通过sim3变换到一个正确的位姿之后，还需要对这些帧中能观测到的地图点进行更新和融合，并且重新建立局部视图关系。
2.进行了sim3的PoseGraph后，还会进行一次 Global BA 的优化，以期得到更加准确的关键帧位姿和地图点。
这部分内容与sim3的联系不是很大，因此没有展开详述，有兴趣的直接撸代码。

> 强烈推荐参考文献[2]，扎实！

# 参考文献
[1]Horn B K P. Closed-form solution of absolute orientation using unit quaternions[J]. JOSA A, 1987, 4(4): 629-642.
[2]Strasdat H. Local accuracy and global consistency for efficient visual SLAM[D]. Department of Computing, Imperial College London, 2012.



