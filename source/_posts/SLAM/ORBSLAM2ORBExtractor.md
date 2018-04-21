---
title: ORB_SLAM2中的ORB特征提取与匹配
date: 2018-01-11 10:11:25
tags:
- ORB-SLAM
- 源码学习
categories:
- 机器人事业
- SLAM
description: ORB_SLAM2中的ORB特征提取与匹配的相关代码学习
---
<!-- more -->

# 摘要
最近在尝试将 VINS 的光流匹配替换为 ORBSLAM 中的 ORB 描述子匹配，看看能不能减少 VINS 在没有回环的情况下的定位精确度。以前只大致知道ORB特征提取和特征匹配理论过程，现在将 ORBSLAM 中的相关代码学习了一遍，本篇博客将作简单记录。

# ORB特征
首先简单回顾一下什么是 ORB 特征。ORB 特征主要由两部分组成，即**关键点**和**描述子**。**关键点**为 ORB 特征在图像中出现的图像坐标，ORB 特征中的关键点称为“Oriented Fast”，是一种改进的角点。**描述子**则为该角点的二进制描述，称为BRIEF特征描述。对于三维空间中的某个点，即使出现在多张图片中且关键点的像素坐标互不相同，其描述子应该是相似的。有了描述子就可以进行图像之间的特征匹配，其方法就是计算描述子之间的汉明距离，汉明距离越小，两个 ORB 特征就越相似。
判断*关键点*的条件以及描述子的计算可以参考《视觉SLAM十四讲》的第七讲。之前的博文[《图像特征匹配 》](http://zhehangt.win/2017/03/03/SLAM/FeatureMatching/)中也有简单提及。
有了这些基础理论，接下来可以进一步学习ORB_SLAM中是如何进行 ORB 特征提取和图像之间的特征匹配的。

# 数据结构
ORB_SLAM2 中与 ORB 特征提取和特征匹配相关的数据结构主要就是 Frame类 、 KeyFrame类 和 MapPoint类。Frame类 和 KeyFrame类  ORB_SLAM 中基本上代替了原始图像，当通过原始图像构造了一个 Frame 或者 KeyFrame 之后， 原始图像就被丢弃了。之后所有的 Tracking，Mapping 和 Loop Closing 都是通过 Frame 或者 KeyFrame 来完成的。对于每个原始图像，首先是将其构造成 Frame类，而构造成 Frame类 的过程中，最重要的一步就是提取 ORB 特征。
Frame类 中与提取 ORB 特征最紧密相关的几个成员变量如下：


```c
//ORB特征提取器
ORBextractor*  mpORBextractorLeft

//原始关键点图像坐标
std::vector<cv::KeyPoint>  mvKeys;
//经过矫正模型矫正的关键点图像坐标
std::vector<cv::KeyPoint>  mvKeysUn;

//存储ORB描述子，在矩阵的第i行存储的是上面vector中第i个关键点的描述子
cv::Mat mDescriptors;
```

KeyFrame类 中与 ORB 特征相关的成员变量与 Frame类 中类似。

而每一个MapPoint中同样会存储一个ORB 描述子。之前我们都是直接在帧与帧之间建立特征匹配，而通过MapPoint中的 ORB 描述子，我们还可以直接建立 MapPoint 和 Frame 之间的关联关系。
这个描述子被称为该 MapPoint 的最具有代表性的 ORB 描述子。其计算的基本思路是，在所有能观测到该 MapPoint 的关键帧的描述子中，选择其中一个描述子，使其与剩余的描述子之间的汉明距离之和最小。
这部分的实现代码为 MapPoint类 的 `void ComputeDistinctiveDescriptors();` 函数。


# 特征提取
ORB_SLAM 中关于特征提取的相关实现在 ORBextractor类 中。 当每一帧的图像数据被送到 Tracking类 进行追踪时，都会将图像数据构造成 Frame类，然后通过 Tracking类 中构造的 `ORBextractor* mpORBextractorLeft;`完成特征提取，特征提取的结果被保存在 Frame类中的成员变量`std::vector<cv::KeyPoint> mvKeys;` 和 `cv::Mat mDescriptors;` 中。

首先来看一看 ORBextractor类 的构造函数。
```c
//nfeatures表示要在当前图像中提取的ORB特征点的数目

//为了使ORB特征具备尺度一致性，通过采样多个层级的图像金字塔进行ORB特征提取，
//scaleFactor表示相邻两层之间ORB特征点数目的倍数，nlevels表示图像金子塔的层数
//参考TUM1.yaml文件中的参数，每一帧图像共提取1000个特征点，分布在金字塔8层中，层间尺度比例1.2
//计算下来金字塔0层大约有217个特征点，7层大约有50个特征点

//在提取FAST角点时，会把中心像素和周围像素之间的亮度差作为判断标准。
//iniThFAST表示这个 threshold 的初始值，当提取的角点数目不够时，则会采用 minThFAST 作为 threshold。
ORBextractor(int nfeatures, float scaleFactor, int nlevels,  int iniThFAST, int minThFAST);
```

在构造函数中主要完成了以下几个事情。
首先根据 scaleFactor 和 nlevels 计算每一层图像金字塔应该提取的 ORB特征 数目。保存在 `std::vector<int> mnFeaturesPerLevel;`
其次是构造了一个`std::vector<cv::Point> pattern;`成员变量。这个成员变量将用于描述子的计算。
最后是构造了一个`std::vector<int> umax;`成员变量。这个成员变量将用于 ORB特征 的方向向量。

在 ORBextractor类 中扮演的最重要的角色就是重载了( )操作符 `void operator()( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors);`。在将图像数据构造成 Frame类时，即在Frame类的构造函数内调用了 ORBextractor类 的( )操作符 `(*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors);`，从而完成对当前图像的 ORB特征 的提取。
下面我们将着重分析这个操作符函数。

在这个操作符函数中，首先进行的是对图像进行金字塔降采样，其调用了函数`void ComputePyramid(cv::Mat image);`。其降采样过程主要是根据之前 scaleFactor，计算每一层金字塔图像的分辨率大小，然后通过`void resize( InputArray src, OutputArray dst, Size dsize, double fx = 0, double fy = 0, int interpolation = INTER_LINEAR );`得到新的图像，存储在成员变量`std::vector<cv::Mat> mvImagePyramid;`。

完成图像金字塔降采样之后，将调用函数`void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);`计算每一层金字塔图像的关键点坐标，存储在局部变量 `vector < vector<KeyPoint> > allKeypoints;`中。allKeypoints 的第一维表示金字塔的层数，第二维表示在该层上提取的各个关键点坐标。得到 allKeypoints 的计算过程如下：
1）将图像划分为像素大小为 W=30 的网格。在每个网格上调用函数 `void FAST( InputArray image, CV_OUT std::vector<KeyPoint>& keypoints, int threshold, bool nonmaxSuppression=true );`进行关键点坐标的提取。这些关键点保存在局部变量`vector<cv::KeyPoint> vToDistributeKeys;`
2）由于在网格中提取的关键点很大程度上也是’扎堆‘出现的，因此在一定区域内仅保留响应极大值的角点，避免角点集中的问题。这个过程是通过将所有当前提取的关键点分配到平面四叉树中去实现的，即函数`std::vector<cv::KeyPoint> DistributeOctTree( ... );`。将所有的角点根据空间关系分配到一定数目的四叉树节点中去，然后取每个节点上的最大响应点。
3）最后将计算每个角点的方向信息。该过程将对每个角点调用`float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)`函数。

有了关键点坐标，最后就是计算描述子。对于每个关键点，都调用了`void computeOrbDescriptor(const KeyPoint& kpt, const Mat& img, const Point* pattern, uchar* desc)`函数，每个描述子都是一个32位的 unchar 类型的字符串。

至此，整个特征提取过程就完成。

# 特征匹配

ORB_SLAM 的特征匹配是通过 ORBmatcher类 完成的。在 ORBmatcher类 中，提供了多种寻找特征匹配方式。简单看一下 ORBmatcher类 的定义。
```c
//寻找当前帧和地图点之间的匹配，用于TrackLocalMap()
int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);

//寻找当前帧和前一帧之间的匹配，用于TrackWithMotionModel()
int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

//寻找当前帧和关键帧之间的匹配，用于检测到回环时，与候选的回环帧之间建立匹配
int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

//当计算出当前帧和回环帧之间的sim(3)变换之后，建立当前帧和地图点之间的匹配
int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

//用于回环检测时当前帧
int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);

//用于初始化时，寻找当前帧和前一帧之间的匹配
int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

//用于进行三角时，寻找当前帧和其他帧之间的匹配
int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12, std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

```

其实搞了这么多种匹配方式，无非就是为了限制待匹配特征的集合大小，而不是对整张图上的每一个特征点都进行汉明距离的计算，从而减小计算开销，加快匹配速度。
比如在`TrackWithMotionModel()`中调用`int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);`寻找当前帧和前一帧之间的匹配时，将通过如下方式加速匹配速度。

1) 对于前一帧能观测到所有地图点`cv::Mat x3Dw`，根据当前帧在世界坐标系下的旋转`const cv::Mat Rcw`和平移`const cv::Mat tcw`计算该地图点在当前帧相机坐标系下的坐标`cv::Mat x3Dc = Rcw*x3Dw+tcw;`
2）利用相机的内参矩阵将`x3Dc`进行投影，得到该地图点在当前帧的像素坐标（u，v）。以（u，v）为搜索关联关系的中心，radius为半径，通过`vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel)`方法返回候选的 ORB特征。选择其中与当前地图点汉明距离最小的 ORB特征，即建立起了关联关系。这个关联关系存储于`std::vector<MapPoint*> mvpMapPoints;`。`mvpMapPoints`的索引即为在该图像帧中的ORB特征的id值。



# 总结
博客写得再详细，依旧会忽略很多代码中的细节。越来越能理解一项工作，论文只能描述20%，剩下的80%全在代码里。














