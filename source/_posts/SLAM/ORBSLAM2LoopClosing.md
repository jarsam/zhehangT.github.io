---
title: ORB_SLAM2中的回环检测
date: 2018-04-11 10:11:25
tags:
- ORB-SLAM
- 源码学习
categories:
- 机器人事业
- SLAM
description: ORB_SLAM2中的回环检测相关代码学习
---
<!-- more -->

# 摘要
ORB_SLAM2 中的回环检测是相对独立的模块，主要采用了词袋模型来进行检测回环，通过建立当前帧和回环帧之间的匹配关系，来修正视觉里程计的局部误差。

# 词袋模型
词袋是通过离线的方式建立的。ORB_SLAM2 中的词袋是基于ORB特征来生成的，主要通过以下几个步骤。
1、采集与目标场景相类似，并且图像具有多样性的图像数据集，并提取所有图像的ORB特征描述子。
2、将抽取的特征描述子用 k-means++ 算法聚类，将描述子划分成 k 类。 
3、将划分得到的k类特征描述子，继续利用 k-means++ 算法做聚类 
4、按照上述循环n，最终可以得到 $k^n$ 种不同的ORB描述子，即 $k^n$个word。
自此词袋就建立完成。不难发现，通过这种方式建立的词袋是k叉树的结构 ，当进行描述子和word之间的匹配时，可以通过分层的方式进行搜索匹配，可以极大的提升搜索速度。
word本身是一个描述子，这个描述子是通过前面所说的聚类的方式得到的，因此可以认为是某一类相似的描述子的平均表达。word除了描述子之外，还会计算WordValue值。直观的讲，这个WordValue描述了这个word到底重不重要，WordValue值越大，表示这个word对于图像的区分越明显。WordValue最常用的计算方式为IDF-TF，这部分内容在十四讲里面有提及，不再详述。

# 图像帧的词袋表达
图像帧的词袋表达依赖于两个数据结构：
```c++
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;
```
`DBoW2::BowVector mBowVec`是从`std::map<WordId, WordValue>`继承的，这里的`WordId`的取值范围即为生成词袋模型时word的数目($k^n$)。这里存储的`WordValue`是 word 的 WordValue的累加值，当图像中的多个特征都与WordId对应的word相似，这里存储的`WordValue`是对 word 中的 WordValue的多次累加。
`DBoW2::FeatureVector mFeatVec`是被称为 direct index 的东西，用来支持图像与图像之间特征匹配的速度。其从`std::map<NodeId, std::vector<unsigned int> >`继承，`NodeId`是词袋树中某一层的节点的id，在ORB_SLAM2中是倒数第四层，`std::vector<unsigned int>`保存的是与该节点下的 word 匹配的特征点的id。

# DetectLoop()

ORB_SLAM2中回环检测的相关代码主要在`LoopClosing.cc`文件中，在ORB_SLAM2的运行过程中，`LoopClosing`运行在单独的一个线程中。
对于每一个关键帧，通过`DetectLoop()`函数进行是不是存在回环的判断。主要经过了如下几个步骤。
1、计算当前帧与其关联视图中的关键帧之间图像词袋相似度，取其中的最小值，检测回环帧时，其与当前帧的相似度要大于这个最小值。
2、通过`KeyFrameDatabase::DetectLoopCandidates()`函数获取回环候选帧。为了加快候选帧的检索，这里采用了一种 inverse index 的技巧，依赖于`std::vector<list<KeyFrame*>> mvInvertedFile`。这个vector的大小为word的数目($k^n$)，下标为i的地方存储的是一系列关键帧，这些关键帧中包含 WordId=i 的 word。这期间还会统计候选帧与当前帧之间有多少个相同的word，存储于KeyFrame类的mnLoopWords成员变量中。候选帧保存在list<KeyFrame*> lKFsSharingWords中。之后会对lKFsSharingWords作两次剔除。首先是保留mnLoopWords大于其最大值的80%的候选帧。其次是计算候选帧所有关联视图中相邻帧与当前帧mnLoopWords的累加和，保留累加和大于最大值75%的候选帧。
3、对从`KeyFrameDatabase::DetectLoopCandidates()`得到的候选帧`vpCandidateKFs`进一步进行一致性检验。对于`vpCandidateKFs`里面的每一个关键帧，作为当前关键帧。我们找出其关联视图中的关键帧们组成一个当前整体`spCandidateGroup`。如果当前关键帧是第一次检测到回环，直接把这个`spCandidateGroup`整体，以分数0直接放到`mvConsistentGroups`中。如果不是第一次检测到回环，就从`mvConsistentGroups`中依次取出里面的元素pair<set<KeyFrame*>,int>的first(sPreviousGroup)，即为之前的`spCandidateGroup`。只要是当前整体中的任意一个关键帧能在以前整体里面找到，就要将当前整体的得分加1，并把当前整体放到mvConsistentGroups里面。如果当前整体的得分大于3（mnCovisibilityConsistencyTh）了的话，当前帧就通过了一致性检测，把当前帧放到`mvpEnoughConsistentCandidates`。如果`mvpEnoughConsistentCandidates`不为空，则检测到回环。

# ComputeSim3()
检测到回环帧之后，开始调用`ComputeSim3()`函数计算当前帧和回环帧之间的平移和旋转。主要经历了如下几个步骤。
1、基于回环帧和当前帧的词袋，通过`matcher.SearchByBoW()`寻找回环帧和当前帧之间的ORB特征匹配。注意因为这里可能有不止一个候选帧，对于每个候选帧都会与当前帧进行特征匹配。
2、对于步骤1中的每一对特征匹配，都会构建一个Sim3问题求解，计算出当前帧和回环帧之间的平移和旋转，这里会通过RANSACS去剔除一些异常的回环候选帧。
3、当通过Sim得到一个初始的平移和旋转之后，会通过`matcher.SearchBySim3()`来寻找更多的特征匹配。主要的思路就是，对于`matcher.SearchByBoW()`中没有被匹配的地图点，分别投影到回环帧和当前帧中，去搜索地图点和特征点之间的匹配，最后对匹配进行验证。
4、有了前三步的异常点剔除和特征点匹配，利用重投影误差构造sim3优化问题，通过`Optimizer::OptimizeSim3()`优化，得到更准确的平移和旋转。
5、最后把回环帧和其关联视图中的关键帧们中的所有地图点投影到当前帧搜索特征匹配，如果匹配的数目大于40，则回环被接受。

# CorrectLoop()
1、通过上一步的`ComputeSim3()`，可以对当前帧的位姿进行调整。利用之前已知的两帧之间的位姿关系，就可以对所有的位姿进行调整。
2、之后利用调整过的位姿更新这些相连关键帧对应的地图点。
3、对回环帧和当前帧中的地图点进行融合，并将融合后的地图点重新投影到回环帧和当前帧以及关联视图中的其他帧中，建立新的匹配关系。
4、根据新的匹配关系，通过`Optimizer::OptimizeEssentialGraph`进行全局位姿图的优化。最后利用`RunGlobalBundleAdjustment`进行全局的BA优化。

至此，整个回环优化完成。

# 总结
回环优化这一块从代码上看还是比较复杂，这部分代码还需要进一步深入阅读。












