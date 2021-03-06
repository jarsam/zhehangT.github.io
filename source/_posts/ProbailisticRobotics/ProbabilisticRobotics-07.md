---
title: 《Probabilistic Robotics》 读书笔记 07
date: 2016-10-17 21:35:55
tags: Probabilistic Robotics
categories:
- 机器人事业
- 读书笔记
description: 《Probabilistic Robotics》第七章论述了如何利用概率机器人学解决机器人的定位问题。
---
<!-- more -->

# 摘要
之前几个章节学习了贝叶斯滤波以及如何实现贝叶斯滤波，学习了如何对动作和测量建模。这一章节将利用这些知识来解决在地图已知的环境中的机器人定位问题。

# 定位问题的分类

局部定位 vs 全局定位
1.位置追踪。机器人的初始位置是已知的。
2.全局定位。机器人的初始位置是未知的。
3.机器人绑架问题。机器人以为自己的初始位置是已知的，但实际上这个位置是错误的。

静态环境 vs 动态环境
1.静态环境。整个环境中只有机器人是运动的。
2.动态环境。整个环境中除了机器人，还有其他物体是运动的。

被动方法 vs 主动方法
1.被动定位。机器人并不是为了定位而移动的，机器人本身有其他的任务。
2.主动定位。机器人为了定位而移动的。

单机器人 vs 多机器人
1.单机器人定位。
2.多机器人定位。

# 马尔可夫定位
马尔可夫定位就是将地图信息融入到贝叶斯滤波中产生的变体。算法如下图所示。

![](1.png)
值得注意的是对于不同的定位问题，$bel(x\_0)$的初始化是有区别的。

对于位置追踪问题会使用指标函数。假设初始位置为$x\_0$。
$bel(x\_0)=\begin{cases}
1 & if \ x\_0=\bar{x\_0} \\\
0 & otherwise
\end{cases}$
但指标函数无法表示一个概率分布，因此也常常使用高斯分布来表示。
$bel(x\_0) \sim \mathcal N(x\_0; \bar{x\_0}, \Sigma)$

对于全局定位问题常常使用均匀分布。
$bel(x\_0)=\dfrac{1}{\vert X \vert}$
其中$\vert X \vert$为空间面积。

对于其他情况，往往与某些特定知识相关。比如假设机器人在门附近，则机器人在所有门附近的状态置信度会明显比其他位置更高。

# 扩展卡尔曼滤波定位

这里阐述的扩展卡尔曼滤波定位是基于第五章的速度动作模型和第六章的基于特征的感知模型实现的，并且测量数据与对应的路标是已知的,即关联关系是已知的。算法流程如下图：
![](2.png)
此处的数学公式推导见7.5.3小节。



# 评估关联关系
上述的扩展卡尔曼滤波定位算法是在测量数据和对应的路标关联关系已知的前提下的。但在实际应用中，这种关联关系往往是不可知的，因此需要对关联关系进行评估。改进后的算法流程如下图：

![](3.png)
其中第6行到第12行以及第14行进行了关联关系的评估，得到了关联度最高的路标的索引。


此处的数学公式推导见7.6.2小节。

# 多假设追踪
当无法通过当前数据得出一个可靠的状态可信度时，可以通过多假设追踪算法(MHT)让假设继续传递，延迟可信度的判定，从而通过后续的测量数据解决这种不确定性，得到更可靠的结果。计算公式如下：

$bel(x\_t) = \dfrac{1}{\sum\_l \varphi\_{t,l}}\sum\_l \varphi\_{t,l} \ \mathcal N(x\_t; \mu\_{t,l}, \Sigma\_{t,l})$
其中$l$代表所有混合成分(mixture component)的索引。$\varphi$代表对应成分的权重。

关于MHT算法的细节以后再详细分析。

# 总结
在实际应用中，以上几个算法依然会存在很多的问题。比如遍历路标时的搜索效率问题，特征之间的关联性问题以及检测到异常物体的处理问题。

最近几章的笔记写的很简单，主要是因为很多内容只停留在公式层面，无法与实际场景相联系。因此对书中的内容理解比较浅显。随着学习的深入，也许会对当前的一些笔记进行修正和深入。






















