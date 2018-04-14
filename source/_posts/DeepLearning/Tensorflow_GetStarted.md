---
title: TensorFlow Get Started 学习笔记
date: 2017-06-26 13:39:49
tags: 深度学习
categories:
- 深度学习
- TensorFlow
description: 关于 TensorFlow Get Started 的学习笔记
---
<!-- more -->

# 摘要
关于 [Tensorflow Get Started](https://www.tensorflow.org/get_started/) 的学习笔记。

# 主要内容
### [Getting Started With TensorFlow](https://www.tensorflow.org/get_started/get_started)
TensorFlow的基本数据单元被称为tensor。每一个tensor实质上是具有特定维度，特定类型的多维数组。
TensorFlow对tensor的每一次操作都是一个节点，对tensor的多个操作就形成了整个的计算任务，整个计算任务被称为图。

首先可以对tensor进行三种不同类型的操作，赋予其一定的属性，从而得到最原始的数据节点。
**tf.constant**：赋予tensor常量属性
**tf.placeholder**：表示tensor在用来进行计算之前才赋予相应的数据，通常用来表示数据集
**tf.Variable**：表示tensor在计算时数值会变化，通常用来表示模型参数

对单个数据节点或者数据节点之间可以进行计算操作，得到计算节点。比如
**tf.square**：表示平方
**tf.add**：表示乘法
**tf.matmul**：表示加法
**...**


对于每个计算任务，首先将tensor构造成数据节点，然后对数据节点进行多次的计算操作，从而得到最终的computational graph。有了computational graph之后，需要通过session的run方法来执行计算，从而得到最终的计算结果。

TensorFlow中最特殊的计算操作为optimizers。

除了通过最简单的节点操作构造计算任务，TensorFlow还提供了更高层的tf.contrib.learn包，来快速的构造计算任务。

### [MNIST For ML Beginners](https://www.tensorflow.org/get_started/mnist/beginners)
这一章节，针对 MNIST 分类问题，构造了一个softmax分类器，交叉熵作为损失函数，利用梯度下降进行训练。
代码整体流程如下：
1. 定义模型。
2. 基于模型定义交叉熵损失函数
3. 构造梯度训练的优化器，损失函数和训练集作为优化器的参数对模型进行训练，训练共进行了1000次迭代
4. 基于模型定义正确率函数，在测试集上进行测试，得到正确率


### [Deep MNIST for Experts](Deep MNIST for Experts)
这一章节，针对 MNIST 分类问题，构造了一个多层的卷积神经网络，交叉熵作为损失函数，利用梯度下降进行训练。
代码整体流程如下：
1. 定义模型，通过deepnn()函数完成。模型为卷积层，池化层，卷积层，池化层，全连接层，dropout层，全连接层
2. 基于模型定义交叉熵损失函数
3. 构造Adam优化器
4. 定义正确率函数
5. 训练共进行20000次迭代，每100次计算一次正确率

### [TensorFlow Mechanics 101](https://www.tensorflow.org/get_started/mnist/mechanics)
这一章节，针对 MNIST 分类问题，构造了一个全连接的神经网络，交叉熵作为损失函数，利用梯度下降进行训练。
其中mnist.py文件中定义了全连接神经网络模型，定义了梯度下降优化器。
可以用TensorBoard显示损失函数的值变化和computational graph。




### [tf.contrib.learn Quickstart](https://www.tensorflow.org/get_started/tflearn)
tf.contrib.learn是TensorFlow中的高层API，可以快速地进行机器学习模型的构建与训练。这一章节，针对 Iris 分类问题。
通过tf.contrib.learn.DNNClassifier方法，可以快速构造一个多层的神经网络。
通过fit方法可以进行自动化的训练。
通过evaluate方法可以进行模型评估。
通过predict方法可以利用模型进行预测。

### [Building Input Functions with tf.contrib.learn](https://www.tensorflow.org/get_started/input_fn)
在使用tf.contrib.learn进行机器学习模型的构建与训练时，支持对原始数据进行预处理。


### [Logging and Monitoring Basics with tf.contrib.learn](https://www.tensorflow.org/get_started/monitors)

在训练机器学习模型时，通常需要实时的跟踪和评估模型，这时可以借助 Monitor 包。



### [TensorBoard: Visualizing Learning](https://www.tensorflow.org/get_started/summaries_and_tensorboard)

TensorBoard 涉及到的运算，通常是在训练庞大的深度神经网络中出现的复杂而又难以理解的运算。

为了更方便 TensorFlow 程序的理解、调试与优化，可以用 TensorBoard 来展现 TensorFlow 图像，绘制图像生成的定量指标图以及附加数据。

TensorBoard 通过读取 TensorFlow 的事件文件来运行。TensorFlow 的事件文件包括了你会在 TensorFlow 运行中涉及到的主要数据。下面是 TensorBoard 中汇总数据（Summary data）的大体生命周期。

1. 首先，创建你想汇总数据的 TensorFlow 图，然后再选择你想在哪个节点进行汇总(summary)操作。
比如，假设你正在训练一个卷积神经网络，用于识别 MNISt 标签。你可能希望记录学习速度(learning rate)的如何变化，以及目标函数如何变化。通过向节点附加scalar_summary操作来分别输出学习速度和期望误差。然后你可以给每个 scalary_summary 分配一个有意义的 标签，比如 'learning rate' 和 'loss function'。
或者你还希望显示一个特殊层中激活的分布，或者梯度权重的分布。可以通过分别附加 histogram_summary 运算来收集权重变量和梯度输出。
2. 在TensorFlow中，所有的操作只有调用执行方法，方法才会真正执行。而summary操作是相互独立的，如果对每个summary操作调用执行操作会相当繁琐，因此，需要通过tf.summary.merge_all汇总所有这些被summary的节点。
3. 汇总之后即可调用执行方法完成汇总操作，会将所有数据生成一个序列化的Summary protobuf对象。
4. 最后，为了将汇总数据写入磁盘，需要将汇总的protobuf对象传递给tf.summary.FileWriter。

### [TensorBoard: Embedding Visualization](https://www.tensorflow.org/get_started/embedding_viz)
Embeddings 在机器学习模型中非常常见，特别是在推荐系统，NLP等等。TensorFlow提供了Embedding Projector可视化工具，对embeddings数据进行可视化显示。


### [TensorBoard: Graph Visualization](https://www.tensorflow.org/get_started/graph_viz)

利用 TensorFlow 构造的 computational graph 非常强大但往往又非常复杂，图表可视化在理解和调试时显得非常有帮助。

对于深度学习模型来说，一个computational graph往往包含上千个节点，同时显示上千个节点并不是一种可取的方式。TensorBoard 可以利用 tf.name_scope 来生成节点的命名空间，借助于命名空间，TensorBoard可以对节点实现层级显示。因此命名空间的设计是可视化的关键所在。

TensorBoard 的计算图可视化有两种形式。节点之间如果是数据相关的，用实线相连。节点之间如果是控制相关的，用虚线相连。

TensorBoard 还有很多技巧来优化计算图的显示，包括改变形状，改变颜色等等。



### [TensorBoard Histogram Dashboard](https://www.tensorflow.org/get_started/tensorboard_histograms)
可以显示数据直方图



# 参考文献
[Tensorflow Get Started](https://www.tensorflow.org/get_started/)