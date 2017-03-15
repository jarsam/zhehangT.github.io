---
title: 李代数求导
date: 2017-03-16 10:11:25
tags:
- 李群李代数
categories:
- 机器人事业
- SLAM基础
description: 关于学习SLAM问题中有关李代数求导的简单记录。
---
<!-- more -->

# 摘要
上一篇我们废了这么大的劲从李群变成了李代数，那李代数到底是为了啥？
其实李代数是大有用处的。在SLAM问题中，如果要使地图和位姿的结果更准取，通常会涉及到优化问题，这个优化问题往往与旋转矩阵或者变换矩阵有关。但是之前我们提到过，$SO(3)$和$SE(3)$是不满足加法运算的，也就是说当$SO(3)$和$SE(3)$作为优化变量的时候，是无法对其求导的，不能求导那还怎么玩？这时候李代数拯救了世界。

# BCH公式
首先来看看$SO(3)$上进行乘法运算时，$\mathfrak{so}(3)$上是一种怎样的运算。

BCH公式定义了两个李代数指数映射乘积的完整形式。对完整形式做近似即可得到我们关心的两个公式：
$
R\_1 R\_2 = \textbf{exp}([\phi\_1]\_{\times})\textbf{exp}([\phi\_2]\_{\times}) \approx \begin{equation}
\begin{cases}
\textbf{exp}( J\_l(\phi\_2)^{-1} \phi\_1 + \phi\_2    )  & \textbf{if  } \phi\_1 \textbf{ is samll}\\\
\textbf{exp}( J\_r(\phi\_1)^{-1} \phi\_2+ \phi\_1    )   & \textbf{if  } \phi\_2 \textbf{ is samll}
\end{cases}
\end{equation}
$

第一个称为左乘模型，第二个称为右乘模型。

$J\_l$即为$\mathfrak{se}(3)$到$SE(3)$转换时求得的$J$，称为左乘近似雅可比：
$J\_l = J=\dfrac{\sin \theta}{\theta} I + (1- \dfrac{\sin \theta}{\theta})aa^T + \dfrac{1-\cos \theta}{\theta}[a]\_{\times}$

它的逆为：
$J\_l^{-1} = \dfrac{\theta}{2} \cot \dfrac{\theta}{2} I + (1- \dfrac{\theta}{2}\cot \dfrac{\theta}{2})aa^T - \dfrac{\theta}{2}[a]\_{\times}$

右乘雅可比仅需要对自变量取负号即可：
$J\_r(\phi)=J\_l(-\phi)$ 

同样如果在李代数上做加法,则有：

$\textbf{exp}([\phi+ \Delta\phi]\_{\times}) = \textbf{exp}([J\_l \Delta\phi]\_{\times}) \textbf{exp}([\phi]\_{\times})=\textbf{exp}([\phi]\_{\times})\textbf{exp}([J\_r \Delta\phi]\_{\times}) $ 

在$\mathfrak{se}(3)$上的运算与$\mathfrak{so}(3)$类似，只是雅可比矩阵的求法不同，而且比较复杂，跳过吧。


# 李代数求导1.0
用李代数进行优化首先涉及的就是李代数的求导问题。
假设空间中的点p进行了一次旋转R，R对应的李代数为 $\phi$。则根据指数映射有$R=\mathbf{exp}([\phi]\_{\times})$。根据导数定义对$\phi$求导，有：

$
\begin{equation}
\begin{aligned}
\dfrac{\partial(\textbf{exp}([\phi]\_{\times})p)}{\partial\phi} &= \lim\_{\Delta \phi \rightarrow 0} \dfrac{\textbf{exp}([\phi + \Delta \phi]\_{\times})p-\textbf{exp}([\phi]\_{\times}) p}{\Delta \phi} \\\
&= ... \\\
&= -[Rp]\_{\times}J\_l
\end{aligned}
\end{equation}
$ 

公式推导过程中有BCH线性近似，泰勒展开舍去高阶项后近似，然后巴拉巴拉巴拉...最终得到了这个导数。但是这个导数包含了一个比较复杂的$j\_l$，有没有更简单的呢？


# 李代数求导2.0
对$R$进行一次扰动$\Delta R$，扰动对应的李代数为$\Delta \phi$，则求导过程有：

$
\begin{equation}
\begin{aligned}
\dfrac{\partial(\textbf{exp}([\phi]\_{\times})p)}{\partial(\phi))} &= \lim\_{\Delta \phi \rightarrow 0} \dfrac{\textbf{exp}([\Delta \phi]\_{\times})\textbf{exp}([\phi]\_{\times}) p-\textbf{exp}([\phi]\_{\times}) p}{\Delta \phi} \\\
&= ... \\\
&= -[Rp]\_{\times}
\end{aligned}
\end{equation}
$ 

曾经的$j\_l$莫名奇妙的没了，或者说根本就没出现过。为什么同样是导数，会相差一个数呢？等我重新学完高数再来解释。

# 相似变换群
对于单目相机，会有一种特殊的群，称为相似变换群$Sim(3)$,以及对应的李代数$\mathfrak{sim}(3)$。
单目相机存在尺度不确定性。如果在单目SLAM中使用$SE(3)表示位姿$，那么由于尺度不确定性与尺度漂移，整个SLAM过程中的尺度会发生变化，这在$SE(3)$中未能体现出来，因此在单目情况下我们一般会显示地把尺度表达出来。用数学语言来说，对于空间中的点p，在相机坐标系下要经过一个**相似变换**，而非欧式变换：

$p'=\begin{bmatrix}
sR & t \\\ 
0^T & 1
\end{bmatrix} p= sRp + t$

在相似变换中，我们把尺度$s$表达了出来。它同时作用在$p$的三个坐标之上，对$p$进行了一次缩放。

$\mathfrak{sim}(3)$与$Sim(3)$也有对应的指数映射，对数映射。求导过程也类似。

$\mathfrak{sim}(3)=\begin{pmatrix}
\zeta = \begin{bmatrix}
\rho \\\
\phi \\\
\sigma
\end{bmatrix} \in \mathbb{R}^7,[\zeta]\_{\times}=\begin{bmatrix}
\sigma I + [\phi]\_{\times} & \rho \\\
0^T & 0
\end{bmatrix} \in \mathbb{R}^{4\times4}
\end{pmatrix}$

$S = \textbf{exp}([\zeta]\_{\times})=\begin{bmatrix}
e^\sigma \textbf{exp}([\phi]\_{\times})  & J\_s \rho \\\
0^T & 1
\end{bmatrix}
$

其中：
$s=e^\sigma, t=J\_s \rho$ 

因为$Sp$是四维的齐次坐标，$\zeta$是七维向量，$\mathfrak{sim}(3)$的导数应该是4x7的雅可比矩阵。为了方便起见，记$Sp$的前三维组成向量q,那么：

$
\dfrac{ \partial Sp }{ \partial(\zeta) } = \begin{bmatrix}
I & [-q]\_{\times} & q\\\
0^T & 0^T & 0
\end{bmatrix}
$ 

# 参考文献
1.《视觉SLAM十四讲》第四讲












