---
title: 四元数
date: 2017-03-27 10:11:25
tags: 刚体运动
categories:
- 机器人事业
- SLAM基础
description: 关于学习四元数的简单记录。
---
<!-- more -->

# 摘要
在学习三维空间中的刚体运动时，我们提到四元数在描述旋转时只有四个自由度而且也没有奇异性。
这篇博客将简单记录四元数的相关内容

# 四元数
旋转矩阵具有冗余性，欧拉角和旋转向量不冗余但是具有奇异性。
接下来出场的四元数就厉害了，只有四个自由度而且也没有奇异性。它是一种类似于复数的表达方式。
四元数内容较多，另外再单独记录。
四元数$q$拥有一个实部和三个虚部。如下：
$q=q\_0+q\_1 i+q\_2 j + q\_3k$
其中：
$
\left\\{
\begin{align}
&i^2=j^2=k^2=-1 \\\
&ij=k,ji=-k \\\
&jk=i,kj=-i \\\
&ki=j,ik=-j
\end{align}
\right.
$

假设某个旋转绕单位向量$n=[n\_x, n\_y, n\_z]^T$进行了角度$\theta$的旋转，那么对应的四元数为：
$
q = [\cos{\dfrac{\theta}{2}}, n\_x\sin \dfrac{\theta}{2}, n\_y \sin \dfrac{\theta}{2}, n\_z \sin \dfrac{\theta}{2}]
$

反之也可以从单位四元数中计算出对应旋转轴和夹角：
$
\left\\{
\begin{align}
&\theta = 2\arccos q\_0 \\\
&[n\_x, n\_y, n\_z]^T=[q\_1, q\_2, q\_3]^T / \sin{\dfrac{\theta}{2}} \\\
\end{align}
\right.
$

对$\theta$加上$2\pi$可以得到一个相同的旋转，但此时对应的四元数变成了$-q$。因此任意的旋转都可以由两个互为相反数的四元数表示。

# 四元数的运算
假设有两个四元数$q\_a$和$q\_b$。如下：

$
q\_a=[s\_a,v\_a]=[s\_a+x\_ai+y\_aj+z\_ak] \\\
q\_b=[s\_b,v\_b]=[s\_b+x\_bi+y\_bj+z\_bk]
$

1.加法和减法
$q\_a \pm q\_b = [s\_a \pm s\_b, v\_a \pm v\_b]$

2.乘法
乘法是把$q\_a$的每一项与$q\_b$每项相乘，最后相加。
$q\_aq\_b = [s\_a s\_b - v\_a^Tv\_b, s\_av\_b+s\_bv\_a + v\_a \times v\_b]$

3.共轭
四元数的共轭是把虚部去成相反数：
$q\_a^* = [s\_a, -v\_a]$

四元数共轭与自己本身相乘，会得到一个实四元数，其实部为模长的平方：
$q^*q = [s\_a^2 + v^Tv, 0]$


4.模长
$\parallel q\_a \parallel = \sqrt{s\_a^2+x\_a^2+y\_a^2+z\_a^2}$

可以验证两个四元数乘积的模即为模的乘积。这保证单位四元数相乘后仍然是单位四元数。
$ \parallel q\_a q\_b \parallel = \parallel q\_a \parallel \parallel q\_b \parallel $


5.逆
$q^{-1}=q^* /  \parallel q \parallel ^2$ 



6.数乘与点乘
$kq = [ks, kv]$

点乘是指两个四元数每个位置上的数值分别相乘：
$q\_a q\_b = s\_as\_b + x\_ax\_bi + y\_ay\_bj + z\_a z\_b k$ 

# 四元数的旋转表示
假设空间中有一点$p=[x,y,z]$，其绕旋转轴$n$，进行了角度为$\theta$的旋转得到了点$p'$，对应的四元数为q。

则可以验证：
$p' = qpq^{-1}$



# 参考文献
1. 《视觉SLAM十四讲》第三讲



