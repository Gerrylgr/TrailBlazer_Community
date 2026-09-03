# PCA 平面拟合：完整计算示例

本文通过四个三维点，完整演示局部 PCA 平面拟合中的以下步骤：

1. 计算点云质心；
2. 构造去中心化后的偏差向量；
3. 计算协方差矩阵；
4. 求解特征值和特征向量；
5. 根据最小特征值对应的特征向量获得拟合平面的法向量。

> [!NOTE]
> 本文统一按照
> $\lambda_1 \leq \lambda_2 \leq \lambda_3$
> 对特征值进行升序排列。不同库可能采用不同的输出顺序，实际使用时应先确认排序约定。
>
> `ground_segmentor.cpp` 使用的 [`Eigen::SelfAdjointEigenSolver`](https://libeigen.gitlab.io/eigen/docs-nightly/classEigen_1_1SelfAdjointEigenSolver.html) 会按升序返回特征值；`eigenvectors().col(i)` 是与第 $i$ 个特征值对应、二范数为 1 的特征向量。

---

## 1. 输入点

假设有以下四个点：

| 点 | $x$ | $y$ | $z$ |
| --- | ---: | ---: | ---: |
| $A$ | $-1$ | $-1$ | $0.5$ |
| $B$ | $-1$ | $1$ | $1$ |
| $C$ | $1$ | $1.4$ | $0$ |
| $D$ | $2$ | $-1$ | $0.5$ |

---

## 2. 计算质心

四个点的质心为：

$$
\begin{aligned}
c_x &= \frac{-1-1+1+2}{4}=0.25, \\
c_y &= \frac{-1+1+1.4-1}{4}=0.10, \\
c_z &= \frac{0.5+1+0+0.5}{4}=0.50.
\end{aligned}
$$

因此：

$$
\boxed{\mathbf{c}=(0.25,\ 0.10,\ 0.50)^T}
$$

---

## 3. 对所有点去中心化

令 $\mathbf{d}_i=\mathbf{p}_i-\mathbf{c}$，得到：

| 点 | 去中心化后的偏差向量 $\mathbf{d}_i$ |
| --- | --- |
| $A$ | $(-1.25,\ -1.10,\ 0)^T$ |
| $B$ | $(-1.25,\ 0.90,\ 0.50)^T$ |
| $C$ | $(0.75,\ 1.30,\ -0.50)^T$ |
| $D$ | $(1.75,\ -1.10,\ 0)^T$ |

每个偏差向量与自身的转置相乘，可得到一个 $3 \times 3$ 的外积矩阵：

$$
\mathbf{d}\mathbf{d}^{T}=
\begin{bmatrix}
d_x\\d_y\\d_z
\end{bmatrix}
\begin{bmatrix}
d_x & d_y & d_z
\end{bmatrix}
=
\begin{bmatrix}
d_x^2 & d_xd_y & d_xd_z\\
d_yd_x & d_y^2 & d_yd_z\\
d_zd_x & d_zd_y & d_z^2
\end{bmatrix}.
$$

四个偏差向量的外积分别为：

$$
\mathbf d_A\mathbf d_A^T=
\begin{bmatrix}
1.5625 & 1.375 & 0\\
1.375 & 1.21 & 0\\
0 & 0 & 0
\end{bmatrix},
$$

$$
\mathbf d_B\mathbf d_B^T=
\begin{bmatrix}
1.5625 & -1.125 & -0.625\\
-1.125 & 0.81 & 0.45\\
-0.625 & 0.45 & 0.25
\end{bmatrix},
$$

$$
\mathbf d_C\mathbf d_C^T=
\begin{bmatrix}
0.5625 & 0.975 & -0.375\\
0.975 & 1.69 & -0.65\\
-0.375 & -0.65 & 0.25
\end{bmatrix},
$$

$$
\mathbf d_D\mathbf d_D^T=
\begin{bmatrix}
3.0625 & -1.925 & 0\\
-1.925 & 1.21 & 0\\
0 & 0 & 0
\end{bmatrix}.
$$

---

## 4. 构造协方差矩阵

将四个外积矩阵相加并除以点数 $N=4$：

$$
\mathbf{C}
=\frac{1}{4}\sum_{i=1}^{4}\mathbf{d}_i\mathbf{d}_i^{T}
=
\begin{bmatrix}
1.6875 & -0.175 & -0.25\\
-0.175 & 1.23 & -0.05\\
-0.25 & -0.05 & 0.125
\end{bmatrix}.
$$

> [!NOTE]
> 这里使用总体协方差形式，即分母为 $N$。如果使用样本协方差形式，分母应为 $N-1$。两种定义会按相同比例缩放全部特征值，但不会改变特征向量，也不会改变本文使用的特征值比值。

### 4.1 协方差矩阵中各元素的含义

| 元素 | 统计含义 |
| --- | --- |
| $C_{11}=\mathrm{Var}(x)$ | $x$ 坐标的方差 |
| $C_{22}=\mathrm{Var}(y)$ | $y$ 坐标的方差 |
| $C_{33}=\mathrm{Var}(z)$ | $z$ 坐标的方差 |
| $C_{12}=C_{21}=\mathrm{Cov}(x,y)$ | $x$ 与 $y$ 是否同向变化 |
| $C_{13}=C_{31}=\mathrm{Cov}(x,z)$ | $x$ 与 $z$ 是否同向变化 |
| $C_{23}=C_{32}=\mathrm{Cov}(y,z)$ | $y$ 与 $z$ 是否同向变化 |

---

## 5. 求解特征值

特征值满足：

$$
\det(\mathbf C-\lambda\mathbf I)=0.
$$

首先写出：

$$
\mathbf C-\lambda\mathbf I=
\begin{bmatrix}
1.6875-\lambda & -0.175 & -0.25\\
-0.175 & 1.23-\lambda & -0.05\\
-0.25 & -0.05 & 0.125-\lambda
\end{bmatrix}.
$$

沿第一行展开行列式：

$$
\det(\mathbf C-\lambda\mathbf I)
=(1.6875-\lambda)M_{11}
-(-0.175)M_{12}
+(-0.25)M_{13},
$$

其中：

$$
\begin{aligned}
M_{11}
&=(1.23-\lambda)(0.125-\lambda)-(-0.05)(-0.05)\\
&=\lambda^2-1.355\lambda+0.15125,\\[4pt]
M_{12}
&=(-0.175)(0.125-\lambda)-(-0.05)(-0.25)\\
&=0.175\lambda-0.034375,\\[4pt]
M_{13}
&=(-0.175)(-0.05)-(1.23-\lambda)(-0.25)\\
&=0.31625-0.25\lambda.
\end{aligned}
$$

代入并分别展开：

$$
\begin{aligned}
(1.6875-\lambda)M_{11}
&=-\lambda^3+3.0425\lambda^2-2.4378125\lambda+0.255234375,\\
0.175M_{12}
&=0.030625\lambda-0.006015625,\\
-0.25M_{13}
&=-0.0790625+0.0625\lambda.
\end{aligned}
$$

整理得到特征方程：

$$
\boxed{\lambda^3-3.0425\lambda^2+2.3446875\lambda-0.17015625=0}.
$$

按升序排列，三个特征值为：

$$
\boxed{
\lambda_1\approx0.0808221,\qquad
\lambda_2\approx1.1849242,\qquad
\lambda_3\approx1.7767537
}.
$$

### 5.1 Planarity 与 Flatness

在本文采用的升序约定下：

平面度（Planarity）为：

$$
P
=\frac{\lambda_2-\lambda_1}{\lambda_3}
=\frac{1.1849242-0.0808221}{1.7767537}
\approx\boxed{0.6214},
$$

平坦度（Flatness）为：

$$
F
=\frac{\lambda_1}{\lambda_3}
=\frac{0.0808221}{1.7767537}
\approx\boxed{0.04549}.
$$

最小特征值相对于最大特征值较小，说明点云在某一个方向上的离散程度明显低于另外两个方向，因而具有较明显的局部平面结构。

### 5.2 补充：行列式的含义

行列式是由方阵计算得到的标量。几何上，它表示该矩阵对应的线性变换对面积或体积的有向缩放倍数。

对于 $2\times2$ 矩阵：

$$
\begin{vmatrix}
a & b\\
c & d
\end{vmatrix}
=ad-bc.
$$

对于 $3\times3$ 矩阵，可以沿任意一行或一列展开。例如沿第一行展开：

$$
\begin{vmatrix}
a_{11} & a_{12} & a_{13}\\
a_{21} & a_{22} & a_{23}\\
a_{31} & a_{32} & a_{33}
\end{vmatrix}
=a_{11}
\begin{vmatrix}
a_{22} & a_{23}\\
a_{32} & a_{33}
\end{vmatrix}
-a_{12}
\begin{vmatrix}
a_{21} & a_{23}\\
a_{31} & a_{33}
\end{vmatrix}
+a_{13}
\begin{vmatrix}
a_{21} & a_{22}\\
a_{31} & a_{32}
\end{vmatrix}.
$$

---

## 6. 求解特征向量

对于每个特征值 $\lambda_i$，对应的特征向量 $\mathbf v_i$ 满足：

$$
(\mathbf C-\lambda_i\mathbf I)\mathbf v_i=\mathbf 0.
$$

### 6.1 为什么方程没有唯一解？

因为 $\lambda_i$ 是特征值，所以：

$$
\det(\mathbf C-\lambda_i\mathbf I)=0.
$$

这意味着 $\mathbf C-\lambda_i\mathbf I$ 是奇异矩阵。对于本例中互不相同的三个特征值，该矩阵的秩为 2、零空间维数为 1，因此展开得到的三个线性方程中只有两个是线性独立的，另一个是冗余约束。

齐次方程当然包含零解，但特征向量按定义不能是零向量。它的非零解只确定了一个**方向**，没有确定长度：

$$
(\mathbf C-\lambda_i\mathbf I)\mathbf v_i=\mathbf0
\quad\Longrightarrow\quad
(\mathbf C-\lambda_i\mathbf I)(k\mathbf v_i)=\mathbf0,
\qquad k\neq0.
$$

因此，如果 $(x,y,z)^T$ 是一个特征向量，那么 $(2x,2y,2z)^T$、$(-x,-y,-z)^T$ 等也都表示同一个特征方向。手工计算时可以固定一个**确认不为零的分量**，把它当作自由变量的尺度，再求另外两个分量。

> [!IMPORTANT]
> “令 $z=1$”不是 PCA 的规定，也不是三个坐标都可以任意指定。这里只能任选一个非零分量确定尺度，其余分量仍必须由方程组解出。如果某个特征方向恰好满足 $z=0$，就不能令 $z=1$，应改为固定 $x$ 或 $y$。数值计算中通常优先固定绝对值较大、远离零的分量，以降低除以很小数造成的误差。

本例的最小特征值对应近似竖直的平面法向量，$z$ 分量最大，因此令 $z=1$ 最方便；后两个特征向量则分别选择 $y=1$ 和 $x=1$。得到任意一个非零解后，再除以它的二范数即可获得单位特征向量。

### 6.2 最小特征值 $\lambda_1\approx0.0808221$

代入 $\lambda_1$：

$$
\mathbf C-\lambda_1\mathbf I\approx
\begin{bmatrix}
1.6066779 & -0.175 & -0.25\\
-0.175 & 1.1491779 & -0.05\\
-0.25 & -0.05 & 0.0441779
\end{bmatrix}.
$$

令 $\mathbf v_1=(x,y,z)^T$，矩阵方程展开为：

$$
\begin{cases}
1.6066779x-0.175y-0.25z=0,\\
-0.175x+1.1491779y-0.05z=0,\\
-0.25x-0.05y+0.0441779z=0.
\end{cases}
$$

在精确特征值下，这三个方程只有两个线性独立。由于该特征方向的 $z$ 分量不为零且相对较大，令 $z=1$，前两个方程变为：

$$
\begin{cases}
1.6066779x-0.175y=0.25,\\
-0.175x+1.1491779y=0.05.
\end{cases}
$$

解这个二元一次方程组：

$$
x\approx0.163044,
\qquad
y\approx0.068338.
$$

代回第三个方程验证：

$$
-0.25(0.163044)-0.05(0.068338)+0.0441779
\approx0.
$$

因此可取一个未归一化的非零解：

$$
\tilde{\mathbf v}_1\approx
\begin{bmatrix}
0.163044\\
0.068338\\
1
\end{bmatrix},
\qquad
\lVert\tilde{\mathbf v}_1\rVert_2\approx1.015506.
$$

归一化后：

$$
\boxed{
\mathbf v_1\approx
\begin{bmatrix}
0.160554\\
0.067295\\
0.984730
\end{bmatrix}}
$$

### 6.3 中间特征值 $\lambda_2\approx1.1849242$

代入 $\lambda_2$：

$$
\mathbf C-\lambda_2\mathbf I\approx
\begin{bmatrix}
0.5025758 & -0.175 & -0.25\\
-0.175 & 0.0450758 & -0.05\\
-0.25 & -0.05 & -1.0599242
\end{bmatrix}.
$$

令 $\mathbf v_2=(x,y,z)^T$，并选择 $y=1$ 作为尺度，方程组变为：

$$
\begin{cases}
0.5025758x-0.25z=0.175,\\
-0.175x-0.05z=-0.0450758,\\
-0.25x-1.0599242z=0.05.
\end{cases}
$$

其中只有两个方程线性独立。求解可得：

$$
x\approx0.290640,
\qquad
z\approx-0.115725.
$$

因此可取：

$$
\tilde{\mathbf v}_2\approx
\begin{bmatrix}
0.290640\\
1\\
-0.115725
\end{bmatrix},
\qquad
\lVert\tilde{\mathbf v}_2\rVert_2\approx1.047790.
$$

归一化后：

$$
\boxed{
\mathbf v_2\approx
\begin{bmatrix}
0.277384\\
0.954390\\
-0.110447
\end{bmatrix}}
$$

### 6.4 最大特征值 $\lambda_3\approx1.7767537$

代入 $\lambda_3$：

$$
\mathbf C-\lambda_3\mathbf I\approx
\begin{bmatrix}
-0.0892537 & -0.175 & -0.25\\
-0.175 & -0.5467537 & -0.05\\
-0.25 & -0.05 & -1.6517537
\end{bmatrix}.
$$

令 $\mathbf v_3=(x,y,z)^T$，并选择 $x=1$ 作为尺度，方程组变为：

$$
\begin{cases}
-0.175y-0.25z=0.0892537,\\
-0.5467537y-0.05z=0.175,\\
-0.05y-1.6517537z=0.25.
\end{cases}
$$

其中同样只有两个方程线性独立。求解可得：

$$
y\approx-0.307080,
\qquad
z\approx-0.142059.
$$

因此可取：

$$
\tilde{\mathbf v}_3\approx
\begin{bmatrix}
1\\
-0.307080\\
-0.142059
\end{bmatrix},
\qquad
\lVert\tilde{\mathbf v}_3\rVert_2\approx1.055689.
$$

归一化后：

$$
\boxed{
\mathbf v_3\approx
\begin{bmatrix}
0.947249\\
-0.290881\\
-0.134565
\end{bmatrix}}
$$

> [!NOTE]
> 特征向量的正负号不唯一。如果 $\mathbf v$ 是特征向量，那么 $-\mathbf v$ 也是同一个特征值对应的特征向量。因此，数值库输出的符号与本文不同并不代表计算错误。

三个单位特征向量两两正交，并分别表示点云沿三个主方向的离散程度。对应的特征值越大，点云沿该特征向量方向的分布越分散。

---

## 7. 获得拟合平面

对于局部平面点云，**最小特征值对应的特征向量**表示点云离散程度最小的方向，因此可作为拟合平面的单位法向量：

$$
\boxed{
\mathbf n=\mathbf v_1\approx
\begin{bmatrix}
0.160554\\
0.067295\\
0.984730
\end{bmatrix}}
$$

工程实现中通常会统一法向量方向。例如，若期望法向量朝上，可以在 $n_z<0$ 时执行：

$$
\mathbf n\leftarrow-\mathbf n.
$$

拟合平面经过质心 $\mathbf c$，因此其点法式方程为：

$$
\mathbf n^T(\mathbf p-\mathbf c)=0.
$$

代入数值：

$$
0.160554(x-0.25)
+0.067295(y-0.10)
+0.984730(z-0.50)=0.
$$

展开后得到：

$$
\boxed{
0.160554x+0.067295y+0.984730z-0.539233=0
}.
$$

该法向量与世界坐标系 $z$ 轴的夹角约为：

$$
\theta=\arccos(|n_z|)\approx\boxed{10.03^\circ}.
$$

因此，这组点拟合出的平面整体接近水平面，但存在约 $10^\circ$ 的倾斜。