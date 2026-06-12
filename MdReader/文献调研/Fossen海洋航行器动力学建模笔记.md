# 《Handbook of Marine Craft Hydrodynamics and Motion Control》动力学建模部分整理

> 说明：本文档基于 Thor I. Fossen《Handbook of Marine Craft Hydrodynamics and Motion Control》中关于海洋航行器动力学建模的核心思想进行整理，并结合 Fossen 公开的 Marine Craft Model 页面和其 1995 年 6-DOF 非线性建模论文进行归纳。本文不是原书章节摘录，而是面向 **AUV 单体控制算法文献调研** 的建模笔记。

---

## 1. 建模对象与基本思想

Fossen 对海洋航行器建模的核心贡献，是将船舶、AUV、ROV、无人艇等海洋航行器的非线性运动方程统一写成紧凑的 **矩阵—向量形式**。这一表达形式比传统的逐项标量方程更适合控制器设计、稳定性分析和仿真实现。

对于 AUV 单体控制而言，该建模框架的主要价值在于：

1. 可以统一描述 6 自由度运动；
2. 可以显式区分刚体动力学、水动力附加质量、科氏/向心项、阻尼、恢复力和控制输入；
3. 模型结构具有对称性、斜对称性、正定性等性质，便于 Lyapunov 稳定性分析；
4. 可以根据控制任务降阶为水平面 3-DOF、垂向面模型、深度控制模型或航向控制模型。

---

## 2. 坐标系定义

AUV 动力学建模通常使用两个坐标系：

### 2.1 惯性坐标系 / 地固坐标系

常用 NED 坐标系：

- $x$：北向；
- $y$：东向；
- $z$：向下。

AUV 的位置和姿态通常在惯性坐标系中描述。

### 2.2 体坐标系

体坐标系固定在 AUV 本体上，随 AUV 一起运动：

- $x_b$：艏向；
- $y_b$：右舷方向；
- $z_b$：向下。

AUV 的线速度、角速度、控制力和控制力矩通常在体坐标系中表示。

---

## 3. 六自由度变量定义

AUV 的六自由度运动包括：

| 自由度 | 运动名称 | 力/力矩 | 速度 | 位置/姿态 |
|---|---|---|---|---|
| 1 | Surge 纵荡 | $X$ | $u$ | $x$ |
| 2 | Sway 横荡 | $Y$ | $v$ | $y$ |
| 3 | Heave 垂荡 | $Z$ | $w$ | $z$ |
| 4 | Roll 横滚 | $K$ | $p$ | $\phi$ |
| 5 | Pitch 俯仰 | $M$ | $q$ | $\theta$ |
| 6 | Yaw 偏航 | $N$ | $r$ | $\psi$ |

因此，广义位置、速度和力可以写成：

$$
\boldsymbol{\eta} = [x, y, z, \phi, \theta, \psi]^T
$$

$$
\boldsymbol{\nu} = [u, v, w, p, q, r]^T
$$

$$
\boldsymbol{\tau} = [X, Y, Z, K, M, N]^T
$$

其中：

- $\boldsymbol{\eta}$：惯性坐标系下的位置和姿态；
- $\boldsymbol{\nu}$：体坐标系下的线速度和角速度；
- $\boldsymbol{\tau}$：体坐标系下的广义力和广义力矩。

---

## 4. AUV 运动学模型

运动学模型用于描述体坐标系速度 $\boldsymbol{\nu}$ 与惯性坐标系位姿变化率 $\dot{\boldsymbol{\eta}}$ 之间的关系：

$$
\dot{\boldsymbol{\eta}} = \boldsymbol{J}(\boldsymbol{\eta}) \boldsymbol{\nu}
$$

其中，$\boldsymbol{J}(\boldsymbol{\eta})$ 是坐标变换矩阵，通常由位置变换矩阵和姿态变换矩阵组成：

$$
\boldsymbol{J}(\boldsymbol{\eta}) =
\begin{bmatrix}
\boldsymbol{R}_{b}^{n}(\phi,\theta,\psi) & \boldsymbol{0}_{3 \times 3} \\
\boldsymbol{0}_{3 \times 3} & \boldsymbol{T}(\phi,\theta)
\end{bmatrix}
$$

其中：

- $\boldsymbol{R}_{b}^{n}$：从体坐标系到惯性坐标系的旋转矩阵；
- $\boldsymbol{T}(\phi,\theta)$：欧拉角速度变换矩阵；
- $\phi,\theta,\psi$：横滚角、俯仰角和偏航角。

对于大姿态运动，欧拉角表达可能出现奇异性；在高机动 AUV 或姿态变化较大时，也可以使用四元数形式描述姿态。

---

## 5. AUV 动力学总模型

Fossen 框架下，海洋航行器 6-DOF 非线性动力学方程可写为：

$$
\boldsymbol{M}\dot{\boldsymbol{\nu}}
+ \boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu}
+ \boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu}
+ \boldsymbol{g}(\boldsymbol{\eta})
= \boldsymbol{\tau}
$$

与运动学方程组合后，完整模型为：

$$
\begin{cases}
\dot{\boldsymbol{\eta}} = \boldsymbol{J}(\boldsymbol{\eta})\boldsymbol{\nu} \\
\boldsymbol{M}\dot{\boldsymbol{\nu}}
+ \boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu}
+ \boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu}
+ \boldsymbol{g}(\boldsymbol{\eta})
= \boldsymbol{\tau}
\end{cases}
$$

各项物理意义如下：

| 项 | 名称 | 物理含义 |
|---|---|---|
| $\boldsymbol{M}\dot{\boldsymbol{\nu}}$ | 惯性项 | 刚体惯性与附加质量共同作用 |
| $\boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu}$ | 科氏与向心项 | 刚体运动和附加质量引起的速度耦合项 |
| $\boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu}$ | 阻尼项 | 水动力阻尼，包括线性阻尼和非线性阻尼 |
| $\boldsymbol{g}(\boldsymbol{\eta})$ | 恢复力项 | 重力和浮力产生的力与力矩 |
| $\boldsymbol{\tau}$ | 控制输入 | 推进器、舵面等产生的广义力和力矩 |

---

## 6. 惯性矩阵

惯性矩阵由刚体惯性矩阵和附加质量矩阵组成：

$$
\boldsymbol{M} = \boldsymbol{M}_{RB} + \boldsymbol{M}_{A}
$$

其中：

- $\boldsymbol{M}_{RB}$：刚体惯性矩阵；
- $\boldsymbol{M}_{A}$：附加质量矩阵。

### 6.1 刚体惯性矩阵

刚体惯性矩阵描述 AUV 本体质量和转动惯量对运动的影响。其一般形式与质量 $m$、质心位置 $\boldsymbol{r}_G=[x_G,y_G,z_G]^T$ 和转动惯量张量有关。

若体坐标系原点位于质心，且惯性积可以忽略，则 $\boldsymbol{M}_{RB}$ 可以简化为近似对角形式：

$$
\boldsymbol{M}_{RB} \approx
\operatorname{diag}(m,m,m,I_x,I_y,I_z)
$$

### 6.2 附加质量矩阵

AUV 在水中加速运动时，需要带动周围流体一起运动，因此表现为“有效惯性”增加，这一部分称为附加质量。

附加质量矩阵记为：

$$
\boldsymbol{M}_{A}
$$

工程中常用近似对角形式：

$$
\boldsymbol{M}_{A}
= -\operatorname{diag}(X_{\dot{u}},Y_{\dot{v}},Z_{\dot{w}},K_{\dot{p}},M_{\dot{q}},N_{\dot{r}})
$$

其中 $X_{\dot{u}},Y_{\dot{v}},Z_{\dot{w}},K_{\dot{p}},M_{\dot{q}},N_{\dot{r}}$ 是与加速度相关的水动力导数。

> 注意：不同文献中的符号正负约定可能不同。使用具体模型时，应保持与所引用文献或仿真工具一致。

---

## 7. 科氏力与向心力矩阵

科氏与向心项同样由刚体部分和附加质量部分组成：

$$
\boldsymbol{C}(\boldsymbol{\nu})
= \boldsymbol{C}_{RB}(\boldsymbol{\nu})
+ \boldsymbol{C}_{A}(\boldsymbol{\nu})
$$

其中：

- $\boldsymbol{C}_{RB}(\boldsymbol{\nu})$：刚体科氏/向心矩阵；
- $\boldsymbol{C}_{A}(\boldsymbol{\nu})$：附加质量引起的科氏/向心矩阵。

该项反映了不同自由度之间的速度耦合。例如：

- 偏航角速度 $r$ 会影响横向速度 $v$；
- 俯仰角速度 $q$ 会影响纵向和垂向运动；
- 高速运动时，交叉耦合项更加明显。

在控制设计中，$\boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu}$ 往往是非线性补偿项的重要组成部分。

Fossen 模型强调该矩阵具有斜对称相关性质，这对能量守恒和 Lyapunov 稳定性证明很重要。

---

## 8. 水动力阻尼模型

阻尼项用于描述水流对 AUV 运动的阻碍作用：

$$
\boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu}
$$

水动力阻尼通常包括：

1. 线性阻尼；
2. 二次非线性阻尼；
3. 交叉耦合阻尼。

常用简化形式为：

$$
\boldsymbol{D}(\boldsymbol{\nu})
= \boldsymbol{D}_{L} + \boldsymbol{D}_{Q}(\boldsymbol{\nu})
$$

其中：

- $\boldsymbol{D}_{L}$：线性阻尼矩阵；
- $\boldsymbol{D}_{Q}(\boldsymbol{\nu})$：与速度绝对值相关的二次阻尼矩阵。

工程中常见的对角阻尼形式为：

$$
\boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu}
= -
\begin{bmatrix}
X_u u + X_{|u|u}|u|u \\
Y_v v + Y_{|v|v}|v|v \\
Z_w w + Z_{|w|w}|w|w \\
K_p p + K_{|p|p}|p|p \\
M_q q + M_{|q|q}|q|q \\
N_r r + N_{|r|r}|r|r
\end{bmatrix}
$$

阻尼项是 AUV 模型中不确定性最强的部分之一。不同航速、雷诺数、外形、附体结构、舵面状态都会影响阻尼参数。

---

## 9. 恢复力与恢复力矩

恢复力项 $\boldsymbol{g}(\boldsymbol{\eta})$ 主要由重力和浮力产生：

$$
\boldsymbol{g}(\boldsymbol{\eta})
$$

其大小与以下因素有关：

- 重力 $W=mg$；
- 浮力 $B=\rho g \nabla$；
- 重心位置 $\boldsymbol{r}_G$；
- 浮心位置 $\boldsymbol{r}_B$；
- 横滚角、俯仰角等姿态变量。

对于水下航行器，重心和浮心不重合时，会产生恢复力矩。这对横滚和俯仰稳定性尤其重要。

一般来说：

- 若 $B \approx W$，AUV 近似中性浮力；
- 若浮心高于重心，可产生被动姿态稳定作用；
- 若重心和浮心偏置明显，需要在模型中保留恢复力矩。

---

## 10. 控制输入项

控制输入记为：

$$
\boldsymbol{\tau}
= [X,Y,Z,K,M,N]^T
$$

它代表推进器、舵面或其他执行机构产生的广义力和广义力矩。

对于不同 AUV，控制输入形式不同：

| AUV 类型 | 控制输入特点 |
|---|---|
| 全驱动 AUV | 6 个自由度均可直接或间接控制 |
| 欠驱动 AUV | 控制输入数量少于自由度数量 |
| 鱼雷型 AUV | 常用推进器控制纵向速度，舵面控制俯仰和偏航 |
| ROV 类平台 | 多推进器配置，通常接近全驱动 |

工程中还需要考虑：

1. 推进器饱和；
2. 推进器死区；
3. 舵角限制；
4. 执行器动态滞后；
5. 推力分配问题。

因此，更完整的模型可写为：

$$
\boldsymbol{\tau} = \boldsymbol{B}_u \boldsymbol{u}_c
$$

其中：

- $\boldsymbol{u}_c$：执行器控制量；
- $\boldsymbol{B}_u$：控制分配矩阵。

---

## 11. 总模型的物理分解

Fossen 动力学模型可以理解为以下力学平衡关系：

$$
\text{惯性力}
+ \text{科氏/向心力}
+ \text{水动力阻尼}
+ \text{重浮力恢复项}
= \text{推进与舵面控制输入}
$$

对应为：

$$
\boldsymbol{M}\dot{\boldsymbol{\nu}}
+ \boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu}
+ \boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu}
+ \boldsymbol{g}(\boldsymbol{\eta})
= \boldsymbol{\tau}
$$

对于 AUV 控制算法而言：

- $\boldsymbol{M}$：决定加速度响应；
- $\boldsymbol{C}$：体现速度耦合和非线性；
- $\boldsymbol{D}$：体现水动力耗散和主要不确定性；
- $\boldsymbol{g}$：影响姿态和深度稳定；
- $\boldsymbol{\tau}$：由控制器产生，是控制律最终作用对象。

---

## 12. AUV 控制中常用降阶模型

完整 6-DOF 模型适合高保真仿真，但控制器设计时常根据任务进行降阶。

### 12.1 水平面 3-DOF 模型

适用于航向控制、路径跟随和水平面轨迹跟踪：

$$
\boldsymbol{\eta}_h = [x,y,\psi]^T
$$

$$
\boldsymbol{\nu}_h = [u,v,r]^T
$$

动力学形式为：

$$
\boldsymbol{M}_h\dot{\boldsymbol{\nu}}_h
+ \boldsymbol{C}_h(\boldsymbol{\nu}_h)\boldsymbol{\nu}_h
+ \boldsymbol{D}_h(\boldsymbol{\nu}_h)\boldsymbol{\nu}_h
= \boldsymbol{\tau}_h
$$

该模型通常忽略横滚、俯仰和垂荡变化，适合近似定深航行。

### 12.2 垂向面模型

适用于深度控制和俯仰控制：

$$
\boldsymbol{\eta}_v = [x,z,\theta]^T
$$

$$
\boldsymbol{\nu}_v = [u,w,q]^T
$$

该模型关注纵向速度、垂向速度和俯仰角速度之间的耦合。

### 12.3 深度控制简化模型

如果只考虑深度 $z$ 和俯仰角 $\theta$，模型可以进一步简化，用于深度保持和深度跟踪控制。

### 12.4 航向控制简化模型

若只关注偏航角 $\psi$ 和偏航角速度 $r$，可得到航向控制模型：

$$
I_z \dot{r} + d_r r = N
$$

$$
\dot{\psi} = r
$$

其中：

- $I_z$：等效偏航转动惯量；
- $d_r$：偏航阻尼；
- $N$：偏航控制力矩。

---

## 13. 建模中的主要不确定性

AUV 动力学建模的主要难点不在于方程形式，而在于参数难以准确获得。

主要不确定性包括：

1. 附加质量参数不确定；
2. 线性和非线性阻尼参数不确定；
3. 舵面和推进器模型不准确；
4. 重心、浮心、载荷变化导致恢复力变化；
5. 洋流、波浪、湍流等环境扰动；
6. AUV 外形附体造成交叉耦合项难以建模；
7. 不同航速下水动力参数变化。

这也是 AUV 控制算法中大量使用以下方法的原因：

- 滑模控制；
- 自适应控制；
- 反步控制；
- 扰动观测器；
- 自抗扰控制；
- MPC/NMPC；
- 神经网络补偿控制。

---

## 14. 对 AUV 单体控制算法的启发

基于 Fossen 建模框架，可以得到以下控制设计思路。

### 14.1 PID 控制

PID 不依赖完整模型，但难以处理强耦合和外部扰动。适合深度、航向等单通道基础控制。

### 14.2 滑模控制

滑模控制可以把模型不确定性和外部扰动视为匹配扰动处理，适合基于如下形式设计：

$$
\boldsymbol{M}\dot{\boldsymbol{\nu}} =
\boldsymbol{\tau}
- \boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu}
- \boldsymbol{D}(\boldsymbol{\nu})\boldsymbol{\nu}
- \boldsymbol{g}(\boldsymbol{\eta})
$$

其中未准确建模的 $\boldsymbol{D}$、$\boldsymbol{C}$、外部流扰可以并入扰动项。

### 14.3 反步控制

反步控制适合非线性系统，常用于欠驱动 AUV 路径跟随和轨迹跟踪。Fossen 模型提供了从运动学误差到动力学控制输入逐步设计的基础。

### 14.4 自适应控制

如果 $\boldsymbol{M}$、$\boldsymbol{D}$ 或水动力导数未知，可以设计参数估计律在线更新模型参数。

### 14.5 MPC / NMPC

MPC 可直接利用状态空间形式处理输入饱和、速度约束、舵角约束和能耗目标。对于 AUV，常见做法是基于降阶模型构建预测模型，而不是直接使用完整 6-DOF 高复杂度模型。

### 14.6 扰动观测器 / ADRC

由于 AUV 阻尼和海流扰动难以精确建模，可将未知部分合并为总扰动：

$$
\boldsymbol{d}_{total}
= \Delta \boldsymbol{M}\dot{\boldsymbol{\nu}}
+ \Delta \boldsymbol{C}\boldsymbol{\nu}
+ \Delta \boldsymbol{D}\boldsymbol{\nu}
+ \boldsymbol{\tau}_{current}
$$

然后通过 DOB、ESO 或 ADRC 估计并补偿。

---

## 15. 可用于项目报告的简洁表述

如果在项目报告或文献综述中介绍 Fossen 动力学建模，可以使用如下表述：

> Fossen 提出的海洋航行器动力学建模框架将 AUV 的六自由度运动统一表示为矩阵—向量形式。该模型由运动学方程和动力学方程组成，其中运动学方程描述体坐标系速度到惯性坐标系位姿变化率的映射，动力学方程则由惯性项、科氏/向心项、水动力阻尼项、重浮力恢复项和控制输入项构成。该模型能够清晰反映 AUV 的强非线性、强耦合、附加质量、水动力阻尼和外部扰动等特征，是开展 AUV 轨迹跟踪、路径跟随、深度控制和姿态控制算法设计的重要理论基础。

---

## 16. AUV 单体控制建模建议

面向后续 AUV 单体控制算法调研和项目设计，建议采用如下建模层级：

| 建模层级 | 用途 | 建议 |
|---|---|---|
| 完整 6-DOF 模型 | 高保真仿真、总体建模说明 | 作为理论基础保留 |
| 水平面 3-DOF 模型 | 路径跟随、水平轨迹跟踪 | 作为主控模型之一 |
| 垂向面模型 | 深度控制、俯仰控制 | 作为深度控制模型 |
| 单通道简化模型 | PID/ADRC 基准控制 | 用于工程基线方案 |
| 扰动扩展模型 | DOB/ESO/ADRC | 用于抗扰控制设计 |
| 约束状态空间模型 | MPC/NMPC | 用于考虑输入饱和与能耗优化 |

---

## 17. 小结

Fossen 动力学建模部分对 AUV 单体控制的核心意义可以概括为：

1. 提供了统一的 6-DOF 非线性动力学表达；
2. 明确区分了刚体惯性、附加质量、科氏/向心力、阻尼、恢复力和控制输入；
3. 揭示了 AUV 控制中的主要困难：强耦合、强非线性、参数不确定和环境扰动；
4. 为滑模控制、反步控制、自适应控制、MPC、ADRC 等算法提供统一模型基础；
5. 支持根据任务进行模型降阶，从而服务于具体的深度控制、航向控制、路径跟随和轨迹跟踪控制。

---

## 参考来源

1. Thor I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, Wiley, 2nd Edition, 2021.
2. Fossen’s Marine Craft Model: https://fossen.biz/html/marineCraftModel.html
3. Thor I. Fossen and Ola-Erik Fjellstad, “Nonlinear Modelling of Marine Vehicles in 6 Degrees of Freedom,” *Journal of Mathematical Modelling of Systems*, 1995. https://www.fossen.biz/publications/1995%20Fossen%20and%20Fjellstad%20JMMS.pdf
4. Wiley Online Library, “Models for Underwater Vehicles,” Chapter 8 of *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd Edition, 2021.
