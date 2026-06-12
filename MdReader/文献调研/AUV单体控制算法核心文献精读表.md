# AUV 单体控制算法核心文献精读表

> 调研范围：聚焦 AUV / UUV 单体运动控制，包括深度/航向控制、路径跟随、轨迹跟踪、抗扰控制和约束控制。  
> 不作为主线：多 AUV 编队、协同控制、路径规划、SLAM、水下通信、纯数据驱动控制。  
> 本表用于后续继续扩展为完整综述报告、PPT 或技术路线论证。

---

## 1. 文献筛选说明

### 1.1 筛选原则

本轮精读优先纳入以下类型文献：

1. 直接面向 AUV / UUV 单体运动控制；
2. 控制任务明确，包括路径跟随、轨迹跟踪、深度/航向控制；
3. 方法属于经典控制、鲁棒控制、非线性控制、MPC/NMPC、扰动观测器、ADRC 等；
4. 对模型不确定性、外部扰动、输入饱和、状态约束等问题有明确处理；
5. 近年高相关论文优先，同时保留少量经典建模与综述文献。

### 1.2 分类标签

| 标签 | 含义 |
|---|---|
| `Model` | AUV 动力学建模与控制问题基础 |
| `Review` | 综述类文献 |
| `PID/LQR` | 经典控制 |
| `SMC` | 滑模控制及其变体 |
| `Backstepping` | 反步控制及其复合方法 |
| `Adaptive` | 自适应控制 |
| `DOB/ESO/ADRC` | 扰动观测器、扩张状态观测器、自抗扰控制 |
| `MPC/NMPC` | 模型预测控制、非线性模型预测控制 |
| `Hybrid` | 复合控制方法 |
| `Intelligent` | 智能控制补充方向 |

---

## 2. 核心文献总览表

| 序号 | 文献 | 年份 | 类型 | 控制任务 | 核心方法 | 验证方式 | 与本项目相关性 |
|---:|---|---:|---|---|---|---|---|
| 1 | Fossen marine craft model / Handbook of Marine Craft Hydrodynamics and Motion Control | 1994/2011/2021 | Model | 6-DOF 海洋航行器建模 | 矩阵-向量 6-DOF 非线性动力学 | 理论框架 | ★★★★★ |
| 2 | AUV Trajectory Tracking Models and Control Strategies | 2021 | Review | AUV 轨迹跟踪 | 建模与控制策略综述 | 综述 + case study | ★★★★★ |
| 3 | A Review of Path Following, Trajectory Tracking, and Formation Control for AUVs | 2025 | Review | 路径跟随、轨迹跟踪 | 传统控制与智能控制综述 | 综述 | ★★★★☆ |
| 4 | Trajectory tracking control of AUVs: a PRISMA-guided review | 2025/2026 | Review | AUV 轨迹跟踪 | PID/SMC/智能/混合分类 | 综述 | ★★★★☆ |
| 5 | Adaptive Tracking Control of an Autonomous Underwater Vehicle | 2014 | Adaptive | AUV 轨迹跟踪 | 基于回归矩阵的自适应控制 | 仿真 | ★★★★☆ |
| 6 | Trajectory tracking for AUV: An adaptive approach | 2019 | Adaptive/DOB | AUV 轨迹跟踪 | 自适应方法 | 仿真/实验相关 | ★★★★☆ |
| 7 | Adaptive disturbance observer for trajectory tracking control of underwater vehicles | 2020 | DOB/Adaptive | 水下航行器轨迹跟踪 | 自适应扰动观测器 | 仿真/实验相关 | ★★★★☆ |
| 8 | Double-loop integral terminal sliding mode tracking control for UUVs | 2019 | SMC | UUV 轨迹跟踪 | 双环积分终端滑模 + 自适应动态补偿 | 仿真 | ★★★★☆ |
| 9 | Robust Trajectory Tracking Control for Underactuated AUVs in Uncertain Environments | 2020/2021 | NMPC/Robust | 欠驱动 AUV 3D 轨迹跟踪 | 鲁棒 NMPC + tube 约束思想 | 仿真 | ★★★★★ |
| 10 | MPC-based 3-D trajectory tracking for an AUV with constraints in complex ocean environments | 2019 | MPC | 3D 轨迹跟踪 | MPC + 约束处理 | 仿真 | ★★★★★ |
| 11 | Lyapunov-based MPC trajectory tracking for an AUV with external disturbances | 2017/2021 | MPC | 轨迹跟踪 | Lyapunov-based MPC | 仿真 | ★★★★☆ |
| 12 | Modeling and trajectory tracking MPC novel method of AUV based on CFD data | 2022 | MPC/SMC | 轨迹跟踪 | CFD 建模 + MPC + 动态 SMC | 仿真 | ★★★★☆ |
| 13 | 3D Trajectory Tracking of AUV Based on NTSM and ADRDC | 2023 | SMC/ADRC | 3D 轨迹跟踪 | 非奇异终端滑模 + 自抗扰解耦控制 | 仿真 | ★★★★★ |
| 14 | State-Transform MPC-SMC-Based Trajectory Tracking Control of Cross-Rudder AUV | 2024 | MPC/SMC/Hybrid | 交叉舵 AUV 搜索任务轨迹跟踪 | 状态变换 + MPC + SMC 双环框架 | 实艇/仿真 | ★★★★★ |
| 15 | GP-MPC for Trajectory Tracking and Obstacle Avoidance in AUVs | 2024 | MPC/GP | 轨迹跟踪与避障 | 高斯过程 + MPC | 仿真 | ★★★☆☆ |
| 16 | Tube-based MPC of an AUV using LOS re-planning | 2024 | MPC/Robust | 3D 轨迹跟踪 | LOS 重规划 + Tube-based MPC | 仿真 | ★★★★★ |
| 17 | Finite-Time Path-Following Control of Underactuated AUVs | 2025 | Backstepping/Adaptive | 欠驱动 AUV 3D 路径跟随 | 鲁棒自适应有限时间控制 | 仿真 | ★★★★☆ |
| 18 | Three-Dimensional Path Following Control for Underactuated AUVs Based on Current Observer and NDO | 2024 | Backstepping/SMC/DOB | 3D 路径跟随 | LOS + Backstepping + NDO + SMC | 仿真 | ★★★★★ |
| 19 | FLOS + FSMC path-following control for AUVs considering multiple factors | 2024 | Fuzzy/SMC | 水平面路径跟随 | 模糊 LOS + 模糊 SMC | 仿真 + 实验 | ★★★★☆ |
| 20 | Trajectory Tracking Control for an Underactuated AUV via Nonsingular Fast Terminal Sliding Mode | 2024 | SMC | 欠驱动 AUV 水平面轨迹跟踪 | DVTER + 非奇异快速终端滑模 | 仿真 | ★★★★☆ |

---

## 3. 单篇文献精读卡片

### 1. Fossen 海洋航行器动力学模型

**文献类型：** 建模基础  
**核心关键词：** 6-DOF, marine craft model, matrix-vector form, hydrodynamics, motion control  
**适用位置：** 报告第 2 章“AUV 动力学模型与控制难点”

#### 研究内容

Fossen 模型将海洋航行器 6 自由度运动写成统一的矩阵-向量形式，常见表达为：


$$
M\dot{\nu}+C(\nu)\nu+D(\nu)\nu+g(\eta)=\tau
$$


其中：

- `η`：惯性坐标系下的位置和姿态；
- `ν`：艇体坐标系下的速度；
- `M`：惯性矩阵，通常包含刚体质量和附加质量；
- `C(ν)`：科氏和向心项；
- `D(ν)`：水动力阻尼；
- `g(η)`：重力和浮力恢复项；
- `τ`：推进器、舵面或控制输入产生的广义力。

#### 对本项目启发

AUV 单体控制算法不宜脱离动力学模型直接讨论。即便后续采用 PID、SMC、Backstepping 或 MPC，也应先统一使用该模型作为理论框架。对于项目初期，可先从水平面 3-DOF 或垂向面简化模型入手，再扩展到 6-DOF。

---

### 2. Li et al., 2021: AUV Trajectory Tracking Models and Control Strategies

**文献类型：** 综述  
**核心关键词：** AUV, trajectory tracking, control-oriented model, unknown input estimation  
**控制任务：** AUV 轨迹跟踪

#### 研究内容

该综述系统讨论了 AUV 轨迹跟踪中的建模方法、未知输入估计和控制策略。文献指出，控制导向模型、未知扰动估计和更可靠的控制策略仍然是 AUV 轨迹跟踪中的开放问题。

#### 优点

- 与本项目“单体轨迹跟踪控制”高度相关；
- 适合作为报告的基础综述文献；
- 可用于提炼 AUV 轨迹跟踪中的主要难点。

#### 局限

- 对 2022 年之后的 MPC、ADRC、学习增强控制覆盖不足；
- 对工程部署细节讨论有限。

#### 对本项目启发

该文可作为报告第 1 章和第 2 章的核心依据，用于说明 AUV 轨迹跟踪控制的典型问题：模型不确定、未知扰动、强非线性和控制约束。

---

### 3. He et al., 2025: A Review of Path Following, Trajectory Tracking, and Formation Control for AUVs

**文献类型：** 综述  
**核心关键词：** path following, trajectory tracking, formation control, AUV motion control  
**控制任务：** 路径跟随、轨迹跟踪、编队控制

#### 研究内容

该综述从路径跟随、轨迹跟踪和多 AUV 编队三个方向总结 AUV 运动控制研究进展。对于本项目，只取其中的单体路径跟随和轨迹跟踪部分。

#### 优点

- 资料较新；
- 便于梳理近年 AUV 控制发展趋势；
- 明确指出环境扰动、非线性、模型不确定性和物理约束是 AUV 控制的核心难点。

#### 局限

- 覆盖多 AUV 编队，本项目需筛除；
- 综述范围广，单篇方法细节不够深入。

#### 对本项目启发

适合作为“研究现状”和“发展趋势”部分的主要参考文献。

---

### 4. Eissa et al., 2025/2026: AUV trajectory tracking control review

**文献类型：** 综述  
**核心关键词：** trajectory tracking, PRISMA, classical control, intelligent control, hybrid control  
**控制任务：** AUV 轨迹跟踪

#### 研究内容

该综述采用 PRISMA-guided survey 思路，将 AUV 轨迹跟踪控制器划分为经典控制、智能控制和混合控制三大类。其中指出，PID 和 SMC 等经典控制方法由于结构清晰、计算需求低和稳定性分析成熟，仍然常用，但在强未建模扰动下性能会下降。

#### 对本项目启发

该文适合用于建立算法分类框架。由于本项目不以数据驱动为主线，因此可以采用该文的分类方式，但把智能控制部分降级为补充趋势。

---

### 5. Sahu & Subudhi, 2014: Adaptive Tracking Control of an AUV

**文献类型：** 自适应控制  
**核心关键词：** adaptive control, hydrodynamic uncertainty, Lyapunov stability  
**控制任务：** AUV 轨迹跟踪

#### 研究内容

该文针对水动力参数不确定条件下的 AUV 轨迹跟踪问题，设计了基于状态相关回归矩阵的自适应控制律，并通过 Lyapunov 方法证明稳定性。

#### 优点

- 针对水动力参数不确定性；
- 控制结构清晰；
- 适合作为自适应控制的基础代表文献。

#### 局限

- 对复杂外部扰动、输入饱和和状态约束考虑不足；
- 与新近 MPC/SMC 复合方法相比，工程鲁棒性论证较弱。

#### 对本项目启发

如果本项目需要处理水动力参数未知，可将自适应律作为附加模块，与 SMC 或 Backstepping 结合。

---

### 6. Guerrero et al., 2019: Trajectory tracking for AUV: An adaptive approach

**文献类型：** 自适应控制  
**核心关键词：** adaptive trajectory tracking, AUV, Ocean Engineering  
**控制任务：** AUV 轨迹跟踪

#### 研究内容

该文研究 AUV 轨迹跟踪中的自适应控制问题，重点面向模型参数不确定和跟踪误差收敛。

#### 优点

- 属于 Ocean Engineering 期刊文献，工程背景较强；
- 可用于补充自适应控制方法的发展脉络。

#### 局限

- 与 MPC 类方法相比，对输入/状态约束处理能力有限；
- 控制律设计依赖模型结构。

#### 对本项目启发

可作为“自适应控制”小节的代表文献，与 2014 年较基础的 adaptive tracking control 形成时间线。

---

### 7. Guerrero et al., 2020: Adaptive Disturbance Observer for Underwater Vehicles

**文献类型：** 扰动观测器 / 自适应控制  
**核心关键词：** adaptive disturbance observer, robust trajectory tracking, unknown disturbances  
**控制任务：** 水下航行器轨迹跟踪

#### 研究内容

该文针对未知扰动下的水下航行器轨迹跟踪问题，设计自适应扰动观测器，用于估计并补偿外部扰动。

#### 优点

- 直接面向未知扰动；
- 与项目中的抗扰控制需求高度相关；
- 可与传统控制器组合成“基准控制器 + 扰动补偿”的工程方案。

#### 局限

- 观测器带宽和噪声敏感性需要注意；
- 对复杂约束的处理能力不如 MPC。

#### 对本项目启发

推荐作为本项目的重点可借鉴方向之一。工程路线可以采用：

```text
PID/Backstepping/SMC 基础控制器 + DOB/ESO 扰动估计 + 补偿项
```

---

### 8. Qiao & Zhang, 2019: Double-loop Integral Terminal SMC for UUVs

**文献类型：** 滑模控制  
**核心关键词：** integral terminal sliding mode, double-loop, adaptive dynamic compensation  
**控制任务：** UUV 轨迹跟踪

#### 研究内容

该文提出双环积分终端滑模控制方法，并引入自适应动态补偿以处理不确定性和外部扰动。

#### 优点

- 收敛速度较快；
- 鲁棒性较强；
- 适合处理非线性和扰动问题。

#### 局限

- 滑模类方法容易出现抖振；
- 参数整定对性能影响较大；
- 工程部署需考虑执行器频繁动作和能耗。

#### 对本项目启发

可作为 SMC 类方法的代表文献。若本项目注重抗扰性，可考虑采用“高阶滑模 / 终端滑模 + 扰动观测器”的组合。

---

### 9. Heshmati-Alamdari et al., 2020/2021: Robust NMPC for Underactuated AUVs

**文献类型：** 鲁棒 NMPC  
**核心关键词：** underactuated AUV, robust NMPC, constraints, obstacle avoidance, tube  
**控制任务：** 欠驱动 AUV 三维轨迹跟踪

#### 研究内容

该文针对欠驱动 AUV 在不确定环境中的三维轨迹跟踪问题，提出鲁棒 NMPC 方法，并考虑工作空间约束、障碍物、外部扰动和模型不确定性。

#### 优点

- 适合处理输入和状态约束；
- 可考虑障碍物和动态环境；
- 与欠驱动 AUV 高度相关。

#### 局限

- 在线优化计算量较大；
- 对模型精度和求解器实时性要求高；
- 工程实现难度高于 PID/SMC/Backstepping。

#### 对本项目启发

如果项目目标偏向“高性能约束控制”，该文很有参考价值；如果项目目标偏向快速落地，则可先采用 SMC/Backstepping，再将 MPC 作为提升路线。

---

### 10. Zhang et al., 2019: MPC-based 3D Trajectory Tracking for an AUV

**文献类型：** MPC  
**核心关键词：** 3D trajectory tracking, constraints, complex ocean environments  
**控制任务：** AUV 三维轨迹跟踪

#### 研究内容

该文提出基于 MPC 的三维轨迹跟踪方法，将轨迹跟踪控制转化为带约束优化问题。其核心价值在于显式处理复杂海洋环境下的控制约束。

#### 优点

- 能处理输入约束和状态约束；
- 适合轨迹跟踪任务；
- 方法框架清晰，适合作为 MPC 小节代表文献。

#### 局限

- 计算复杂度较高；
- 对模型和预测精度依赖较强。

#### 对本项目启发

若项目涉及推进器饱和、航速限制或安全边界，MPC 是重要备选路线。

---

### 11. Shen et al., 2017 / Gong et al., 2021: Lyapunov-based MPC for AUV

**文献类型：** Lyapunov-based MPC  
**核心关键词：** Lyapunov constraint, MPC, external disturbances  
**控制任务：** AUV 轨迹跟踪

#### 研究内容

Lyapunov-based MPC 通过在预测控制框架中引入稳定性约束，提升闭环稳定性保证。该类方法适合解决常规 MPC 在鲁棒稳定性上的不足。

#### 优点

- 兼顾 MPC 约束处理能力和 Lyapunov 稳定性分析；
- 理论性较强；
- 适合用于项目方案论证。

#### 局限

- 设计复杂；
- 在线求解负担较大；
- 对候选 Lyapunov 函数构造要求高。

#### 对本项目启发

可作为“优化控制方法”部分的重要理论路线，但不建议作为项目初始版本控制器。

---

### 12. Bao & Zhu, 2022: CFD Data-based AUV Modeling and MPC-SMC

**文献类型：** MPC + SMC  
**核心关键词：** CFD data, model predictive control, dynamic sliding mode control  
**控制任务：** AUV 轨迹跟踪

#### 研究内容

该文利用 CFD 数据建立 AUV 控制模型，并结合 MPC 与动态滑模控制实现轨迹跟踪。

#### 优点

- 尝试将 CFD 水动力建模与控制结合；
- MPC 负责优化，SMC 增强抗扰；
- 是“模型改进 + 控制算法”的代表路线。

#### 局限

- CFD 建模成本高；
- 数据质量直接影响模型可靠性；
- 工程复现门槛较高。

#### 对本项目启发

如果项目中有水动力仿真或辨识条件，可借鉴其“高保真模型 + 控制器”的思路；否则可仅作为进阶参考。

---

### 13. Zhang et al., 2023: NTSM-ADRDC for 3D AUV Trajectory Tracking

**文献类型：** 非奇异终端滑模 + 自抗扰解耦控制  
**核心关键词：** NTSM, ADRDC, 3D trajectory tracking, decoupling  
**控制任务：** AUV 三维轨迹跟踪

#### 研究内容

该文提出 NTSM-ADRDC 控制框架。首先通过 ADRDC 将 AUV 模型解耦为多个单输入单输出通道，然后设计非奇异终端滑模控制器实现三维轨迹跟踪。

#### 优点

- 适合处理 AUV 强耦合问题；
- ADRC/ADRDC 对未知扰动具有补偿能力；
- NTSM 提高收敛速度并避免传统终端滑模奇异问题。

#### 局限

- 多通道解耦效果依赖模型和观测器设计；
- 控制器参数较多；
- 仿真有效不等于实艇部署稳定。

#### 对本项目启发

这是本项目高度值得精读的文献，尤其适合抗扰轨迹跟踪方向。可作为报告中的重点案例。

---

### 14. Hong et al., 2024: State-Transform MPC-SMC for Cross-Rudder AUV

**文献类型：** MPC + SMC 复合控制  
**核心关键词：** cross-rudder AUV, MPC-SMC, dual-loop framework, ocean disturbances  
**控制任务：** 交叉舵 AUV 搜索任务轨迹跟踪

#### 研究内容

该文提出双环鲁棒轨迹跟踪框架，通过状态变换、MPC 和 SMC 组合提高交叉舵 AUV 在海洋扰动下的轨迹跟踪能力。文中还包含真实 AUV 实验用于验证交叉舵构型在下潜阶段保持航行角稳定的能力。

#### 优点

- 复合控制框架较接近工程应用；
- 兼顾 MPC 的优化能力和 SMC 的鲁棒性；
- 包含真实 AUV 实验背景。

#### 局限

- 针对交叉舵 AUV，泛化到其他构型需重新建模；
- 控制框架复杂；
- 参数和状态变换设计需要深入理解。

#### 对本项目启发

如果项目 AUV 具有舵面或交叉舵结构，该文优先级很高。即使构型不同，也可借鉴“外环轨迹优化 + 内环鲁棒控制”的框架。

---

### 15. Liu et al., 2024: GP-MPC for AUV Trajectory Tracking and Obstacle Avoidance

**文献类型：** 高斯过程 + MPC  
**核心关键词：** Gaussian Process, MPC, obstacle avoidance, model uncertainty  
**控制任务：** 轨迹跟踪与避障

#### 研究内容

该文提出 GP-MPC 方法，用高斯过程处理模型不确定性，并在 MPC 框架中实现轨迹跟踪与避障。

#### 优点

- 能处理模型不确定性；
- 与 MPC 结合后可处理约束；
- 适合复杂动态环境。

#### 局限

- 属于学习增强 MPC，不是本项目主线；
- 计算量和训练数据要求较高；
- 工程部署复杂度较高。

#### 对本项目启发

可作为智能控制补充方向。不建议作为第一阶段主线，但可用于说明“模型驱动 + 学习补偿”是未来趋势。

---

### 16. Jimoh et al., 2024: Tube-based MPC with LOS Re-planning

**文献类型：** Tube-based MPC  
**核心关键词：** tube-based MPC, LOS re-planning, input saturation, unknown disturbances, energy metric  
**控制任务：** AUV 三维轨迹跟踪

#### 研究内容

该文研究输入饱和和未知环境扰动下的 AUV 离散时间三维轨迹跟踪问题。方法包括：

1. 采用 LOS 策略进行局部重规划；
2. 使用 Tube-based MPC 提高对环境扰动的鲁棒性；
3. 通过名义模型 MPC 和状态反馈控制构造扰动管；
4. 引入能耗评价指标。

#### 优点

- 同时考虑输入饱和、未知扰动和能耗；
- 适合复杂海况下的鲁棒轨迹跟踪；
- 对比 NMPC 展示其有效性。

#### 局限

- 控制设计和求解复杂；
- 更适合作为高性能控制路线，而非快速工程基线。

#### 对本项目启发

这是 MPC 类文献中最值得精读的一篇，适合支撑“约束 + 抗扰 + 能耗”方向的技术路线。

---

### 17. Ebrahimpour et al., 2025: Finite-Time Path-Following Control of Underactuated AUVs

**文献类型：** 鲁棒自适应有限时间控制  
**核心关键词：** underactuated AUV, finite-time control, path following, actuator saturation  
**控制任务：** 欠驱动 AUV 三维路径跟随

#### 研究内容

该文提出三维鲁棒自适应有限时间路径跟随控制器，处理模型不确定、外部扰动、执行器幅值和速率饱和等问题。

#### 优点

- 任务与本项目高度相关；
- 有限时间收敛有利于提高响应速度；
- 同时考虑扰动和执行器饱和。

#### 局限

- 控制律复杂；
- 有限时间控制参数选择较敏感；
- 以仿真为主，工程部署需进一步验证。

#### 对本项目启发

可作为“欠驱动 + 路径跟随 + 输入饱和”的重点参考文献。

---

### 18. 2024: Current Observer + NDO + Backstepping SMC for 3D Path Following

**文献类型：** Backstepping + SMC + 扰动观测器  
**核心关键词：** current observer, nonlinear disturbance observer, LOS, backstepping, integral SMC  
**控制任务：** 欠驱动 AUV 三维路径跟随

#### 研究内容

该文针对未知海流和不确定扰动下的欠驱动 AUV 三维路径跟随问题，设计了：

1. 海流观测器；
2. 非线性扰动观测器；
3. LOS + Backstepping 运动学控制器；
4. 积分滑模动力学控制器。

#### 优点

- 控制框架完整；
- 抗海流扰动能力强；
- 非常适合 AUV 单体路径跟随问题；
- 理论上给出闭环稳定性分析。

#### 局限

- 结构复杂；
- 参数整定工作量较大；
- 需要观测器估计质量较好。

#### 对本项目启发

该文非常适合作为本项目的主线参考。可将其简化为：

```text
LOS guidance → Backstepping kinematic control → SMC/DOB dynamic control
```

---

### 19. 2024: FLOS + FSMC Path-Following Control for AUVs

**文献类型：** 模糊 LOS + 模糊滑模控制  
**核心关键词：** FLOS, FSMC, path following, ocean current, experiments  
**控制任务：** 水平面路径跟随

#### 研究内容

该文提出基于模糊控制器的 LOS 制导方法和模糊滑模控制器，用于改善洋流扰动下 AUV 路径跟随性能。文献包含仿真和实验验证。

#### 优点

- 关注实际路径跟随任务；
- 有实验验证；
- 模糊机制用于在线调节参数，可减轻固定参数控制器的不足。

#### 局限

- 模糊规则设计依赖经验；
- 理论可解释性弱于标准 SMC/Backstepping/MPC；
- 泛化能力取决于规则库设计质量。

#### 对本项目启发

可作为“传统鲁棒控制 + 模糊调参”的补充路线。若项目不希望引入数据驱动，可将其放在智能控制补充部分。

---

### 20. Wang & Du, 2024: Nonsingular Fast Terminal Sliding Mode for Underactuated AUV

**文献类型：** 非奇异快速终端滑模控制  
**核心关键词：** underactuated AUV, DVTER, nonsingular fast terminal sliding mode  
**控制任务：** 水平面轨迹跟踪

#### 研究内容

该文研究欠驱动 AUV 水平面轨迹跟踪问题，构造期望速度—跟踪误差关系，避免直接对期望位置求导获得速度，并设计非奇异快速终端滑模控制器。

#### 优点

- 针对欠驱动 AUV；
- 终端滑模有利于快速收敛；
- 避免传统终端滑模奇异问题。

#### 局限

- 仍需注意滑模抖振；
- 主要聚焦水平面模型；
- 对复杂约束和能耗问题处理不足。

#### 对本项目启发

适合作为 SMC 类方法的近期代表文献，可与 NTSM-ADRDC、Tube-MPC 等方法对比。

---

## 4. 方法类别对比

| 方法类别 | 代表文献 | 优点 | 局限 | 项目建议 |
|---|---|---|---|---|
| PID/LQR | Fossen 建模框架及传统控制 | 简单、实时性好、易实现 | 抗扰和非线性处理能力有限 | 可作为基准控制器 |
| 自适应控制 | Sahu 2014; Guerrero 2019 | 可处理参数不确定 | 对突变扰动和约束处理较弱 | 可与 Backstepping/SMC 结合 |
| SMC/终端滑模 | Qiao 2019; Zhang 2023; Wang 2024 | 鲁棒性强、收敛快 | 抖振、参数整定敏感 | 重点关注 |
| Backstepping | Ebrahimpour 2025; Current Observer + NDO 2024 | 适合非线性和欠驱动系统 | 推导复杂，易出现虚拟控制项膨胀 | 重点关注 |
| DOB/ESO/ADRC | Guerrero 2020; Zhang 2023 | 抗扰能力强，工程意义明确 | 观测器参数和噪声敏感性需处理 | 重点关注 |
| MPC/NMPC | Zhang 2019; Heshmati 2020; Jimoh 2024 | 可显式处理约束和能耗 | 计算复杂、模型依赖强 | 作为高性能路线 |
| Hybrid | Hong 2024; Bao 2022 | 兼顾优化与鲁棒性 | 框架复杂 | 适合项目中后期 |
| Intelligent | GP-MPC 2024; DRL-SMC 2026 等 | 可处理复杂不确定性 | 部署难、稳定性证明弱 | 补充趋势，不作为主线 |

---

## 5. 针对本项目的推荐精读顺序

### 第一批：必须精读

1. Fossen marine craft model / Handbook  
2. Li et al., 2021, *AUV Trajectory Tracking Models and Control Strategies*  
3. Zhang et al., 2023, *NTSM-ADRDC for 3D AUV Trajectory Tracking*  
4. Jimoh et al., 2024, *Tube-based MPC with LOS Re-planning*  
5. Current Observer + NDO + Backstepping SMC for 3D Path Following, 2024  
6. Hong et al., 2024, *State-Transform MPC-SMC-Based Trajectory Tracking Control of Cross-Rudder AUV*

### 第二批：重点扩展

1. Heshmati-Alamdari et al., 2020/2021, Robust NMPC for Underactuated AUVs  
2. Zhang et al., 2019, MPC-based 3D trajectory tracking  
3. Guerrero et al., 2020, Adaptive disturbance observer  
4. Ebrahimpour et al., 2025, finite-time path following  
5. Wang & Du, 2024, Nonsingular fast terminal SMC

### 第三批：辅助理解

1. Sahu & Subudhi, 2014, Adaptive tracking control  
2. Guerrero et al., 2019, Adaptive trajectory tracking  
3. Bao & Zhu, 2022, CFD + MPC-SMC  
4. Liu et al., 2024, GP-MPC  
5. FLOS + FSMC, 2024

---

## 6. 推荐技术路线

结合本轮文献，若项目目标是 AUV 单体控制算法设计，建议优先采用以下路线之一。

### 路线 A：工程基准路线

```text
6-DOF/3-DOF 动力学模型
        ↓
PID/PD 深度与航向控制
        ↓
DOB/ESO 扰动估计
        ↓
抗扰补偿
```

**优点：** 快速实现，适合建立基准。  
**缺点：** 理论创新较弱，复杂轨迹跟踪能力有限。

---

### 路线 B：鲁棒非线性控制路线

```text
LOS 路径跟随制导
        ↓
Backstepping 运动学控制器
        ↓
SMC / Terminal SMC 动力学控制器
        ↓
NDO / ESO 扰动补偿
```

**优点：** 适合欠驱动、非线性、扰动环境，是当前最推荐路线。  
**缺点：** 控制律推导和参数整定较复杂。

---

### 路线 C：高性能约束控制路线

```text
AUV 预测模型
        ↓
MPC / NMPC
        ↓
输入饱和、状态约束、能耗约束
        ↓
Tube-based robust design
```

**优点：** 约束处理能力强，适合高性能轨迹跟踪。  
**缺点：** 在线计算量较大，对模型和求解器要求高。

---

### 路线 D：复合控制路线

```text
MPC 外环轨迹优化
        ↓
SMC / ADRC 内环鲁棒控制
        ↓
DOB / ESO 扰动估计
```

**优点：** 兼顾约束优化和抗扰鲁棒性。  
**缺点：** 系统复杂，适合中后期扩展。

---

## 7. 下一步建议

下一步建议从“文献表”进入“正文综述写作”，具体可以按如下顺序推进：

1. 写第 2 章：AUV 动力学模型与控制难点；
2. 写第 3 章：AUV 单体控制任务分类；
3. 写第 4 章：传统与鲁棒控制方法；
4. 写第 5 章：Backstepping、SMC、DOB/ADRC 复合控制；
5. 写第 6 章：MPC/NMPC 约束控制；
6. 写第 7 章：项目算法路线建议。

---

## 8. 参考文献与来源链接

> 注：以下为本轮检索使用的主要来源，后续可继续补充 DOI、BibTeX 和 GB/T 7714 格式。

1. Fossen marine craft model: https://fossen.biz/html/marineCraftModel.html  
2. Fossen Handbook repository: https://github.com/cybergalactic/FossenHandbook  
3. Li et al., 2021, *AUV Trajectory Tracking Models and Control Strategies*: https://www.mdpi.com/2077-1312/9/9/1020  
4. He et al., 2025, *A Review of Path Following, Trajectory Tracking, and Formation Control for AUVs*: https://www.mdpi.com/2504-446X/9/4/286  
5. Eissa et al., 2025/2026, trajectory tracking controller review: https://link.springer.com/article/10.1007/s40435-025-01963-5  
6. Sahu & Subudhi, 2014, adaptive tracking control: https://www.mi-research.net/article/doi/10.1007/s11633-014-0792-7  
7. Heshmati-Alamdari et al., robust NMPC for underactuated AUVs: https://vbn.aau.dk/en/publications/robust-trajectory-tracking-control-for-underactuated-autonomous-u/  
8. Jimoh et al., 2024, Tube-based MPC with LOS re-planning: https://strathprints.strath.ac.uk/91192/1/Jimoh-etal-OE-2024-Tube-based-model-predictive-control-of-an-autonomous-underwater-vehicle.pdf  
9. Liu et al., 2024, GP-MPC: https://www.mdpi.com/2077-1312/12/4/676  
10. Hong et al., 2024, State-transform MPC-SMC: https://www.mdpi.com/2077-1312/12/6/883  
11. Zhang et al., 2023, NTSM-ADRDC: https://www.mdpi.com/2077-1312/11/5/959  
12. Ebrahimpour et al., 2025, finite-time path following: https://www.mdpi.com/2504-446X/9/1/70  
13. Current observer + NDO + Backstepping SMC, 2024: https://www.mdpi.com/2504-446X/8/11/672  
14. FLOS + FSMC path following, 2024: https://www.mdpi.com/2077-1312/12/11/2045  
15. Wang & Du, 2024, Nonsingular fast terminal SMC: https://www.mdpi.com/2077-1312/12/8/1442  
16. Bao & Zhu, 2022, CFD data + MPC-SMC: https://pmc.ncbi.nlm.nih.gov/articles/PMC9185449/  
17. Tube-based MPC reference list and related AUV MPC papers: https://strathprints.strath.ac.uk/91192/1/Jimoh-etal-OE-2024-Tube-based-model-predictive-control-of-an-autonomous-underwater-vehicle.pdf  
