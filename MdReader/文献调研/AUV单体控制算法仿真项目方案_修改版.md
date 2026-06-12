# AUV 单体控制算法仿真项目方案（修改版）

> 项目定位：以 **C++ 为核心** 实现 AUV 单体控制算法与动力学仿真；第一版以 **水平面 3-DOF 欠驱动 AUV 路径跟随** 为主线，后续再扩展抗扰控制、Backstepping + SMC/NTSM、MPC/NMPC 和 Web/ROS 展示。
>
> 当前约束：暂时缺少具体 AUV 的几何参数、惯性参数和水动力参数，因此第一版不追求真实艇体高保真 CFD，而是建立一个 **参数化、可扩展、可对比的控制算法仿真平台**。

---

## 1. 修改后的总体判断

原方案的总体方向是正确的：

1. 以 Fossen 海洋航行器模型作为理论基础；
2. 以 C++ 实现动力学仿真和控制器核心；
3. 考虑参数不确定、外部海流扰动和执行器约束；
4. 通过轨迹、误差曲线、控制输入曲线进行展示。

但原方案存在一个主要问题：**第一版内容过多，容易导致实现周期过长、调试困难、各模块都不稳定。**

因此修改后的核心思路是：

```text
第一版先做稳：
水平面 3-DOF 欠驱动 AUV + LOS 路径跟随 + PID baseline

第二版再做强：
加入 ESO/ADRC 或 SMC，展示抗扰能力

第三版再做完整算法：
LOS + Backstepping + SMC/NTSM + DOB/ESO

第四版再做高级扩展：
MPC/NMPC、Monte Carlo、Web 3D、ROS/Gazebo
```

---

## 2. 修改后的项目目标

### 2.1 第一版目标

第一版建议明确为：

> 实现一个 **水平面 3-DOF 欠驱动 AUV 路径跟随仿真平台**。

核心功能包括：

1. C++17/20 + Eigen + CMake 工程框架；
2. 水平面 3-DOF AUV 动力学模型；
3. 欠驱动控制输入：纵向推力 $X$ 和偏航力矩 $N$；
4. LOS 路径跟随制导；
5. 纵向速度 PI/PID 控制；
6. 航向 PID baseline；
7. 恒定海流扰动；
8. 参数偏差：truth model / controller model；
9. CSV/JSON 数据输出；
10. 轨迹、横向误差、航向误差、控制输入曲线展示。

### 2.2 第一版暂不包含

第一版不建议纳入以下内容：

1. 完整 6-DOF 代码实现；
2. 垂向面深度控制；
3. 定点保持；
4. 严格时间参数化轨迹跟踪；
5. Backstepping + SMC/NTSM 完整主算法；
6. MPC/NMPC；
7. ROS/Gazebo；
8. 复杂 Web 3D 动画。

这些内容可以作为后续版本扩展。

---

## 3. 第一版推荐技术路线

第一版主线建议收敛为：

```text
水平面 3-DOF 欠驱动 AUV 模型
        ↓
恒定纵向速度控制
        ↓
LOS 路径跟随制导
        ↓
航向 PID baseline
        ↓
海流扰动与参数偏差
        ↓
仿真数据输出
        ↓
轨迹与误差曲线展示
```

第一版的目标不是展示最复杂算法，而是先把以下基础闭环跑通：

```text
参考路径 → LOS 期望航向 → 航向控制器 → AUV 动力学 → 实际轨迹 → 误差评价
```

---

## 4. 建模方案

## 4.1 理论保留：Fossen 6-DOF 模型

报告中保留完整 Fossen 6-DOF 模型作为理论基础：

$$
\dot{\eta} = J(\eta)\nu
$$

$$
M\dot{\nu} + C(\nu)\nu + D(\nu)\nu + g(\eta) = \tau
$$

其中：

- $\eta = [x, y, z, \phi, \theta, \psi]^T$：惯性坐标系下的位置和姿态；
- $\nu = [u, v, w, p, q, r]^T$：体坐标系下的线速度和角速度；
- $\tau = [X, Y, Z, K, M, N]^T$：体坐标系下的广义力和广义力矩；
- $M$：刚体惯性和附加质量矩阵；
- $C(\nu)$：科氏和向心矩阵；
- $D(\nu)$：水动力阻尼矩阵；
- $g(\eta)$：重力和浮力恢复项。

但第一版代码不建议直接实现 6-DOF，因为参数不完整、姿态变换复杂、恢复力项和科氏项容易出错。

---

## 4.2 第一版主控模型：水平面 3-DOF

水平面 3-DOF 适合第一版路径跟随任务，状态定义为：

$$
\eta_h = [x, y, \psi]^T
$$

$$
\nu_h = [u, v, r]^T
$$

其中：

- $x, y$：惯性坐标系下水平位置；
- $\psi$：偏航角；
- $u$：纵向速度；
- $v$：横向速度；
- $r$：偏航角速度。

运动学方程为：

$$
\dot{x} = u\cos\psi - v\sin\psi
$$

$$
\dot{y} = u\sin\psi + v\cos\psi
$$

$$
\dot{\psi} = r
$$

动力学方程为：

$$
M_h\dot{\nu}_h + C_h(\nu_h)\nu_h + D_h(\nu_h)\nu_h = \tau_h
$$

---

## 4.3 第一版采用欠驱动输入

建议第一版明确按鱼雷型欠驱动 AUV 处理：

$$
\tau_h = [X, 0, N]^T
$$

其中：

- $X$：纵向推力；
- $N$：偏航控制力矩；
- $Y = 0$：无直接横向控制力。

这样比全驱动模型更符合常见 AUV，也更能体现 LOS 路径跟随的意义。

---

## 4.4 海流扰动建模方式

海流不建议简单作为外力直接加到 $\tau$ 中。更合理的第一版方式是使用**相对速度**。

假设惯性坐标系中的恒定海流为：

$$
V_c^n = [V_{cx}, V_{cy}]^T
$$

将其转换到体坐标系：

$$
V_c^b = R_n^b(\psi) V_c^n
$$

相对速度为：

$$
\nu_r = [u, v, r]^T - [V_{cx}^b, V_{cy}^b, 0]^T
$$

阻尼项使用相对速度：

$$
D_h(\nu_r)\nu_r
$$

这样可以避免 AUV 转向后海流方向处理错误的问题。

---

## 5. 参数未知问题处理方案

由于当前缺少真实几何和水动力参数，建议采用 **nominal model / truth model** 的方式。

### 5.1 两套模型

```text
truth model:
  仿真环境真正使用的参数

controller model:
  控制器以为自己知道的参数
```

两者之间设置参数偏差，用于展示算法在参数不确定条件下的性能。

### 5.2 参数偏差设置

建议第一版设置以下偏差：

| 参数类型 | 建议偏差范围 |
|---|---:|
| 惯性参数 | ±20% |
| 阻尼参数 | ±30% |
| 附加质量参数 | ±20% |
| 推进器效率 | ±10% |

后续鲁棒性测试可以扩大到：

| 参数类型 | 扩展偏差范围 |
|---|---:|
| 惯性参数 | ±40% |
| 阻尼参数 | ±50% |
| 附加质量参数 | ±50% |
| 推进器效率 | ±30% |

注意：仿真不能直接表述为“证明鲁棒性”，更准确的说法是：

> 用于验证和展示控制器在给定参数不确定范围内的鲁棒性能。

---

## 6. 第一版控制算法设计

## 6.1 纵向速度控制

第一版可以先让 AUV 保持期望纵向速度 $u_d$：

$$
e_u = u_d - u
$$

纵向推力采用 PI/PID：

$$
X = K_{pu} e_u + K_{iu}\int e_u dt + K_{du}\dot{e}_u
$$

也可以第一版简化为恒定推力或 PI 控制。

---

## 6.2 LOS 路径跟随制导

LOS 制导用于把路径误差转换为期望航向角 $\psi_d$。

对于二维路径跟随，基本逻辑为：

```text
当前位置 + 当前路径段
        ↓
计算横向误差 e_y
        ↓
计算期望航向角 psi_d
        ↓
航向控制器跟踪 psi_d
```

第一版建议先实现：

1. 直线路径；
2. 圆形路径；
3. S 形路径。

折线路径可以后加，因为路径点切换容易引入抖动。

### 6.2.1 路径点切换建议

对于折线路径，可以使用切换半径：

```text
当 AUV 到当前路径段终点的距离 < switching_radius 时，切换到下一段。
```

同时需要对 $\psi_d$ 做角度连续化和限速，避免期望航向角突变。

---

## 6.3 航向 PID baseline

航向误差为：

$$
e_{\psi} = \operatorname{wrapToPi}(\psi_d - \psi)
$$

偏航力矩为：

$$
N = K_{p\psi} e_{\psi} + K_{i\psi}\int e_{\psi}dt + K_{d\psi}(r_d-r)
$$

第一版可以取 $r_d = 0$ 或根据 $\dot{\psi}_d$ 估计得到。

必须注意：

1. 所有角度统一使用弧度；
2. 航向误差必须 wrap 到 $[-\pi, \pi]$；
3. 控制输入需要加入饱和；
4. 积分项需要 anti-windup。

---

## 7. 第二版抗扰控制扩展

第二版建议在第一版稳定后加入抗扰控制。

### 7.1 航向 ADRC / ESO

先从单通道航向模型开始：

$$
I_z \dot{r} + d_r r = N + d
$$

$$
\dot{\psi} = r
$$

其中 $d$ 表示未知扰动。ESO 用于估计总扰动并进行补偿。

建议先完成：

1. 航向 PID；
2. 航向 ADRC；
3. PID vs ADRC 对比。

不要一开始就做 3-DOF 多通道 ESO。

### 7.2 航向 SMC

可以设计滑模面：

$$
s = \dot{e}_{\psi} + \lambda e_{\psi}
$$

切换项不建议直接使用 $\operatorname{sign}(s)$，建议使用：

$$
\operatorname{sat}\left(\frac{s}{\Phi}\right)
$$

或：

$$
\tanh\left(\frac{s}{\Phi}\right)
$$

以降低滑模抖振和执行器高频动作。

---

## 8. 第三版主算法扩展

第三版再实现文献主线算法：

```text
LOS guidance
        ↓
Backstepping kinematic controller
        ↓
SMC / NTSM dynamic controller
        ↓
DOB / ESO disturbance compensation
```

建议逐步实现：

1. 先实现 LOS + Backstepping；
2. 再加入 SMC；
3. 最后加入 DOB/ESO；
4. 再考虑 NTSM 替换普通 SMC。

不要一次性把 Backstepping、SMC、NTSM、DOB/ESO 全部叠加，否则调参困难。

---

## 9. 第四版高级扩展

高级扩展建议包括：

1. MPC/NMPC；
2. Tube-based MPC；
3. Monte Carlo 鲁棒性统计；
4. Web 3D 展示；
5. ROS 2 / Gazebo；
6. 垂向面深度控制；
7. 完整 6-DOF 模型。

MPC/NMPC 不建议放入第一版验收标准。第一版只保留接口即可：

```cpp
class MpcController : public Controller {
public:
    ControlInput compute(
        const State& state,
        const Reference& reference,
        double t
    ) override;
};
```

---

## 10. C++ 模块设计

建议代码结构如下：

```text
core/
  math/
    angle_utils.h
    rotation_2d.h
    vector_utils.h

  model/
    auv_3dof_model.h
    auv_3dof_model.cpp
    auv_params.h

  actuator/
    saturation.h
    first_order_lag.h
    deadzone.h

  environment/
    current_model.h
    disturbance_model.h
    noise_model.h

  guidance/
    los_guidance.h
    path_manager.h
    reference_path.h

  controller/
    pid_controller.h
    speed_pid.h
    heading_pid.h
    heading_adrc.h       // 第二版
    heading_smc.h        // 第二版
    backstepping_smc.h   // 第三版
    mpc_controller.h     // 第四版接口

  simulator/
    rk4_integrator.h
    simulator.h
    logger_csv.h
    logger_json.h

  evaluation/
    metrics.h
    cross_track_error.h
    energy_proxy.h
```

核心接口建议保持统一：

```cpp
struct State3DOF {
    double x;
    double y;
    double psi;
    double u;
    double v;
    double r;
};

struct ControlInput3DOF {
    double X;
    double N;
};

class Auv3DofModel {
public:
    State3DOF derivative(
        const State3DOF& state,
        const ControlInput3DOF& input,
        const Environment& env
    ) const;
};

class Controller {
public:
    virtual ControlInput3DOF compute(
        const State3DOF& state,
        const Reference& reference,
        double t
    ) = 0;
};
```

---

## 11. 仿真场景设计

## 11.1 第一版场景

| 场景 | 目的 | 控制器 |
|---|---|---|
| 直线路径跟随，无海流 | 验证 LOS + PID 基础闭环 | LOS + PID |
| 圆形路径跟随，无海流 | 验证曲线路径跟随 | LOS + PID |
| S 形路径跟随，无海流 | 验证连续机动路径 | LOS + PID |
| 直线路径 + 恒定横向海流 | 验证海流对路径偏差的影响 | LOS + PID |
| 圆形路径 + 参数偏差 | 验证模型不确定影响 | LOS + PID |

## 11.2 第二版场景

| 场景 | 目的 | 控制器 |
|---|---|---|
| 航向阶跃 + 恒定扰动 | PID 与 ADRC 对比 | PID / ADRC |
| 直线路径 + 横向海流 | 路径跟随抗扰对比 | PID / ADRC / SMC |
| S 形路径 + 海流 + 参数偏差 | 综合抗扰测试 | PID / ADRC / SMC |

## 11.3 第三版场景

| 场景 | 目的 | 控制器 |
|---|---|---|
| 欠驱动路径跟随 + 强海流 | 验证 Backstepping + SMC | LOS + Backstepping + SMC |
| 参数偏差 + 执行器饱和 | 验证鲁棒性与工程约束 | SMC / NTSM |
| 多组随机参数 | Monte Carlo 鲁棒性统计 | 主算法 |

---

## 12. 展示指标设计

第一版至少输出以下指标：

| 指标 | 含义 |
|---|---|
| RMS position error | 平均位置误差 |
| max position error | 最大位置误差 |
| cross-track error | 横向路径误差，路径跟随中最重要 |
| heading error | 航向误差 |
| control peak | 控制输入峰值 |
| control smoothness | 控制输入平滑程度 |
| energy proxy | 能耗近似指标 |

能耗近似指标可以定义为：

$$
E = \int_0^T \tau^T\tau \, dt
$$

离散仿真中：

$$
E \approx \sum_{k=0}^{N} \tau_k^T\tau_k \Delta t
$$

对于第一版欠驱动 3-DOF，可写成：

$$
E \approx \sum_{k=0}^{N} (X_k^2 + N_k^2)\Delta t
$$

---

## 13. 可视化方案

### 13.1 第一版推荐：Python 或静态 Web 2D 图

第一版不建议把复杂 3D Web 展示作为刚性目标。建议先输出 CSV/JSON，然后用以下方式快速画图：

1. Python + Matplotlib；
2. Web + Plotly.js；
3. 简单 HTML + JavaScript。

第一版展示内容：

1. 参考路径和实际路径；
2. 横向误差曲线；
3. 航向误差曲线；
4. 纵向速度曲线；
5. 控制输入 $X$ 和 $N$；
6. 海流方向箭头。

### 13.2 第二阶段再做 Web 3D

Web 3D 可以作为后续展示升级：

```text
C++ 仿真核心
        ↓
输出 CSV / JSON
        ↓
Web 前端
        ↓
Three.js 3D 动画 + Plotly 曲线图
```

---

## 14. 推荐项目目录结构

```text
AUV_proj/
  README.md
  CMakeLists.txt

  docs/
    project_plan_revised.md
    model_notes.md
    algorithm_notes.md

  config/
    auv_nominal.json
    auv_truth_case_01.json
    scenario_straight_path.json
    scenario_circle_path.json
    scenario_current_disturbance.json

  include/
    auv/
      math/
      model/
      controller/
      guidance/
      actuator/
      environment/
      simulator/
      evaluation/

  src/
    math/
    model/
    controller/
    guidance/
    actuator/
    environment/
    simulator/
    evaluation/

  apps/
    auv_sim/
      main.cpp

  tests/
    test_angle_utils.cpp
    test_rotation_2d.cpp
    test_pid.cpp
    test_integrator.cpp
    test_3dof_model.cpp
    test_los_guidance.cpp

  results/
    data/
    figures/

  scripts/
    plot_results.py
    compare_algorithms.py

  web/
    index.html
    src/
      main.js
      charts.js
      scene.js
```

---

## 15. 开发阶段规划

## 15.1 阶段 1：基础工程与单通道验证

目标：

1. 建立 CMake 项目；
2. 引入 Eigen；
3. 实现角度归一化、二维旋转矩阵；
4. 实现 RK4 积分器；
5. 实现 CSV 日志输出；
6. 实现航向单通道模型；
7. 实现航向 PID。

验收标准：

1. 航向阶跃响应能够收敛；
2. 航向误差正确 wrap 到 $[-\pi, \pi]$；
3. 控制输入有饱和和 anti-windup；
4. 输出 CSV 可画图。

---

## 15.2 阶段 2：水平面 3-DOF 模型

目标：

1. 实现水平面 3-DOF 动力学；
2. 实现欠驱动输入 $\tau_h=[X,0,N]^T$；
3. 加入纵向速度控制；
4. 加入恒定海流模型；
5. 正确处理相对速度。

验收标准：

1. 无控制输入时运动趋势合理；
2. 恒定推力下速度收敛到合理值；
3. 海流方向随坐标变换正确；
4. 轨迹不会出现明显数值发散。

---

## 15.3 阶段 3：LOS 路径跟随 baseline

目标：

1. 实现直线路径 LOS；
2. 实现圆形路径；
3. 实现 S 形路径；
4. 实现 LOS + 航向 PID；
5. 输出路径跟随误差。

验收标准：

1. 无海流情况下能够稳定跟随直线路径；
2. 圆形路径无明显发散；
3. S 形路径航向角连续；
4. 可以输出参考路径、实际路径、横向误差、航向误差。

---

## 15.4 阶段 4：扰动与参数偏差

目标：

1. 加入恒定横向海流；
2. 加入时变海流；
3. 加入 truth model / controller model 参数偏差；
4. 加入执行器饱和和一阶滞后。

验收标准：

1. PID 在海流扰动下出现可解释的路径偏差；
2. 参数偏差会导致跟踪性能下降；
3. 执行器饱和不会导致积分项无限积累。

---

## 15.5 阶段 5：抗扰控制扩展

目标：

1. 实现航向 ADRC/ESO；
2. 实现航向 SMC；
3. 对比 PID、ADRC、SMC；
4. 输出统一指标。

验收标准：

1. 同等扰动下 ADRC/SMC 的横向误差小于 PID；
2. 控制输入不过度高频振荡；
3. SMC 使用 sat/tanh 边界层，而不是直接 sign。

---

## 15.6 阶段 6：主算法扩展

目标：

1. 实现 LOS + Backstepping；
2. 实现 SMC/NTSM 动力学控制；
3. 加入 DOB/ESO 扰动补偿；
4. 加入 Monte Carlo 参数扰动测试。

验收标准：

1. 在海流和参数偏差下仍能稳定跟随路径；
2. 相比 PID，RMS 误差和横向误差明显降低；
3. 控制输入峰值和能耗指标可解释。

---

## 16. 实现中最容易遇到的问题与应对

| 问题 | 表现 | 应对 |
|---|---|---|
| 坐标系错误 | 轨迹方向反了、航向控制反了 | 单独测试旋转矩阵和坐标变换 |
| 角度 wrap 错误 | 航向误差从 $\pi$ 跳到 $-\pi$ | 实现并测试 `wrapToPi()` |
| 参数不合理 | AUV 像质点或直接发散 | 使用量级合理参数，逐步调试 |
| 海流处理错误 | 转向后海流方向不对 | 惯性系海流转换到体坐标系 |
| 欠驱动处理不清 | 控制器输出横向力 $Y$ | 第一版固定 $Y=0$ |
| LOS 切换抖动 | 路径点附近来回切换 | 使用 switching radius 和航向限速 |
| SMC 抖振 | 控制输入高频尖峰 | 用 sat/tanh 替代 sign |
| ESO 带宽难调 | 估计慢或控制发抖 | 先单通道，再多通道 |
| RK4 步长不合适 | 积分发散或数值噪声 | 先用 $dt=0.01\sim0.02s$ |
| 执行器饱和 | 控制器算得出，执行器做不到 | 加饱和、一阶滞后和 anti-windup |
| Web 展示坐标不一致 | 动画上下反、左右反 | 仿真坐标与显示坐标分开映射 |

---

## 17. 推荐第一版最小可行成果

第一版最小可行成果建议为：

```text
1. C++17/20 + Eigen + CMake 项目
2. 水平面 3-DOF 欠驱动 AUV 动力学模型
3. 欠驱动输入 tau = [X, 0, N]^T
4. 纵向速度 PI/PID 控制
5. LOS 路径跟随制导
6. 航向 PID baseline
7. 恒定海流扰动
8. 参数偏差 truth model / controller model
9. CSV/JSON 数据输出
10. Python 或 Web 轨迹与误差曲线展示
```

第一版验收图建议至少包括：

1. 参考路径 vs 实际路径；
2. 横向路径误差；
3. 航向误差；
4. 纵向速度；
5. 控制输入 $X$ 和 $N$；
6. 有无海流对比；
7. 有无参数偏差对比。

---

## 18. 推荐最终技术路线

综合项目目标、实现难度和展示效果，推荐最终路线为：

```text
第一阶段：基础闭环
水平面 3-DOF 欠驱动 AUV
+ LOS 路径跟随
+ PID baseline
+ 海流扰动
+ 参数偏差
+ 数据曲线展示

第二阶段：抗扰控制
+ 航向 ADRC/ESO
+ 航向 SMC
+ PID vs ADRC vs SMC 对比

第三阶段：文献主线算法
+ LOS + Backstepping
+ SMC/NTSM 动力学控制
+ DOB/ESO 扰动补偿
+ Monte Carlo 鲁棒性统计

第四阶段：高级扩展
+ MPC/NMPC
+ Web 3D
+ ROS 2/Gazebo
+ 垂向面深度控制
+ 完整 6-DOF
```

---

## 19. 待确认问题

正式开始写代码前，需要先确认以下问题：

1. 第一版是否确定只做 **水平面路径跟随**？
2. AUV 类型是否确定为 **欠驱动鱼雷型**？
3. 第一版可视化是否先用 **Python/Plotly 2D 曲线**，而不是复杂 Web 3D？
4. 是否需要优先找一组公开 AUV 3-DOF 参数？
5. 第一版验收是否以“路径跟随 + 抗海流对比”为核心？

---

## 20. 结论

修改后的方案不改变原始技术方向，而是对实现顺序进行了收敛。第一版不追求复杂算法堆叠，而是先把 **水平面 3-DOF 欠驱动 AUV 路径跟随仿真闭环** 做稳定。

推荐第一版聚焦：

```text
3-DOF 欠驱动模型
+ LOS 路径跟随
+ 航向 PID baseline
+ 海流扰动
+ 参数偏差
+ 误差与控制输入展示
```

在这个基础上，再逐步加入：

```text
ADRC/ESO → SMC → Backstepping + SMC/NTSM → MPC/NMPC → Web/ROS 展示
```

这样项目逻辑更清楚，仿真实现风险更低，最终展示也更容易形成稳定结果。

