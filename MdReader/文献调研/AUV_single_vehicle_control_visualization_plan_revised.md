# AUV 单体控制算法仿真可视化方案（修改版）

> 方案定位：在已有水平面 3-DOF AUV 仿真、PID/LOS 路径跟随、恒定海流、圆形路径和 S 形路径测试的基础上，构建一个轻量、稳定、可解释的仿真结果可视化系统。  
> 第一版目标：不做复杂 3D，不引入 ROS/Gazebo，不重构 C++ 仿真核心，而是先通过标准 CSV 数据和 Plotly HTML 报告，把控制效果、误差变化和控制输入讲清楚。

---

## 1. 可视化目标

当前项目已经完成了水平面 3-DOF 简化模型、PID/LOS 路径跟随、恒定海流、圆形路径和 S 形路径等基础仿真模块。可视化部分的第一版目标不是做复杂 3D，而是先形成一个稳定、清楚、可解释的仿真结果展示系统。

第一版可视化需要满足：

1. 能展示 AUV 实际轨迹与参考路径；
2. 能展示横向路径误差、航向误差、速度和控制输入；
3. 能对直线、圆形、S 形三类路径跟随场景生成固定报告；
4. 能在每个报告中给出场景信息和定量 summary 指标；
5. 能显示控制输入限幅线，判断执行器是否长期饱和；
6. 能处理航向角跨越 $\pi$ 与 $-\pi$ 时的跳变问题；
7. 生成的结果可以直接在浏览器中打开，方便演示；
8. 不影响 C++ 核心算法模块，保持仿真和展示解耦。

整体流程设计为：

```text
C++ 仿真程序 / 测试场景
  -> 输出标准化 CSV 数据
  -> Python 读取并检查 CSV 字段
  -> 计算 summary 指标
  -> 生成交互式 HTML 图表
  -> 浏览器打开展示
```

---

## 2. 技术路线

第一版推荐采用：

```text
Python + pandas + Plotly
```

可选依赖：

```text
numpy
```

选择该路线的原因：

1. 当前 C++ 仿真已经能输出 CSV，Python 读取 CSV 很方便；
2. Plotly 可以生成独立 HTML 文件，不需要额外启动 Web 服务器；
3. HTML 图表支持缩放、悬停查看数值、图例开关，展示效果优于静态图片；
4. 不需要改动 C++ 核心算法结构；
5. 后续如果要升级为网页仪表盘，可以继续复用当前数据格式。

第一版暂不建议使用：

```text
Three.js
ROS / Gazebo
复杂 Web 前端框架
```

原因是当前项目仍处于算法仿真基础版本阶段，优先目标是把控制效果讲清楚，而不是一开始就投入复杂三维展示。

---

## 3. 数据来源与 CSV 规范

### 3.1 数据来源

当前主要使用测试程序生成的 CSV 数据。

推荐从项目根目录运行以下程序：

```powershell
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe
```

生成的数据文件包括：

```text
test_outputs/straight_line_path_following.csv
test_outputs/circle_path_following.csv
test_outputs/s_curve_path_following.csv
```

也可以使用以下响应测试数据：

```text
test_outputs/heading_step_response.csv
test_outputs/speed_step_response.csv
```

### 3.2 必须固定 CSV 字段规范

可视化脚本开发前，建议先固定 CSV 字段名称、单位、坐标系和角度定义。否则 C++ 输出字段一旦反复修改，Python 可视化脚本会频繁失效。

路径跟随 CSV 建议包含以下字段：

| 字段 | 含义 | 单位 | 坐标系 / 说明 |
|---|---|---|---|
| `time` | 仿真时间 | s | 从 0 开始 |
| `x` | AUV 北向位置 | m | NED 水平面坐标 |
| `y` | AUV 东向位置 | m | NED 水平面坐标 |
| `psi` | 实际航向角 | rad | 原始数据保留弧度 |
| `u` | 纵向速度 | m/s | 体坐标系 |
| `v` | 横向速度 | m/s | 体坐标系 |
| `r` | 偏航角速度 | rad/s | 体坐标系 |
| `psi_d` | LOS 期望航向角 | rad | 原始数据保留弧度 |
| `cross_track_error` | 横向路径误差 | m | 建议保留正负号 |
| `heading_error` | 航向误差 | rad | 应由 `wrapToPi(psi_d - psi)` 得到 |
| `X_cmd` | 纵向推力命令 | N | 体坐标系 |
| `N_cmd` | 偏航力矩命令 | N·m | 体坐标系 |

可选字段：

| 字段 | 含义 | 单位 | 说明 |
|---|---|---|---|
| `desired_speed` | 期望纵向速度 | m/s | 用于速度跟踪对比 |
| `X_max` | 纵向推力上限 | N | 可用于画限幅线 |
| `X_min` | 纵向推力下限 | N | 可用于画限幅线 |
| `N_max` | 偏航力矩上限 | N·m | 可用于画限幅线 |
| `N_min` | 偏航力矩下限 | N·m | 可用于画限幅线 |
| `current_x` | 惯性系海流 x 分量 | m/s | 用于海流可视化 |
| `current_y` | 惯性系海流 y 分量 | m/s | 用于海流可视化 |

> 建议：CSV 内部角度统一使用 rad，图表中为了便于阅读，可以将 `heading_error`、`psi`、`psi_d` 转换为 degree 展示。

### 3.3 参考路径与参考点需要区分

路径跟随任务中，参考路径不一定与时间绑定。`x_ref, y_ref` 容易产生歧义，需要明确它们表示的是：

1. 完整参考路径采样点；
2. 最近路径点；
3. LOS 前视点；
4. 时间参数化参考轨迹点。

第一版建议采用两个文件：

```text
scenario_path.csv        # 保存完整参考路径
simulation_log.csv       # 保存仿真过程数据
```

其中 `scenario_path.csv` 包含：

```text
path_x, path_y
```

`simulation_log.csv` 包含：

```text
time, x, y, psi, u, v, r, psi_d, cross_track_error, heading_error, X_cmd, N_cmd
```

如果暂时不拆分文件，建议至少把字段命名区分为：

```text
x_path, y_path       # 原始参考路径点
x_los, y_los         # LOS 前视点
x_nearest, y_nearest # 最近路径点
```

不要把这些含义都混写成 `x_ref, y_ref`。

---

## 4. 第一版可视化内容

### 4.1 场景信息表

每个 HTML 报告顶部建议加入场景信息表，用于说明当前图表对应的仿真条件。

| 项目 | 内容示例 |
|---|---|
| Scenario | straight / circle / s-curve |
| Controller | LOS + PID |
| Model | horizontal 3-DOF underactuated AUV |
| Actuation | $\tau_h=[X,0,N]^T$ |
| Current | none / constant current |
| Parameter set | nominal / disturbed |
| Simulation time | 100 s |
| Time step | 0.01 s |
| Speed reference | 1.0 m/s |

这样老师打开报告后，可以立即知道这组结果的仿真条件，而不是只看到曲线。

### 4.2 Summary 指标表

仅画曲线不够，第一版建议直接加入定量指标表。每个 HTML 报告顶部给出：

| 指标 | 含义 |
|---|---|
| `RMS cross-track error` | 横向路径误差均方根 |
| `Max abs cross-track error` | 最大绝对横向路径误差 |
| `RMS heading error` | 航向误差均方根 |
| `Max abs heading error` | 最大绝对航向误差 |
| `Mean speed` | 平均纵向速度 |
| `Max |X_cmd|` | 最大纵向推力命令 |
| `Max |N_cmd|` | 最大偏航力矩命令 |
| `Energy proxy` | 控制输入平方积分近似 |
| `Saturation ratio` | 控制输入触碰限幅的时间占比 |

能耗近似指标可定义为：

$$
E \approx \sum_{k=0}^{N} \left( X_{cmd,k}^2 + N_{cmd,k}^2 \right) \Delta t
$$

如果推力和力矩量纲差异较大，也可以进行归一化：

$$
E_{norm} \approx \sum_{k=0}^{N}
\left[
\left(\frac{X_{cmd,k}}{X_{max}}\right)^2
+
\left(\frac{N_{cmd,k}}{N_{max}}\right)^2
\right]
\Delta t
$$

饱和比例可定义为：

$$
R_{sat}=
\frac{N_{sat}}{N_{total}}
$$

其中 $N_{sat}$ 表示控制输入达到或接近限幅的采样点数量，$N_{total}$ 表示总采样点数量。

### 4.3 AUV 轨迹图

展示内容：

```text
参考路径
实际轨迹
起点
终点
若干时刻的 AUV 朝向箭头
可选：海流方向箭头
可选：实际轨迹按误差大小着色
```

轨迹图是最重要的展示图，用于直观看 AUV 是否沿参考路径运动。

要求：

1. 横纵坐标使用等比例；
2. 直线路径应显示为直线；
3. 圆形路径应显示为圆，而不是椭圆；
4. S 形路径应能清楚看出连续转向；
5. 起点和终点应有明确标记；
6. 悬停信息建议显示 `time, x, y, cross_track_error, heading_error`；
7. 如果加入海流，应在图中显示海流方向和大小。

增强建议：

```text
实际轨迹可按照 abs(cross_track_error) 进行颜色映射
```

这样可以直观看出路径误差最大的位置。

### 4.4 横向误差图

展示：

```text
time - cross_track_error
```

用途：

1. 判断 LOS 路径跟随是否收敛；
2. 对比不同路径场景下误差大小；
3. 用于后续有无海流、有无参数偏差的性能比较。

建议：

1. 保留误差正负号；
2. 同时在 summary 中计算 RMS 和最大绝对值；
3. 可增加零误差参考线。

### 4.5 航向误差图

展示：

```text
time - heading_error
```

用途：

1. 判断航向 PID 是否能稳定跟踪 LOS 给出的期望航向；
2. 检查是否存在明显振荡；
3. 检查路径切换或曲线路径跟随时是否出现不合理跳变。

角度处理要求：

1. CSV 中 `psi`、`psi_d`、`heading_error` 建议保留 rad；
2. 画航向角曲线时可使用 unwrap，避免 $\pi$ 到 $-\pi$ 的显示跳变；
3. 画航向误差时必须使用 wrapToPi；
4. 图表中建议转换为 degree，便于阅读。

航向误差推荐计算方式：

$$
 e_{\psi} = \operatorname{wrapToPi}(\psi_d - \psi)
$$

### 4.6 速度响应图

展示：

```text
time - u
```

如果 CSV 增加 `desired_speed` 字段，可以同时展示：

```text
time - desired_speed
```

用途：

1. 判断纵向速度控制是否稳定；
2. 检查曲线路径跟随时速度是否出现异常波动；
3. 检查海流扰动下速度响应是否合理。

### 4.7 控制输入图

展示：

```text
time - X_cmd
time - N_cmd
```

建议同时显示限幅线：

```text
X_max, X_min
N_max, N_min
```

用途：

1. 判断纵向推力和偏航力矩是否超出限幅；
2. 检查控制输入是否频繁饱和；
3. 检查控制输入是否出现高频震荡；
4. 辅助判断是否需要 anti-windup 或控制输入平滑处理。

---

## 5. 输出文件设计

建议生成以下 HTML 报告：

```text
results/visualization/straight_path_report.html
results/visualization/circle_path_report.html
results/visualization/s_curve_path_report.html
```

每个报告包含：

```text
场景信息表
summary 指标表
轨迹图
横向误差图
航向误差图
速度图
控制输入图
```

额外生成一个总入口页面：

```text
results/visualization/index.html
```

总入口页面包含三个场景链接：

```text
直线路径跟随
圆形路径跟随
S 形路径跟随
```

后续扩展时，`index.html` 可以继续加入：

```text
有无海流对比
有无参数偏差对比
PID vs ADRC / SMC 对比
```

---

## 6. 建议新增文件结构

建议在项目中增加：

```text
scripts/
  plot_simulation_results.py
  generate_visualization_index.py
  batch_generate_reports.py

results/
  data/
    straight_line_path_following.csv
    circle_path_following.csv
    s_curve_path_following.csv

results/
  visualization/
    assets/
      plotly.min.js       # 可选，用于离线模式
    index.html
    straight_path_report.html
    circle_path_report.html
    s_curve_path_report.html
```

其中：

1. `plot_simulation_results.py` 用于读取单个 CSV 并生成对应 HTML 报告；
2. `generate_visualization_index.py` 用于生成汇总入口页面；
3. `batch_generate_reports.py` 用于一键生成全部场景报告；
4. `results/data/` 用于保存或复制待展示的数据；
5. `results/visualization/` 用于保存可直接打开的展示结果。

推荐一键生成命令：

```powershell
python scripts/batch_generate_reports.py
```

---

## 7. 展示场景设计

### 7.1 场景 1：直线路径跟随

数据文件：

```text
test_outputs/straight_line_path_following.csv
```

展示目的：

1. 展示基础 LOS + PID 闭环是正确的；
2. 展示 AUV 从偏离位置逐渐收敛到直线路径；
3. 展示横向误差和航向误差逐渐减小。

重点观察：

```text
轨迹是否贴近直线
cross_track_error 是否收敛
heading_error 是否稳定
X_cmd 和 N_cmd 是否合理
控制输入是否触碰限幅
```

### 7.2 场景 2：圆形路径跟随

数据文件：

```text
test_outputs/circle_path_following.csv
```

展示目的：

1. 展示曲线路径跟随能力；
2. 展示 AUV 能够绕圆形参考路径运动；
3. 展示路径跟随系统在连续转向时不发散。

重点观察：

```text
实际轨迹是否围绕圆形路径
横向误差是否保持在较小范围
期望航向变化是否连续
偏航力矩 N_cmd 是否无明显异常震荡
圆形路径是否因为坐标比例错误显示成椭圆
```

### 7.3 场景 3：S 形路径跟随

数据文件：

```text
test_outputs/s_curve_path_following.csv
```

展示目的：

1. 展示连续机动路径跟随能力；
2. 展示 AUV 可以跟随方向不断变化的参考路径；
3. 检查路径段切换是否平滑。

重点观察：

```text
实际轨迹是否跟随 S 形曲线
heading_error 是否存在不合理跳变
cross_track_error 是否保持在可接受范围
X_cmd 和 N_cmd 是否平滑
是否存在路径点切换导致的控制输入尖峰
```

---

## 8. 第一版实施步骤

### 步骤 1：固定 CSV 字段规范

实现内容：

1. 确定必须字段和可选字段；
2. 确定单位、坐标系和角度单位；
3. C++ 输出端和 Python 读取端使用同一份字段规范。

验证标准：

1. 所有路径跟随 CSV 至少包含必须字段；
2. 字段名称在不同测试场景中保持一致；
3. 缺失字段时 Python 脚本给出明确提示。

### 步骤 2：读取 CSV 并做字段检查

实现内容：

1. 使用 Python 读取 CSV；
2. 检查必要字段是否存在；
3. 对缺失字段给出清晰错误提示；
4. 自动识别可选字段，如限幅、海流、期望速度等。

验证标准：

1. 能正确读取 `straight_line_path_following.csv`；
2. 能识别 `time`、`x`、`y`、`psi`、`cross_track_error`、`heading_error` 等字段；
3. 如果 CSV 不存在或字段缺失，脚本不崩溃，而是给出可理解的错误信息。

### 步骤 3：计算 summary 指标

实现内容：

1. 计算 RMS 横向误差；
2. 计算最大绝对横向误差；
3. 计算 RMS 航向误差；
4. 计算最大绝对航向误差；
5. 计算平均速度；
6. 计算控制输入峰值；
7. 计算能耗近似指标；
8. 如果有限幅字段，计算控制输入饱和比例。

验证标准：

1. 每个 HTML 报告顶部能显示 summary 表；
2. 指标数值单位明确；
3. 指标计算不受空值或缺失可选字段影响。

### 步骤 4：生成单个轨迹图

实现内容：

1. 绘制实际轨迹 `x-y`；
2. 绘制参考路径或参考点；
3. 标记起点和终点；
4. 设置等比例坐标轴；
5. 可选：绘制 AUV 朝向箭头；
6. 可选：绘制海流方向箭头；
7. 可选：实际轨迹按误差大小着色。

验证标准：

1. 直线路径显示为直线；
2. 圆形路径显示为圆；
3. S 形路径连续平滑；
4. 图例和坐标轴清晰；
5. 起点和终点易识别。

### 步骤 5：生成误差、速度和控制输入图

实现内容：

1. 绘制横向误差；
2. 绘制航向误差；
3. 绘制纵向速度；
4. 绘制 `X_cmd` 和 `N_cmd`；
5. 若存在限幅字段，绘制控制输入限幅线；
6. 航向误差图建议转为 degree。

验证标准：

1. 每个子图都能正常显示；
2. 时间轴一致；
3. 单位和标题明确；
4. 控制输入曲线能看出是否触碰限幅。

### 步骤 6：生成单场景 HTML 报告

实现内容：

1. 将场景信息表、summary 表、轨迹图、误差图、速度图、控制输入图整合为一个 HTML；
2. 每个 HTML 对应一个场景；
3. 文件保存到 `results/visualization/`。

验证标准：

1. HTML 可双击打开；
2. 图表能缩放、悬停查看数值；
3. 报告中场景名称明确；
4. 关闭网络时也能正常查看。

### 步骤 7：生成汇总入口页面

实现内容：

1. 生成 `results/visualization/index.html`；
2. 页面中包含三个场景报告链接；
3. 链接能跳转到对应 HTML。

验证标准：

1. 双击 `index.html` 能打开；
2. 三个场景链接均可访问；
3. 页面结构清晰，适合演示前快速选择场景。

### 步骤 8：加入多场景或多算法对比

第一版稳定后，再扩展：

1. 同一路径下有无海流对比；
2. 同一路径下有无参数偏差对比；
3. PID vs ADRC / SMC 对比；
4. 多场景 summary 指标总表。

---

## 9. 第一版验收标准

第一版可视化完成后，应满足：

1. 可以从已有 CSV 数据生成 HTML 报告；
2. 至少支持直线、圆形、S 形三类路径跟随结果；
3. 每个报告包含场景信息表和 summary 指标表；
4. 每个报告包含轨迹图、横向误差图、航向误差图、速度图、控制输入图；
5. 轨迹图坐标比例正确；
6. 航向角和航向误差处理正确，避免 $\pi$ 与 $-\pi$ 附近的显示跳变；
7. 控制输入图包含限幅线，能够看出是否触碰饱和；
8. 图表标题、图例、坐标轴和单位清晰；
9. 生成的 HTML 能直接用浏览器打开；
10. `index.html` 能跳转到三个场景报告；
11. README 中给出运行可视化脚本和查看结果的方法；
12. 脚本对字段缺失、文件不存在等问题给出清晰错误提示。

---

## 10. 后续扩展方向

第一版完成后，可以继续扩展：

1. 增加有无海流对比图；
2. 增加有无参数偏差对比图；
3. 增加 PID、ADRC、SMC 等不同控制器对比；
4. 增加 RMS 横向误差、最大误差、控制能耗等跨场景 summary 总表；
5. 将多场景结果整合为一个总 dashboard；
6. 增加简化 AUV 图标或姿态箭头动画；
7. 后续再考虑 Three.js 3D 可视化。

推荐扩展顺序：

```text
Plotly 静态交互报告
  -> 指标 summary
  -> 多场景 / 多算法对比
  -> HTML dashboard
  -> 简化动画
  -> 3D 展示
```

注意：指标 summary 应前置到第一版完成，而不是等到后期再做。它实现成本低，但对报告、答辩和项目演示的解释价值很高。

---

## 11. 实现过程中容易出现的问题与应对

| 问题 | 表现 | 应对 |
|---|---|---|
| CSV 字段不统一 | 不同场景脚本频繁报错 | 先固定字段规范，再写可视化脚本 |
| 角度跳变 | 航向曲线在 $\pi$ 与 $-\pi$ 处出现尖峰 | 航向角画图用 unwrap，航向误差用 wrapToPi |
| 圆形路径显示成椭圆 | 坐标轴比例不一致 | 轨迹图必须设置等比例坐标 |
| 控制输入解释不清 | 不知道是否饱和 | 控制输入图加入限幅线和饱和比例 |
| 参考路径含义不清 | `x_ref,y_ref` 不知道是路径点还是前视点 | 区分 `path`、`nearest`、`LOS` 点 |
| 曲线能看但结论弱 | 只能主观判断好坏 | 加入 RMS、最大误差、能耗等 summary 指标 |
| HTML 离线打不开 | Plotly 依赖在线 CDN | 使用内嵌 Plotly 或本地 `plotly.min.js` |
| 老师看不懂场景条件 | 只有图，没有仿真配置 | 每个报告顶部加场景信息表 |

---

## 12. 推荐结论

第一版可视化建议采用：

```text
C++ 输出标准化 CSV
        ↓
Python 检查字段与单位
        ↓
计算 summary 指标
        ↓
Plotly 生成单场景 HTML 报告
        ↓
index.html 统一入口
        ↓
后续扩展多算法 / 多扰动对比
```

对于当前项目阶段，最合适的展示重点是：

```text
参考路径和实际轨迹是否一致
横向误差是否收敛
航向误差是否稳定
速度响应是否合理
控制输入是否平滑且不长期饱和
summary 指标是否能支撑算法效果结论
```

该方案实现成本低、展示效果清晰、便于调试，也不会影响 C++ 核心算法结构。第一版不建议投入复杂 3D 或 ROS/Gazebo，应优先把数据规范、定量指标和单场景报告做扎实。
