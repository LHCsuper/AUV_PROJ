# AUV 单体控制算法仿真可视化方案

## 1. 可视化目标

当前项目已经完成了水平面 3-DOF 简化模型、PID/LOS 路径跟随、恒定海流、圆形路径和 S 形路径等基础仿真模块。可视化部分的第一版目标不是做复杂 3D，而是先形成一个稳定、清楚、可解释的仿真结果展示系统。

第一版可视化需要满足：

1. 能展示 AUV 实际轨迹与参考路径；
2. 能展示横向路径误差、航向误差、速度和控制输入；
3. 能对直线、圆形、S 形三类路径跟随场景生成固定报告；
4. 生成的结果可以直接在浏览器中打开，方便给老师演示；
5. 不影响 C++ 核心算法模块，保持仿真和展示解耦。

整体流程设计为：

```text
C++ 仿真程序 / 测试场景
  -> 输出 CSV 数据
  -> Python 读取 CSV
  -> 生成交互式 HTML 图表
  -> 浏览器打开展示
```

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

## 3. 数据来源

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

路径跟随 CSV 中建议重点使用的字段：

```text
time
x, y, psi
u, v, r
x_ref, y_ref
psi_d
cross_track_error
heading_error
X_cmd, N_cmd
```

## 4. 第一版可视化内容

### 4.1 AUV 轨迹图

展示内容：

```text
参考路径
实际轨迹
起点
终点
若干时刻的 AUV 朝向箭头
```

轨迹图是最重要的展示图，用于直观看 AUV 是否沿参考路径运动。

要求：

1. 横纵坐标使用等比例；
2. 直线路径应显示为直线；
3. 圆形路径应显示为圆，而不是椭圆；
4. S 形路径应能清楚看出连续转向；
5. 起点和终点应有明确标记。

### 4.2 横向误差图

展示：

```text
time - cross_track_error
```

用途：

1. 判断 LOS 路径跟随是否收敛；
2. 对比不同路径场景下误差大小；
3. 后续可以用于有无海流、有无参数偏差的性能比较。

### 4.3 航向误差图

展示：

```text
time - heading_error
```

用途：

1. 判断航向 PID 是否能稳定跟踪 LOS 给出的期望航向；
2. 检查是否存在明显振荡；
3. 检查路径切换或曲线路径跟随时是否出现不合理跳变。

### 4.4 速度响应图

展示：

```text
time - u
```

如果后续 CSV 增加 `desired_speed` 字段，可以同时展示：

```text
time - desired_speed
```

用途：

1. 判断纵向速度控制是否稳定；
2. 检查曲线路径跟随时速度是否出现异常波动。

### 4.5 控制输入图

展示：

```text
time - X_cmd
time - N_cmd
```

用途：

1. 判断纵向推力和偏航力矩是否超出限幅；
2. 检查控制输入是否频繁饱和；
3. 检查控制输入是否出现高频震荡。

## 5. 输出文件设计

建议生成以下 HTML 报告：

```text
results/visualization/straight_path_report.html
results/visualization/circle_path_report.html
results/visualization/s_curve_path_report.html
```

每个报告包含：

```text
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

## 6. 建议新增文件结构

建议在项目中增加：

```text
scripts/
  plot_simulation_results.py
  generate_visualization_index.py

results/
  visualization/
    index.html
    straight_path_report.html
    circle_path_report.html
    s_curve_path_report.html
```

其中：

1. `plot_simulation_results.py` 用于读取单个 CSV 并生成对应 HTML 报告；
2. `generate_visualization_index.py` 用于生成汇总入口页面；
3. `results/visualization/` 用于保存可直接打开的展示结果。

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
```

## 8. 第一版实施步骤

### 步骤 1：读取 CSV

实现内容：

1. 使用 Python 读取 CSV；
2. 检查必要字段是否存在；
3. 对缺失字段给出清晰错误提示。

验证标准：

1. 能正确读取 `straight_line_path_following.csv`；
2. 能识别 `time`、`x`、`y`、`psi`、`cross_track_error`、`heading_error` 等字段；
3. 如果 CSV 不存在或字段缺失，脚本不崩溃，而是给出可理解的错误信息。

### 步骤 2：生成单个轨迹图

实现内容：

1. 绘制实际轨迹 `x-y`；
2. 绘制参考路径或参考点 `x_ref-y_ref`；
3. 标记起点和终点；
4. 设置等比例坐标轴。

验证标准：

1. 直线路径显示为直线；
2. 圆形路径显示为圆；
3. S 形路径连续平滑；
4. 图例和坐标轴清晰。

### 步骤 3：生成误差和控制输入子图

实现内容：

1. 绘制横向误差；
2. 绘制航向误差；
3. 绘制纵向速度；
4. 绘制 `X_cmd` 和 `N_cmd`。

验证标准：

1. 每个子图都能正常显示；
2. 时间轴一致；
3. 单位和标题明确；
4. 控制输入曲线能看出是否触碰限幅。

### 步骤 4：生成单场景 HTML 报告

实现内容：

1. 将轨迹图、误差图、速度图、控制输入图整合为一个 HTML；
2. 每个 HTML 对应一个场景；
3. 文件保存到 `results/visualization/`。

验证标准：

1. HTML 可双击打开；
2. 图表能缩放、悬停查看数值；
3. 报告中场景名称明确；
4. 关闭网络时也能正常查看。

### 步骤 5：生成汇总入口页面

实现内容：

1. 生成 `results/visualization/index.html`；
2. 页面中包含三个场景报告链接；
3. 链接能跳转到对应 HTML。

验证标准：

1. 双击 `index.html` 能打开；
2. 三个场景链接均可访问；
3. 页面结构清晰，适合演示前快速选择场景。

## 9. 第一版验收标准

第一版可视化完成后，应满足：

1. 可以从已有 CSV 数据生成 HTML 报告；
2. 至少支持直线、圆形、S 形三类路径跟随结果；
3. 每个报告包含轨迹图、横向误差图、航向误差图、速度图、控制输入图；
4. 轨迹图坐标比例正确；
5. 图表标题、图例、坐标轴清晰；
6. 生成的 HTML 能直接用浏览器打开；
7. README 中给出运行可视化脚本和查看结果的方法。

## 10. 后续扩展方向

第一版完成后，可以继续扩展：

1. 增加有无海流对比图；
2. 增加有无参数偏差对比图；
3. 增加 RMS 横向误差、最大误差、控制能耗等 summary 指标；
4. 将多场景结果整合为一个总 dashboard；
5. 增加简化 AUV 图标或姿态箭头动画；
6. 后续再考虑 Three.js 3D 可视化。

推荐扩展顺序：

```text
Plotly 静态交互报告
  -> 多场景对比
  -> 指标 summary
  -> HTML dashboard
  -> 简化动画
  -> 3D 展示
```

## 11. 结论

第一版可视化建议采用“C++ 输出 CSV + Python/Plotly 生成 HTML 报告”的方案。该方案实现成本低、展示效果清晰、便于调试，也不会影响 C++ 核心算法结构。

对于当前项目阶段，最合适的展示重点是：

```text
参考路径和实际轨迹是否一致
横向误差是否收敛
航向误差是否稳定
速度响应是否合理
控制输入是否平滑且不长期饱和
```

先把这些内容做扎实，再进一步扩展复杂场景和三维展示。
