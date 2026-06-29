# AUV_PROJ

本项目用于搭建一个基于 C++ 的 AUV 单体控制算法仿真与可视化展示系统。当前项目已经包含水平面 3-DOF 欠驱动 AUV 模型、LOS 路径跟随、速度/航向 PID 控制、直线/圆形/S 形路径场景、CSV/JSON 数据输出、HTML 报告和 Electron 可操作可视化 APP。

项目定位是横向项目交付展示：核心算法模块用 C++ 实现，展示部分使用 Web/Electron，最终可以打包成离线桌面程序运行。

## 1. 项目目录

```text
AUV_proj/
  apps/auv_sim/                  # C++ 最小入口程序
  include/auv/                   # C++ 头文件
  src/                           # C++ 源文件
  tests/                         # 仿真场景与验证程序
  scripts/                       # CSV 转 JSON、HTML 报告生成脚本
  web/                           # 可操作可视化 APP，React + ECharts + Electron
  results/visualization/         # 静态 HTML 可视化报告
  test_outputs/                  # C++ 仿真生成的 CSV，运行后生成
  build/cmake/                   # CMake 构建目录，运行后生成
  tools/                         # CMake、Eigen、Electron 缓存等本地工具
  MdReader/文献调研/             # 文献调研、项目方案和综述文档
```

## 2. 环境说明

所有命令默认在 Windows PowerShell 中执行。

进入项目根目录：

```powershell
cd E:\AUV_proj
```

如果当前 PowerShell 不在 E 盘，也可以使用：

```powershell
Set-Location E:\AUV_proj
```

项目当前使用：

```text
C++: C++17
构建: CMake + MinGW Makefiles
数学库: Eigen
可视化 APP: React + ECharts + Electron
```

CMake 和 Eigen 已按项目方式放在 E 盘目录下，不需要安装到 C 盘：

```text
tools/cmake/cmake-4.3.3-windows-x86_64/
tools/eigen/eigen-3.4.1/
```

## 3. 配置和编译 C++ 项目

第一次运行、清理过 `build/`、或者修改了 `CMakeLists.txt` 后，需要重新配置：

```powershell
cd E:\AUV_proj
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe -S . -B build\cmake -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER=g++
```

编译项目：

```powershell
cd E:\AUV_proj
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
```

编译成功后，可执行文件位于：

```text
build/cmake/
```

运行全部 C++ 验证：

```powershell
cd E:\AUV_proj
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\ctest.exe --test-dir build\cmake --output-on-failure
```

## 4. 运行仿真并生成 CSV

### 4.1 最小入口程序

```powershell
cd E:\AUV_proj
.\build\cmake\auv_sim.exe
```

该程序用于确认 C++ 工程和 Eigen 链接正常，不生成路径跟随数据。

### 4.2 航向 PID 阶跃响应

```powershell
cd E:\AUV_proj
.\build\cmake\test_heading_pid.exe
```

输出：

```text
test_outputs/heading_step_response.csv
```

### 4.3 纵向速度控制响应

```powershell
cd E:\AUV_proj
.\build\cmake\test_speed_controller.exe
```

输出：

```text
test_outputs/speed_step_response.csv
```

### 4.4 直线路径 LOS 跟随

```powershell
cd E:\AUV_proj
.\build\cmake\test_path_following.exe
```

输出：

```text
test_outputs/straight_line_path_following.csv
```

### 4.5 圆形路径和 S 形路径跟随

```powershell
cd E:\AUV_proj
.\build\cmake\test_curved_path_following.exe
```

输出：

```text
test_outputs/circle_path_following.csv
test_outputs/s_curve_path_following.csv
```

路径跟随 CSV 主要字段：

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

## 5. 生成静态 HTML 报告

先运行路径跟随仿真：

```powershell
cd E:\AUV_proj
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe
```

再生成 HTML 报告：

```powershell
cd E:\AUV_proj
py scripts\batch_generate_reports.py
```

生成结果：

```text
results/visualization/index.html
results/visualization/straight_path_report.html
results/visualization/circle_path_report.html
results/visualization/s_curve_path_report.html
```

推荐打开总入口：

```text
E:\AUV_proj\results\visualization\index.html
```

## 6. 运行可操作可视化 APP

APP 位于：

```text
web/
```

### 6.1 生成 APP 所需 JSON 数据

如果 C++ 仿真结果发生变化，需要先重新生成 CSV：

```powershell
cd E:\AUV_proj
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe
```

再把 CSV 转成 APP 使用的 JSON：

```powershell
cd E:\AUV_proj
py scripts\convert_simulation_csv_to_json.py --input test_outputs\straight_line_path_following.csv --output web\public\data\straight.json --scenario straight
py scripts\convert_simulation_csv_to_json.py --input test_outputs\circle_path_following.csv --output web\public\data\circle.json --scenario circle
py scripts\convert_simulation_csv_to_json.py --input test_outputs\s_curve_path_following.csv --output web\public\data\s_curve.json --scenario s_curve
```

### 6.2 开发模式运行

```powershell
cd E:\AUV_proj\web
npm run dev -- --port 5173
```

浏览器打开：

```text
http://127.0.0.1:5173
```

### 6.3 构建前端静态文件

```powershell
cd E:\AUV_proj\web
npm run build
```

输出目录：

```text
web/dist/
```

### 6.4 打包 Windows 桌面程序

```powershell
cd E:\AUV_proj\web
npm run electron:pack
```

输出目录：

```text
web/release/win-unpacked/
```

运行程序：

```text
web/release/win-unpacked/AUV Visualization.exe
```

交付给其他电脑时，需要复制整个文件夹：

```text
web/release/win-unpacked/
```

不要只复制 `.exe`，因为同目录下的 `resources/`、`locales/`、`.dll`、`.pak` 等文件也是运行所需内容。

如需压缩交付，可压缩整个 `win-unpacked` 文件夹。

## 7. 项目参数如何修改

这一节是后续改项目最重要的部分。当前项目参数分为五类：AUV 模型参数、控制器参数、仿真场景参数、可视化数据参数、前端展示参数。

### 7.1 AUV 模型参数改哪里

AUV 水平面 3-DOF 模型默认参数定义在：

```text
include/auv/model/auv_3dof_params.h
```

主要参数：

```cpp
struct Auv3DofParams {
    double m11{60.0};       // 纵向等效惯性
    double m22{80.0};       // 横向等效惯性
    double m33{12.0};       // 偏航等效转动惯量

    double d_u{18.0};       // 纵向线性阻尼
    double d_v{30.0};       // 横向线性阻尼
    double d_r{8.0};        // 偏航线性阻尼

    double d_uu{25.0};      // 纵向二次阻尼
    double d_vv{45.0};      // 横向二次阻尼
    double d_rr{12.0};      // 偏航二次阻尼

    double max_X{80.0};     // 纵向推力限幅，单位近似 N
    double max_N{20.0};     // 偏航力矩限幅，单位近似 N*m

    double thruster_scale{1.0};
};
```

参数校验逻辑在：

```text
src/model/auv_3dof_params.cpp
```

一般只需要改 `include/auv/model/auv_3dof_params.h` 中的默认值，不建议随意修改校验逻辑。

改完 AUV 模型参数后，需要：

```powershell
cd E:\AUV_proj
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe
```

如果要更新 APP 数据，还要继续执行 CSV 转 JSON：

```powershell
py scripts\convert_simulation_csv_to_json.py --input test_outputs\straight_line_path_following.csv --output web\public\data\straight.json --scenario straight
py scripts\convert_simulation_csv_to_json.py --input test_outputs\circle_path_following.csv --output web\public\data\circle.json --scenario circle
py scripts\convert_simulation_csv_to_json.py --input test_outputs\s_curve_path_following.csv --output web\public\data\s_curve.json --scenario s_curve
```

### 7.2 直线路径场景参数改哪里

直线路径场景在：

```text
tests/test_path_following.cpp
```

常改位置：

```cpp
LineOfSightGuidance(8.0)                 // LOS 前视距离
PidController({90.0, 25.0, 0.0}, ...)    // 速度 PID: kp, ki, kd
PidController({10.0, 0.0, 8.0}, ...)     // 航向 PID: kp, ki, kd

reference.path = LineSegment2D{{0.0, 0.0}, {120.0, 0.0}};  // 直线路径起点和终点
reference.desired_speed = 1.0;                              // 目标航速

state.x = 0.0;       // 初始 x
state.y = 5.0;       // 初始 y
state.psi = 0.0;     // 初始航向，单位 rad

const double dt = 0.02;        // 仿真步长
const double duration = 90.0;  // 仿真时长
```

改完后运行：

```powershell
cd E:\AUV_proj
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
.\build\cmake\test_path_following.exe
```

### 7.3 圆形路径和 S 形路径参数改哪里

圆形路径和 S 形路径在：

```text
tests/test_curved_path_following.cpp
```

控制器参数在 `makeController()` 中：

```cpp
LineOfSightGuidance(6.0)                 // LOS 前视距离
PidController({90.0, 25.0, 0.0}, ...)    // 速度 PID
PidController({12.0, 0.0, 9.0}, ...)     // 航向 PID
```

圆形路径参数：

```cpp
circle_initial.x = 30.0;
circle_initial.y = 0.0;
circle_initial.psi = deg2rad(90.0);

const auto circle = makeCirclePath(
    Eigen::Vector2d{0.0, 0.0},  // 圆心
    25.0,                       // 半径
    120                         // 路径采样点数
);

// desired_speed = 0.6, duration = 120.0
```

S 形路径参数：

```cpp
sine_initial.x = 0.0;
sine_initial.y = -4.0;
sine_initial.psi = deg2rad(25.0);

const auto sine = makeSinePath(
    0.0,     // x 起点
    100.0,   // x 终点
    8.0,     // 振幅
    0.08,    // 频率系数
    160      // 路径采样点数
);

// desired_speed = 0.8, duration = 120.0
```

改完后运行：

```powershell
cd E:\AUV_proj
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
.\build\cmake\test_curved_path_following.exe
```

### 7.4 速度 PID 和航向 PID 参数怎么改

PID 参数结构定义在：

```text
include/auv/controller/pid_controller.h
```

实际场景中的数值主要写在：

```text
tests/test_path_following.cpp
tests/test_curved_path_following.cpp
tests/test_speed_controller.cpp
tests/test_heading_pid.cpp
```

格式如下：

```cpp
PidController({kp, ki, kd}, limits)
```

例如：

```cpp
PidController({90.0, 25.0, 0.0}, speed_limits)   // 速度 PID
PidController({10.0, 0.0, 8.0}, heading_limits)  // 航向 PID
```

一般调参建议：

```text
kp 增大：响应更快，但可能超调或振荡
ki 增大：减小稳态误差，但过大可能积分饱和
kd 增大：增加阻尼，减小振荡，但过大可能放大噪声
```

PID 输出限幅由 `PidLimits` 设置：

```cpp
speed_limits.output_min = -params.max_X;
speed_limits.output_max = params.max_X;
heading_limits.output_min = -params.max_N;
heading_limits.output_max = params.max_N;
```

积分限幅由下面参数设置：

```cpp
speed_limits.integral_min = -10.0;
speed_limits.integral_max = 10.0;
heading_limits.integral_min = -2.0;
heading_limits.integral_max = 2.0;
```

### 7.5 LOS 前视距离怎么改

LOS 制导类定义在：

```text
include/auv/guidance/los_guidance.h
src/guidance/los_guidance.cpp
```

场景中的前视距离在测试场景里设置：

```cpp
LineOfSightGuidance(8.0)   // 直线路径
LineOfSightGuidance(6.0)   // 圆形和 S 形路径
```

一般规律：

```text
前视距离变小：路径修正更积极，可能更容易振荡
前视距离变大：路径修正更平滑，但收敛可能变慢
```

### 7.6 推力和偏航力矩限幅怎么改

限幅默认值在：

```text
include/auv/model/auv_3dof_params.h
```

对应参数：

```cpp
double max_X{80.0};
double max_N{20.0};
```

`max_X` 会影响纵向推力上限，`max_N` 会影响偏航控制力矩上限。修改后需要重新编译、重新运行仿真、重新生成 JSON。

### 7.7 海流参数怎么改

海流模型定义在：

```text
include/auv/environment/current_model.h
src/environment/current_model.cpp
```

当前路径跟随主场景默认没有显式加入海流；海流相关验证主要在：

```text
tests/test_current_model.cpp
```

其中类似：

```cpp
const Current2D current{0.2, 0.0};
```

表示惯性坐标系下的海流速度分量：

```text
vx = 0.2 m/s
vy = 0.0 m/s
```

如果要把海流加入路径跟随主场景，需要在 `tests/test_path_following.cpp` 或 `tests/test_curved_path_following.cpp` 的积分处，把：

```cpp
return model.derivative(s, output.input);
```

改成类似：

```cpp
return model.derivative(s, output.input, auv::environment::Current2D{0.2, 0.0});
```

改完后重新编译并运行对应场景。

### 7.8 Web APP 显示数据怎么改

Web APP 读取的数据文件在：

```text
web/public/data/straight.json
web/public/data/circle.json
web/public/data/s_curve.json
```

这些文件通常不要手动改，而是由 CSV 转换生成：

```powershell
py scripts\convert_simulation_csv_to_json.py --input test_outputs\straight_line_path_following.csv --output web\public\data\straight.json --scenario straight
py scripts\convert_simulation_csv_to_json.py --input test_outputs\circle_path_following.csv --output web\public\data\circle.json --scenario circle
py scripts\convert_simulation_csv_to_json.py --input test_outputs\s_curve_path_following.csv --output web\public\data\s_curve.json --scenario s_curve
```

如果只改了 Web 前端样式或布局，不需要重新跑 C++ 仿真；如果改了 C++ 模型、控制器或场景参数，就必须重新生成 CSV 和 JSON。

### 7.9 前端界面显示、阈值和指标怎么改

前端主界面逻辑在：

```text
web/src/App.jsx
```

样式在：

```text
web/src/styles.css
```

常见修改位置：

```text
场景卡片名称和说明：web/src/App.jsx 中的 SCENARIOS
顶部栏、状态面板、指标面板：web/src/App.jsx
轨迹图和曲线图配置：web/src/App.jsx 中的 ECharts option
颜色、布局、字号：web/src/styles.css
```

前端中用于图表显示的控制输入限幅来自 JSON 的 `limits` 字段；如果 JSON 没有该字段，则前端使用默认值：

```text
X: -80 至 80
N: -20 至 20
```

对应代码在：

```text
web/src/App.jsx
```

搜索：

```text
xCommandMax
xCommandMin
nCommandMax
nCommandMin
```

### 7.10 修改参数后的完整刷新流程

如果改了 C++ 模型参数、控制器参数或场景参数，完整流程是：

```powershell
cd E:\AUV_proj

# 1. 重新编译
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake

# 2. 重新运行仿真，生成 CSV
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe

# 3. 重新生成静态 HTML 报告
py scripts\batch_generate_reports.py

# 4. 重新生成 Web APP JSON 数据
py scripts\convert_simulation_csv_to_json.py --input test_outputs\straight_line_path_following.csv --output web\public\data\straight.json --scenario straight
py scripts\convert_simulation_csv_to_json.py --input test_outputs\circle_path_following.csv --output web\public\data\circle.json --scenario circle
py scripts\convert_simulation_csv_to_json.py --input test_outputs\s_curve_path_following.csv --output web\public\data\s_curve.json --scenario s_curve

# 5. 运行前端开发模式查看
cd E:\AUV_proj\web
npm run dev -- --port 5173
```

如果要重新打包桌面版：

```powershell
cd E:\AUV_proj\web
npm run electron:pack
```

然后重新打开：

```text
E:\AUV_proj\web\release\win-unpacked\AUV Visualization.exe
```

## 8. 推荐日常工作流程

### 8.1 只看当前 APP

```powershell
cd E:\AUV_proj\web
npm run dev -- --port 5173
```

打开：

```text
http://127.0.0.1:5173
```

### 8.2 改了 C++ 参数后看效果

```powershell
cd E:\AUV_proj
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe
py scripts\batch_generate_reports.py
py scripts\convert_simulation_csv_to_json.py --input test_outputs\straight_line_path_following.csv --output web\public\data\straight.json --scenario straight
py scripts\convert_simulation_csv_to_json.py --input test_outputs\circle_path_following.csv --output web\public\data\circle.json --scenario circle
py scripts\convert_simulation_csv_to_json.py --input test_outputs\s_curve_path_following.csv --output web\public\data\s_curve.json --scenario s_curve
```

然后运行 APP：

```powershell
cd E:\AUV_proj\web
npm run dev -- --port 5173
```

### 8.3 准备交付版桌面程序

```powershell
cd E:\AUV_proj\web
npm run electron:pack
```

交付目录：

```text
E:\AUV_proj\web\release\win-unpacked
```

## 9. 常见问题

### 9.1 修改参数后 APP 没变化

通常是因为只改了 C++，但没有重新生成 JSON。正确流程是：

```text
改 C++ 参数 → 编译 → 跑仿真生成 CSV → CSV 转 JSON → 重新打开 APP
```

### 9.2 桌面快捷方式打开后还是旧界面

可能原因：

```text
1. Electron 没有重新打包
2. 快捷方式指向旧的 exe
3. 旧的 AUV Visualization 进程还在运行
```

建议重新打包：

```powershell
cd E:\AUV_proj\web
npm run electron:pack
```

然后直接打开：

```text
E:\AUV_proj\web\release\win-unpacked\AUV Visualization.exe
```

### 9.3 找不到 `cmake.exe`

确认当前目录是：

```text
E:\AUV_proj
```

并检查：

```text
tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe
```

### 9.4 找不到 `g++`

检查：

```powershell
g++ --version
```

如果失败，需要把 MinGW 的 `bin` 目录加入环境变量，或者在 CMake 配置时填写完整的 `g++.exe` 路径。

### 9.5 没有看到 CSV

确认是从项目根目录运行测试程序：

```powershell
cd E:\AUV_proj
.\build\cmake\test_curved_path_following.exe
```

如果用 `ctest` 运行，CSV 可能生成在：

```text
build/cmake/test_outputs/
```

### 9.6 `py` 命令无法运行

检查 Python Launcher：

```powershell
py --version
```

如果不可用，需要安装 Python 或修复环境变量。

## 10. 文献和方案文档

主要文档位于：

```text
MdReader/文献调研/
```

其中包括：

```text
AUV单体控制算法项目文献综述.md
AUV单体控制算法核心文献精读表.md
Fossen海洋航行器动力学建模笔记.md
AUV单体控制算法仿真项目方案_修改版.md
AUV单体控制算法可操作可视化APP执行方案.md
```
