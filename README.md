# AUV_PROJ

本项目用于搭建一个基于 C++ 的 AUV 单体控制算法仿真框架。当前版本已经完成水平面 3-DOF 简化模型、PID/LOS 路径跟随、恒定海流、圆形路径和 S 形路径等基础模块。

当前展示部分主要通过测试程序生成 CSV 数据，后续会继续补充 Python/Plotly 绘图脚本和更完整的展示入口。

## 1. 项目目录

```text
AUV_proj/
  apps/auv_sim/        # 当前最小程序入口
  include/auv/         # C++ 头文件
  src/                 # C++ 源文件
  tests/               # 验证程序和仿真场景
  tools/               # 本地工具依赖，已忽略提交
  build/cmake/         # CMake 构建目录，已忽略提交
  test_outputs/        # 手动运行测试程序时生成的 CSV，已忽略提交
  MdReader/文献调研/   # 文献调研和方案文档
```

## 2. 运行位置

所有命令都在 PowerShell 中执行，并且先进入项目根目录：

```powershell
cd E:\AUV_proj
```

如果你的 PowerShell 当前不在 E 盘，也可以直接使用：

```powershell
Set-Location E:\AUV_proj
```

## 3. 配置工程

第一次运行、清理过 `build/`、或者修改了 `CMakeLists.txt` 后，需要重新配置：

```powershell
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe -S . -B build\cmake -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER=g++
```

说明：

- CMake 使用项目内的便携版：`tools/cmake/cmake-4.3.3-windows-x86_64`
- Eigen 使用项目内的本地版本：`tools/eigen/eigen-3.4.1`
- 编译器使用本机已经可用的 `g++`

## 4. 编译项目

在 `E:\AUV_proj` 下执行：

```powershell
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
```

编译成功后，可执行文件会生成在：

```text
build/cmake/
```

## 5. 运行最小入口程序

当前 `auv_sim` 主要用于确认程序能正常启动、Eigen 能正常链接：

```powershell
.\build\cmake\auv_sim.exe
```

正常情况下会看到类似输出：

```text
AUV simulation started
Eigen check: current norm = 0.316228
```

注意：当前 `auv_sim` 还不是完整演示入口，它不会生成路径跟随结果。要生成仿真 CSV，请运行下面的测试程序。

## 6. 运行全部验证

在 `E:\AUV_proj` 下执行：

```powershell
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\ctest.exe --test-dir build\cmake --output-on-failure
```

如果全部正常，会看到类似：

```text
100% tests passed, 0 tests failed
```

`ctest` 主要用于确认所有模块没有坏掉。它也会运行部分会输出 CSV 的测试，但输出位置可能在 `build/cmake/test_outputs/`。

## 7. 生成可查看的仿真结果

推荐直接从项目根目录手动运行下面几个测试程序。这样 CSV 会稳定生成在：

```text
E:\AUV_proj\test_outputs\
```

### 航向 PID 阶跃响应

```powershell
.\build\cmake\test_heading_pid.exe
```

生成：

```text
test_outputs/heading_step_response.csv
```

### 纵向速度控制响应

```powershell
.\build\cmake\test_speed_controller.exe
```

生成：

```text
test_outputs/speed_step_response.csv
```

### 直线路径 LOS 跟随

```powershell
.\build\cmake\test_path_following.exe
```

生成：

```text
test_outputs/straight_line_path_following.csv
```

### 圆形路径和 S 形路径跟随

```powershell
.\build\cmake\test_curved_path_following.exe
```

生成：

```text
test_outputs/circle_path_following.csv
test_outputs/s_curve_path_following.csv
```

该程序还会在终端输出关键误差指标，例如初始横向误差、最终横向误差、稳定后的最大误差和速度误差。

## 8. 查看运行结果

当前结果以 CSV 为主，可以用以下方式查看：

1. 在 VS Code 中打开 `test_outputs/*.csv`
2. 用 Excel 打开 CSV
3. 用 Python、MATLAB 或 Origin 读取 CSV 后画图

路径跟随 CSV 的主要字段包括：

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

建议优先查看这几组曲线：

- `x, y`：AUV 实际轨迹
- `x_ref, y_ref`：参考路径或最近参考点
- `cross_track_error`：横向路径误差
- `heading_error`：航向误差
- `X_cmd, N_cmd`：纵向推力和偏航力矩控制输入

## 9. 生成可视化 HTML 报告

当前项目已经提供第一版可视化脚本，可以从 CSV 数据生成浏览器可打开的 HTML 报告。

先确认已经生成路径跟随 CSV：

```powershell
cd E:\AUV_proj
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe
```

然后一键生成全部可视化报告：

```powershell
py scripts\batch_generate_reports.py
```

生成结果位于：

```text
results/visualization/index.html
results/visualization/straight_path_report.html
results/visualization/circle_path_report.html
results/visualization/s_curve_path_report.html
```

推荐直接打开总入口：

```text
E:\AUV_proj\results\visualization\index.html
```

每个报告包含：

- 场景信息表
- summary 指标表
- AUV 实际轨迹与参考路径
- 横向路径误差
- 航向误差和航向角
- 纵向速度
- 控制输入及限幅线
- 每幅图的简短含义说明
- 横纵坐标名称、单位、刻度数字和网格线

当前可视化脚本使用 Python 标准库生成离线 HTML，不依赖 pandas/Plotly，方便在当前环境直接运行。后续如果安装 Plotly，可以继续升级为 Plotly 交互图表。

轨迹图坐标含义：

```text
横坐标：x, inertial-frame position [m]
纵坐标：y, inertial-frame position [m]
```

时间序列图坐标含义：

```text
横坐标：time, simulation time [s]
纵坐标：对应物理量，例如横向误差、航向误差、速度、推力或偏航力矩
```

## 10. 运行可操作可视化 APP

当前项目已经开始提供面向交付展示的可操作 Web APP，位置在：

```text
web/
```

APP 当前支持：

- 工程交付首页
- 已完成模块展示
- 控制算法流程展示
- 直线、圆形、S 形场景切换
- AUV 2D 动画回放
- 播放、暂停、重置、时间滑条、倍速控制
- 实时状态面板
- 轨迹图、误差图、速度图和控制输入图

### 10.1 生成 APP 数据

如果 C++ 仿真结果发生变化，先重新生成 CSV：

```powershell
cd E:\AUV_proj
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe
```

再将 CSV 转换为 Web APP 使用的 JSON：

```powershell
py scripts\convert_simulation_csv_to_json.py --input test_outputs\straight_line_path_following.csv --output web\public\data\straight.json --scenario straight
py scripts\convert_simulation_csv_to_json.py --input test_outputs\circle_path_following.csv --output web\public\data\circle.json --scenario circle
py scripts\convert_simulation_csv_to_json.py --input test_outputs\s_curve_path_following.csv --output web\public\data\s_curve.json --scenario s_curve
```

### 10.2 开发模式运行

在本机调试 APP 时执行：

```powershell
cd E:\AUV_proj\web
npm run dev -- --port 5173
```

然后打开：

```text
http://127.0.0.1:5173
```

### 10.3 离线打包

生成离线静态文件：

```powershell
cd E:\AUV_proj\web
npm run build
```

生成目录：

```text
web/dist/
```

其中包含：

```text
web/dist/index.html
web/dist/assets/
web/dist/data/
```

`web/dist/data/` 中包含三个演示场景的数据：

```text
straight.json
circle.json
s_curve.json
```

### 10.4 断网环境展示

断网环境下，推荐用本机静态服务打开：

```powershell
cd E:\AUV_proj\web\dist
py -m http.server 8000
```

然后打开：

```text
http://127.0.0.1:8000
```

这不需要访问外网，所有 JS、CSS 和数据都来自本地 `web/dist/`。

如果后续要给甲方拷贝演示，只需要带上：

```text
web/dist/
```

并在演示电脑上用本地服务打开即可。后续也可以将该目录封装为 Electron 桌面程序，实现双击运行。

### 10.5 封装 Windows 桌面程序

当前项目已经提供 Electron 桌面封装配置。先确保 APP 可以正常构建：

```powershell
cd E:\AUV_proj\web
npm run build
```

然后生成 Windows 目录版桌面程序：

```powershell
npm run electron:pack
```

生成结果位于：

```text
web/release/win-unpacked/
```

可执行程序为：

```text
web/release/win-unpacked/AUV Visualization.exe
```

这个目录版程序不需要联网，内部已经包含打包后的前端页面和演示数据。给甲方演示时，需要整体拷贝 `win-unpacked/` 文件夹，不要只单独拷贝 `.exe`。

Electron 相关依赖安装在：

```text
web/node_modules/
```

Electron 下载缓存位于：

```text
tools/electron-cache/
```

electron-builder 缓存位于：

```text
tools/electron-builder-cache/
```

这些目录都在项目 E 盘目录下，不需要安装到 C 盘，也不需要全局安装。

## 11. 推荐运行顺序

如果只是想快速确认项目是否正常：

```powershell
cd E:\AUV_proj
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\ctest.exe --test-dir build\cmake --output-on-failure
```

如果想生成可看的路径跟随数据：

```powershell
cd E:\AUV_proj
.\build\cmake\test_path_following.exe
.\build\cmake\test_curved_path_following.exe
```

然后查看：

```text
test_outputs/straight_line_path_following.csv
test_outputs/circle_path_following.csv
test_outputs/s_curve_path_following.csv
```

如果想生成 HTML 展示报告：

```powershell
py scripts\batch_generate_reports.py
```

然后打开：

```text
results/visualization/index.html
```

如果想运行可操作 Web APP：

```powershell
cd E:\AUV_proj\web
npm run dev -- --port 5173
```

如果想离线打包可操作 Web APP：

```powershell
cd E:\AUV_proj\web
npm run build
cd dist
py -m http.server 8000
```

然后打开：

```text
http://127.0.0.1:8000
```

如果想生成桌面版程序：

```powershell
cd E:\AUV_proj\web
npm run electron:pack
```

然后运行：

```text
web/release/win-unpacked/AUV Visualization.exe
```

## 12. 常见问题

### 找不到 `cmake.exe`

确认当前目录是：

```text
E:\AUV_proj
```

并检查这个文件是否存在：

```text
tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe
```

### 找不到 `g++`

说明当前 PowerShell 环境里无法直接调用 MinGW 的 `g++`。可以先检查：

```powershell
g++ --version
```

如果失败，需要把 MinGW 的 `bin` 目录加入环境变量，或者在 CMake 配置时填写完整的 `g++.exe` 路径。

### 没有看到 CSV

确认你是从 `E:\AUV_proj` 根目录运行测试程序，例如：

```powershell
.\build\cmake\test_curved_path_following.exe
```

如果用 `ctest` 运行，CSV 可能生成在：

```text
build/cmake/test_outputs/
```

### `python` 命令无法运行

当前环境中可以优先使用 Python Launcher：

```powershell
py --version
py scripts\batch_generate_reports.py
```

如果 `py` 也不可用，再检查本机 Python 安装和环境变量。

### 修改代码后结果没变化

修改 C++ 代码后需要重新编译：

```powershell
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
```

修改 `CMakeLists.txt` 或新增源文件后，建议先重新配置再编译：

```powershell
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe -S . -B build\cmake -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER=g++
.\tools\cmake\cmake-4.3.3-windows-x86_64\bin\cmake.exe --build build\cmake
```
