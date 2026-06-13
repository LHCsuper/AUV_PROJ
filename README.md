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

## 9. 推荐运行顺序

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

## 10. 常见问题

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
