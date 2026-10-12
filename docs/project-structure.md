# UWScan2 项目结构说明

更新日期：2026-09-14

UWScan2 是一个基于 Qt 5、C++17、PCL、OpenCV 的 Windows 桌面扫描程序，
集成相机采集、设备通讯、图像处理、点云处理和三维重建能力。

## 顶层目录

| 路径 | 作用 | 是否影响运行 |
| --- | --- | --- |
| `src/` | C++ 实现文件；`src/main.cpp` 是程序入口。 | 是 |
| `include/` | 模块公开头文件和 MIPP 算法头文件。 | 是 |
| `resources/` | Qt 界面文件、图标、图片和资源清单。 | 是 |
| `cmake/` | Ceres、OpenCV、SARibbon、UW3dLib、LASTools 的 CMake 查找脚本。 | 构建时需要 |
| `lib/` | 项目随附的 Ceres、SARibbon/QWindowKit、UW3dLib 头文件、库和 DLL。 | 是 |
| `bin/` | 当前可直接运行的发布目录，包含 `UWScan.exe`、依赖 DLL 和 Qt 插件。 | 是 |
| `python/` | `uwscan3d` Python 包及其依赖清单。 | Python 调用时需要 |
| `examples/` | 调用独立算法 DLL 的 Python 示例。 | 否 |
| `scripts/` | 独立算法 DLL 的构建辅助脚本。 | 否 |
| `prompt/` | 点云系统的开发上下文说明。 | 否 |
| `dll_project/` | 独立算法 DLL 工程的预留目录；当前没有独立的 `CMakeLists.txt`。 | 主程序不需要 |
| `.agents/` | 本地智能体工作配置。 | 否 |
| `docs/` | 项目维护文档。 | 否 |

## 源码模块

| 路径 | 职责 |
| --- | --- |
| `src/mainwindow/`、`include/mainwindow/` | 主窗口、Ribbon 界面和状态栏。 |
| `src/scanner/`、`include/scanner/` | 相机、电机/Modbus、扫描流程、图像处理、标定和重建。 |
| `src/pointcloud/`、`include/pointcloud/` | 点云数据管理、显示、选择、导入和滤波。 |
| `src/server/`、`include/server/` | TCP 命令服务、命令对话框和通讯协议。 |
| `src/setting/`、`include/setting/` | 设置持久化和设置界面。 |
| `src/imagewidget.cpp` | 图像显示组件。 |
| `src/messagestation.cpp` | 消息面板组件。 |
| `src/infofullstatusbar.cpp` | 通用状态栏组件。 |

## 关键配置文件

| 文件 | 作用 |
| --- | --- |
| `CMakeLists.txt` | 顶层构建入口，配置 Qt、PCL、OpenCV、Ceres、UW3dLib、SARibbon 和 Pylon。 |
| `src/CMakeLists.txt` | 定义应用模块、链接关系和 Windows 运行时部署规则。 |
| `CMakePresets.json` | Visual Studio CMake 构建预设。 |
| `CMakeUserPresets.json` | 本机 Qt 开发预设；不应视为跨机器的唯一构建配置。 |
| `.gitignore` | 排除构建产物和本机开发文件。 |
| `autobuild.sh` | Visual Studio 的便捷构建脚本。 |
| `Doxyfile` | Doxygen API 文档生成配置。 |
| `README.md` | 项目入口说明和 Python/DLL 使用概览。 |

## 构建与运行产物

- 主程序名称为 `UWScan`，可执行文件输出至 `bin/UWScan.exe`。
- 静态库和中间库输出至 `build/lib/<配置>/`。
- Windows 构建完成后，CMake 会调用 `windeployqt`，并将项目随附的
  Ceres、UW3dLib、SARibbon 和 QWindowKit DLL 复制到 `bin/`。
- 本机还需要可用的 Qt 5.15.2、PCL 1.12.1、OpenCV 4.5.0 和 Basler pylon
  开发环境；具体路径由顶层 `CMakeLists.txt` 配置。

## 可安全清理的内容

以下内容不属于源码，也不影响已部署的 `bin/UWScan.exe` 直接运行：

| 路径 | 说明 |
| --- | --- |
| `.vs/` | Visual Studio 索引、IntelliSense 缓存和工作区元数据。 |
| `build/` | 主程序的 CMake 缓存、对象文件和中间库。 |
| `build-uwscan3d-algorithms/` | 独立算法 DLL 的历史构建缓存和对象文件。 |
| `python/**/__pycache__/` | Python 自动生成的字节码缓存。 |

不要删除 `bin/`、`lib/`、`resources/`、`src/` 或 `include/`；前四者分别承载
当前运行环境、构建依赖、界面资源和程序源代码。
