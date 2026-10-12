# UWScan2 模块封装记录

更新日期：2026-09-14

主程序通过 `src/CMakeLists.txt` 拆分为静态库模块，并为每个模块提供
`UWScan::` 命名空间别名。最终的 `UWScan` 可执行文件只负责应用入口和
模块组装。

## 模块总览

| CMake 目标 | 别名 | 职责 | 主要依赖 |
| --- | --- | --- | --- |
| `uwscan_settings` | `UWScan::uwscan_settings` | 设置持久化和设置界面。 | Qt Core、Qt Widgets |
| `uwscan_widgets` | `UWScan::uwscan_widgets` | 图像显示、消息面板和状态栏。 | Qt、OpenCV、Pylon 头文件 |
| `uwscan_pointcloud` | `UWScan::uwscan_pointcloud` | 点云数据、可视化、导入和滤波。 | Qt、PCL/VTK |
| `uwscan_scanner` | `UWScan::uwscan_scanner` | 相机采集、图像处理、电机、扫描、标定和重建。 | settings、widgets、pointcloud、Qt、OpenCV、PCL、Ceres、UW3dLib |
| `uwscan_server` | `UWScan::uwscan_server` | TCP 命令服务和命令对话框。 | Qt Network、scanner |
| `uwscan_app_shell` | `UWScan::uwscan_app_shell` | Ribbon、主窗口和应用装配。 | 上述模块、Qt、PCL、OpenCV、SARibbon |
| `UWScan` | 无 | Windows 桌面程序。 | `uwscan_app_shell` |

## 源码归属

| 模块 | 实现与头文件 |
| --- | --- |
| settings | `src/setting/`、`include/setting/` |
| widgets | `src/imagewidget.cpp`、`src/messagestation.cpp`、`src/infofullstatusbar.cpp` 及对应头文件 |
| pointcloud | `src/pointcloud/`、`include/pointcloud/` |
| scanner | `src/scanner/`、`include/scanner/` |
| server | `src/server/`、`include/server/` |
| app shell | `src/mainwindow/`、`include/mainwindow/` |

Qt 界面文件位于 `resources/ui/`。`uwscan_settings`、`uwscan_widgets` 和
`uwscan_scanner` 会将对应 `.ui` 文件作为源文件交给 AUTOUIC 处理。
图标和图片通过 `resources/images/Icon.qrc` 与 `resources/QRibbon.qrc` 编译进
最终可执行程序。

## 依赖方向

```text
settings ─┐
widgets ──┼──> scanner ───> server ───> app_shell ───> UWScan
pointcloud┘                       │
                                  └───────────────> app_shell
```

`app_shell` 同时直接链接 settings、widgets、pointcloud、scanner 和 server，
用于创建完整桌面交互界面。新代码应落在职责最接近的模块中，避免把设备控制、
点云算法或协议处理直接写入主窗口。

## 扩展原则

### 新增点云能力

将点云数据结构、渲染、导入或滤波能力放入 `pointcloud` 模块；通过主窗口中的
Action 或信号槽接入界面。若需要调用扫描结果，可由 `scanner` 将结果以明确的
数据接口传递给点云模块。

### 新增设备或采集能力

相机、投影仪、电机、串口、Modbus 和重建流程放入 `scanner` 模块。需要新增
第三方 SDK 时，先在顶层 CMake 和 `cmake/` 中建立可配置的依赖查找方式，再将
库链接限制在实际使用的目标上。

### 新增远程命令

协议结构、命令解析和 TCP 生命周期放入 `server` 模块。命令层调用 scanner 或
pointcloud 的公开接口，不应直接操作主窗口内部控件。

### 新增界面

界面编排和用户触发入口放入 `app_shell`；可复用的小型显示控件优先归入
`widgets`。新增 `.ui` 文件后，将其置于 `resources/ui/`，并在对应模块的
CMake 源文件列表中显式引用。

## 独立算法接口

`src/scanner/uwscan3d_algorithms.h` 和
`src/scanner/uwscan3d_algorithms.cpp` 提供了面向跨语言调用的算法入口；
Python 侧由 `python/uwscan3d/dll.py` 使用 `ctypes` 包装。

当前 `dll_project/` 目录没有独立 `CMakeLists.txt`，因此独立
`UWScan3DAlgorithms` DLL 工程不能视为可复现的构建入口。恢复该工程时，应：

1. 只链接算法所需的 OpenCV、Ceres 和必要头文件，避免引入 Qt UI 和设备控制。
2. 将 DLL、导入库和调试符号输出到明确的发布目录。
3. 为 C/C++ 和 Python 调用提供最小示例。
4. 将构建命令和运行时 DLL 依赖更新到 `README.md` 与本文件。

## 模块检查清单

- 新增 `.cpp`、`.h`、`.ui` 或 `.qrc` 文件后，确认相应 CMake 目标已引用。
- 新增公开能力时，在 `include/` 中提供稳定接口，避免外部模块依赖内部实现。
- 避免形成循环依赖；需要共享的数据类型时，抽取轻量公共接口而非互相包含模块。
- 改动模块边界、链接库或部署 DLL 时，同步更新 `docs/project-structure.md`。
