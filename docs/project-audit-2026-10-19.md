# UWScan2 项目检查报告

检查日期：2026-10-19（Asia/Shanghai）  
检查基线：`0bc3ca2`（Initial commit）  
检查方式：跨模块静态审查、CMake 配置、独立 C++ 数值复现、Python 最小用例、资源文件校验。

## 结论

定位到 18 项需要处理的问题：10 项 P1、8 项 P2。优先修复崩溃、数据丢失和重建结果错误，再处理构建交付与界面阻塞问题。

本次没有修改业务代码。检查过程中生成的程序、Python 依赖和最小复现脚本均在被 Git 忽略的 `out/` 下；本报告是唯一新增的受版本管理范围覆盖的文件。

P1 表示应优先修复的崩溃、数据完整性、数值正确性或构建阻断问题；P2 表示影响特定功能或可靠性的明确缺陷。静态确认的执行路径与实际运行复现分开标明，未将设备行为推断当作实机测试结果。

## 覆盖范围与限制

- 检查了项目结构、构建及部署配置、相机封装、扫描状态机、重建线程、关联数据保存、点云编辑与导入、TCP/UDP 协议、Python 封装及算法、Qt 资源与主要界面调用路径。
- 对主窗口中的大量测量/绘图辅助代码进行了重点路径审查；第三方 SARibbon、MIPP、Eigen/Ceres 源码及二进制库未作为项目自研代码逐行审计。
- 主程序未完成编译或启动：当前机器缺少项目要求的 Qt5 开发包，配置停在 `CMakeLists.txt:50`。因此没有进行完整 GUI、Pylon 相机、真实电机和网络设备联调。
- 未发现项目自有的自动化测试或 CI 配置。第三方库中的测试不计入项目测试覆盖。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| 初始 Git 工作区 | 干净 |
| 主程序 CMake 配置 | 已识别 MSVC 19.51，随后因缺少 Qt5Config.cmake 中断 |
| 独立 C++ 求交复现程序 | Release 编译、运行成功；复现错误交点及非有限坐标 |
| Python 语法解析 | 9 个文件通过 |
| Qt 资源引用 | 383 条文件引用均存在 |
| Qt UI XML | 5 个文件均可解析 |
| Python 灰度中心线冒烟验证 | 合成条纹提取到 112 个有限坐标点 |
| Python 四次曲面、标定转换 | 复现系数错位、无交点仍返回坐标、尺寸序列读取异常、标定范围丢失 |
| 部署依赖完整性 | POST_BUILD 引用的 5 个 DLL 不存在 |
| 独立算法 DLL 交付 | 未找到 DLL，也未找到独立构建工程 |

Python 验证使用本机 NumPy 2.2.6，以及单独安装到 `out/audit-python-deps/` 的 OpenCV Python 5.0.0。主程序配置要求 OpenCV 4.5.0；Python 验证不能代替该版本的完整 C++ 集成测试。C++ 数值用例直接从项目源文件提取完整求交函数，并使用仓库内 Eigen 编译，没有改写被测函数。

## 问题清单

### A01 · P1 · Scanner 状态字段未初始化即被读取

位置：`include/scanner/scanner.h:329`、`include/scanner/scanner.h:336`、`src/scanner/scanner.cpp:159`、`src/scanner/scanner.cpp:944`、`src/scanner/scanner.cpp:1010`。

构造函数没有初始化 `isSingleScan_`、`pendingBeforeScanSnapshot_`、`pendingAfterScanSnapshot_` 等控制字段。`connectDevices()` 只初始化帧计数；正常连接后随即开始预览，而第一帧就读取这些布尔字段。

结果可能是预览帧误入场景快照、单帧重建或扫描分支。这是未初始化读取造成的未定义行为，不能依赖某次运行时内存恰好为零。

建议：在成员声明或构造函数中初始化全部基本类型字段，并在连接、断开、取消时恢复一致的状态。验证方式：静态调用路径确认。

### A02 · P1 · 横向多阈值算法在无激光图像上越界

位置：`src/scanner/ImgProcess.cpp:2774`、`src/scanner/ImgProcess.cpp:2784`。

`bruteLaserCenter_heng()` 可以返回空集合，随后过滤也可能移除所有候选点。函数未检查 `x.empty()` 就访问 `x[0]`，之后还访问 `y[0]`。

全黑图像、无激光或有效区域内没有条纹时，选择横向多阈值方法即可进入此路径；重建与预览都调用该算法。访问违规也不能由外围 `std::exception` 捕获可靠处理。

建议：空候选立即返回空结果，并检查后续固定长度结果缓冲区和索引边界。验证方式：静态数据流确认；未执行带完整 OpenCV C++ 依赖的该函数。

### A03 · P1 · 曲面求交返回不存在的交点和 NaN/Inf

位置：`src/scanner/QuadricSurfaceFitting.cpp:495`、`src/scanner/QuadricSurfaceFitting.cpp:516`、`src/scanner/QuadricSurfaceFitting2.cpp:458`、`src/scanner/reconstructor.cpp:518`、`python/uwscan3d/surface.py:143`。

二次曲面函数直接使用复根实部，没有验证虚部；退化为一次方程时也没有检查分母。四次曲面在没有实根时主动选择一个复根的实部。重建端只剔除接近零向量的结果，NaN/Inf 会通过该判断。Python 四次求交使用采样残差最小值，也没有要求残差接近零。

独立运行复现：

| 输入 | 实际结果 | 问题 |
| --- | --- | --- |
| 单位球，射线起点 `(2,0,0)`、方向 `(0,0,1)` | C++ 返回 `(2,0,0)`，曲面残差 3 | 射线与球不相交 |
| 曲面 `x²=1`，同一射线 | C++ 返回 NaN、NaN、-Inf | 退化方程除零 |
| 曲面 `z⁴+1=0` | C++ 返回 `z≈-0.707107`，残差 1.25 | 实数范围根本无解 |
| 曲面 `z⁴+1=0` | Python 返回 `z≈-0.010753`，残差约 1 | 采样最小值被误当成交点 |

建议：显式处理降阶、无实根与平行情况；检查根的虚部、有效区间、射线方向、曲面残差和全部输出坐标的有限性；以明确的成功状态传递结果。

### A04 · P1 · 停止扫描没有取消待执行的启动回调

位置：`src/scanner/scanner.cpp:561`、`src/scanner/scanner.cpp:802`、`src/scanner/scanner.cpp:976`。

开始扫描后，完整初始化和电机移动延后 100ms 执行。该回调没有检查扫描是否已取消；收到扫描前场景图后，另一个 50ms 回调还会无条件把 `softwareScanActive_` 设回 true。

因此在启动后立即停止，或在场景快照与扫描切换之间停止，旧回调仍可能重新移动电机或启动扫描。仅清空当前标志不能使已排队的回调失效。

建议：为每次扫描分配递增代号，取消时使旧代号失效，所有延迟回调执行前校验；也可使用可停止的成员 QTimer。验证方式：静态事件时序确认。

### A05 · P1 · 退出时未排空重建与保存队列

位置：`src/scanner/reconstructor.cpp:120`、`src/scanner/reconstructor.cpp:136`、`src/scanner/associatedpointsmanager.cpp:312`、`src/scanner/associatedpointsmanager.cpp:448`。

停止重建会令工作循环直接退出，队列中的未处理帧被放弃。保存线程同样在 `isRunning_` 变为 false 后直接退出；更严重的是，`stop()` 在工作线程已经 join 后才调用 `cancelAllSessions()`，后者新增的最终保存任务已没有消费者。

扫描后立即退出、保存速度慢于采集速度或仍存在未完成会话时，最后一批数据可能只有临时文件或完全未落盘。

建议：先停止生产新任务，再处理完已接收帧和最终保存任务，最后通知线程退出并 join。验证方式：静态线程/队列生命周期确认。

### A06 · P1 · 最终文件写入失败仍删除原始临时数据

位置：`src/scanner/associatedpointsmanager.cpp:510`、`src/scanner/associatedpointsmanager.cpp:531`、`src/scanner/associatedpointsmanager.cpp:552`、`src/scanner/associatedpointsmanager.cpp:616`。

最终保存函数没有检查输入/输出文件是否打开成功、写入结果或关闭结果，就无条件删除临时文件。权限问题、磁盘写入失败和路径编码问题都可能造成原始数据丢失。增量保存仅检查 `.bad()`，而文件打开失败通常设置的是 `failbit`。

独立 C++ 用例验证了文件打开失败时 `is_open=0, fail=1, bad=0`，现有 `.bad()` 判断无法覆盖该失败。

建议：检查打开和全部写入状态，使用临时目标及原子提交，仅在成功提交后移除源临时文件；失败时保留可恢复数据。

### A07 · P1 · 点云导入的单位换算方向相反

位置：`src/pointcloud/importoptiondialog.cpp:15`、`src/pointcloud/importoptiondialog.cpp:58`、`include/pointcloud/pc.hpp:847`。

选项依次是毫米、厘米、分米、米，项目的滤波与测量按毫米工作，但实际比例为 `pow(0.1, unit_index)`，即 `1/0.1/0.01/0.001`。

厘米到毫米应乘 10，米到毫米应乘 1000。当前导入 1 米会变成 0.001 毫米，与正确结果相差一百万倍，后续直径、缺陷尺寸及滤波尺度都会错误。

建议：统一内部长度单位为毫米，使用明确的 `1/10/100/1000` 映射。验证方式：静态数值与界面单位对照。

### A08 · P1 · 发布步骤必需的 5 个 DLL 缺失且被忽略

位置：`src/CMakeLists.txt:64`、`src/CMakeLists.txt:71`、`.gitignore:1`。

部署清单引用的以下文件在当前检出中不存在：

- `lib/ceresLib/ceres/bin/ceres.dll`
- `lib/ceresLib/gflags/bin/gflags.dll`
- `lib/ceresLib/glog/bin/glog.dll`
- `lib/SARibbon/bin_qt5.15.2_MSVC_x64/bin/QWKCore.dll`
- `lib/SARibbon/bin_qt5.15.2_MSVC_x64/bin/QWKWidgets.dll`

`git check-ignore -v` 确认，顶层 `bin/` 规则也匹配这些第三方子目录。即使补齐 Qt/PCL/Pylon 开发环境，POST_BUILD 的 `copy_if_different` 仍会因缺少源文件失败，发布包也不完整。

建议：将忽略规则限定为项目根输出 `/bin/`，并提供明确的依赖获取/校验方式；不应只修改忽略规则而不补齐真实文件。验证方式：实际文件存在性与 Git 忽略规则验证，未执行完整发布构建。

### A09 · P2 · 已删除的点会在滤波或合并后恢复

位置：`include/pointcloud/pc.hpp:357`、`include/pointcloud/pc.hpp:582`、`include/pointcloud/pc.hpp:609`、`include/pointcloud/pc.hpp:675`。

删除点和只保留选中点仅修改 `all_indices_`，原始 `cloud_` 仍保留全部点。三个滤波函数却直接处理原始点云，并重新建立完整索引；合并也直接拼接原始点云。

因此已删除点会再次参与滤波、显示和导出。多点云合并还把逻辑点数与原始点数混用，Debug 构建可能触发断言，Release 构建则可能恢复旧点并漏掉尾部新点。

建议：滤波及合并前物化当前有效索引对应的点集，确保所有算法使用相同的数据视图。验证方式：静态索引与底层存储追踪。

### A10 · P2 · Python 四次曲面系数约定不一致

位置：`python/uwscan3d/surface.py:86`、`python/uwscan3d/surface.py:138`、`src/scanner/QuadricSurfaceFitting2.cpp:437`。

拟合时 `xy²`、`y³` 的系数写入位置不符合 C++ 的 35 项约定；Python 求值还把 `xy` 项解释为 `y²`，并混淆若干其他交叉项。

运行复现：

- 一个正交构造的三次曲面，拟合函数报告平均误差约 `7.18e-17`，导出的 35 项系数按 C++ 约定求值后实际平均绝对残差约 `0.06617`。
- 曲面 `xy-z=0`，固定 `x=2,y=3` 的射线应在 `z=6` 相交，Python 返回 `z≈9.01271`。

建议：定义一份明确的项次表，统一拟合映射、Python 求值及 DLL 约定；用带交叉项的曲面进行往返校验。

### A11 · P2 · Python 无法读取序列形式的 picSize

位置：`python/uwscan3d/calibration.py:14`、`python/uwscan3d/calibration.py:57`。

C++ 把 `picSize` 读入 `cv::Size`，而 Python 无条件调用 `FileNode.mat()`。当标定文件采用常见的 `picSize: [640, 480]` 序列形式时，它不是 OpenCV 矩阵节点，读取立即抛出异常。

已用 OpenCV FileStorage 写出的尺寸序列复现 `isMap()` 断言异常。

建议：区分序列节点和矩阵节点，支持两种合法尺寸表示，并验证宽高为两个正整数。

### A12 · P2 · 从 DLL 转换标定数据时丢失扫描范围

位置：`python/uwscan3d/calibration.py:115`、`python/uwscan3d/calibration.py:130`。

`CalibrationData.from_dll()` 把 `surf_begin_pos`、`surf_end_pos` 固定为 0，把 `surf_step` 固定为 1。第 130 行还保留了永远不执行的占位条件表达式。

用三个曲面槽位的替身对象验证，转换后的三个不同位置全部映射到槽位 0，而不是预期的 0、1、2。纯 Python 重建因此可能静默使用错误的激光曲面。

建议：在 C API 中提供真实范围与步长的读取能力，转换时完整保留元数据；无法获得时明确拒绝转换。该用例验证的是转换逻辑，未加载真实 DLL。

### A13 · P2 · 后台重建持有队列锁，阻塞主线程

位置：`src/scanner/reconstructor.cpp:130`、`src/scanner/reconstructor.cpp:144`、`src/scanner/reconstructor.cpp:278`、`src/mainwindow/uwscanmainwindow.cpp:5092`。

工作线程从队列取帧后仍持有 `m_queueMutex`，直到中心线提取、去畸变、逐点求交和关联数据提交结束。Scanner 和 Reconstructor 的 QObject 均在主线程，入队槽直接执行并等待同一把锁。

连续帧或较慢重建会让 UI 等待整帧计算，影响停止按钮响应和设备反馈处理。

建议：只在检查队列和弹出元素时持锁，在锁外完成图像和数值计算。验证方式：静态锁作用域与信号连接确认，未进行 GUI 性能实测。

### A14 · P1 · 远程保存图像的异常可逃逸 Qt 槽

位置：`src/server/commandserver.cpp:289`、`src/mainwindow/uwscanmainwindow.cpp:5702`、`src/mainwindow/uwscanmainwindow.cpp:5710`。

`SAVE_CURRENT_IMAGE` 的文件名直接来自远程请求。处理槽调用 `cv::imwrite()`，没有格式白名单或异常处理。带不支持扩展名的路径会抛出 `cv::Exception`，而不是简单返回 false。

已独立复现 `.unsupported` 扩展名触发异常。结合未捕获的槽函数路径，在设备已连接且有图像时，错误请求可造成应用异常终止；本次没有启动真实网络服务发送该请求。

建议：验证格式、约束保存位置，并捕获编码和文件系统异常，通过协议返回失败结果。

### A15 · P2 · 到位容差大于常用扫描步长

位置：`src/scanner/scanner.cpp:779`、`src/scanner/scanner.cpp:783`。

`shouldCaptureAtCurrentFeedback()` 固定允许 50 个百分之一度，即 ±0.5°。当步长为 0.1° 时，上一帧位置已经落在下一目标的容差内。例如目标 42.1°、反馈 42.0°，函数会判定可以停止并拍照。

这会使小步长扫描过早拍照、相邻采样位置重复或分布不符合设置。

建议：根据步长和设备反馈分辨率设置更严格容差，并结合方向、跨越检测和稳定状态判断到位。验证方式：静态条件代入；实际精度仍需设备联调。

### A16 · P1 · 标定加载未验证关键参数就标记成功

位置：`src/scanner/reconstructor.cpp:175`、`src/scanner/reconstructor.cpp:196`、`src/scanner/reconstructor.cpp:251`、`src/scanner/reconstructor.cpp:406`、`src/scanner/uwscan3d_algorithms.cpp:925`。

桌面重建器使用未初始化的临时折射率、距离和法向量；对应节点缺失时，这些值仍覆盖构造函数设置的默认值。读取成功后也没有验证相机矩阵尺寸、曲面系数长度和 `surfStep != 0` 就设置 `m_isInitialized=true`。

DLL 标定加载虽有部分默认值，也未拒绝零步长，随后重建直接用该步长作整数除法。一个结构不完整或零步长的标定文件可能造成无效坐标、Eigen 断言或除零崩溃。

建议：以明确默认值初始化临时对象，完整校验后原子替换现有标定，失败时给出具体节点错误。验证方式：静态输入验证与使用路径确认。

### A17 · P2 · 电机断线后的取消和界面恢复逻辑不可达

位置：`src/scanner/scanner.cpp:220`、`src/mainwindow/uwscanmainwindow.cpp:5129`。

两个错误处理 lambda 在记录消息后立即 `return`，后面的断线识别、停止通知和界面恢复逻辑永远不会执行。扫描等待目标角度反馈时，网络断开会停止电机查询，但这一路没有到位超时，扫描可能一直停在等待状态。

建议：删除不可达分支或实现统一的断线状态迁移，取消扫描会话、使延迟回调失效并恢复控件。验证方式：静态控制流确认。

### A18 · P2 · 独立算法 DLL 缺少可复现构建入口

位置：`README.md:84`、`scripts/build_uwscan3d_algorithms.bat:7`、`scripts/build_uwscan3d_algorithms.bat:13`、`src/CMakeLists.txt:35`。

README 和脚本要求 `dll_project/CMakeLists.txt`，实际不存在；仓库主工程仅把算法实现加入静态模块，没有生成 `UWScan3DAlgorithms.dll` 的 SHARED 目标。Python `discover()` 所查找的 DLL 也不存在。

辅助脚本和示例还固定使用另一台机器的 `C:\Users\AAA\Desktop\UWScan2` 路径。本项目现有结构文档已承认独立工程缺失，但 README 仍将它描述成推荐的唯一入口，用户无法按说明从当前检出得到 DLL。

建议：恢复仅依赖算法所需库的独立构建目标，使用脚本位置推导项目根目录，并统一 README、示例和结构文档。

## 本地复现入口

本次检查保留了以下被忽略的本地文件，供当前工作区继续核验：

- `out/audit_checks.py`：Python 语法、资源、标定和曲面数值检查。
- `out/audit-native/CMakeLists.txt`：从原始源码生成独立求交验证目标。
- `out/audit-native/native_checks.cpp`：无实根、平行情况及文件流错误状态用例。
- `out/audit-main/`：未完成的主程序 CMake 配置记录。

在项目根目录运行：

```powershell
python out/audit_checks.py
& 'D:\Software\VS2026\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' -S out/audit-native -B out/audit-native/build -G 'Visual Studio 18 2026' -A x64
& 'D:\Software\VS2026\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build out/audit-native/build --config Release
& '.\out\audit-native\build\Release\native_checks.exe'
```

这些脚本是本次检查的本地复现材料，不是项目已有的回归测试套件，也不会随当前 Git 忽略规则提交。

## 建议修复顺序

1. A01、A02、A04、A14、A16：消除未初始化读取、越界、停止后重启及未处理异常。
2. A05、A06：保证退出、取消和写入失败时数据可恢复。
3. A03、A07、A10、A12、A15：修正数值正确性、单位和标定映射。
4. A08、A11、A18：补齐可复现构建与 Python 标定读取。
5. A09、A13、A17：统一点云有效数据视图，缩小锁范围，恢复断线状态迁移。

修复后应补充最小回归用例，并在 Qt5/OpenCV 4.5/PCL/Pylon 环境完成完整构建、GUI 操作和真实设备联调。
