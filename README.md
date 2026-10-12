# UWScan2

- 原始 Qt/C++ 桌面程序
- 独立的三维算法 DLL 和 Python 调用层

项目目录、保留文件与可安全清理内容见：
[docs/project-structure.md](/C:/Users/AAA/Desktop/UWScan2/docs/project-structure.md)


## 目录

- Qt 主程序源码：
  [src](/C:/Users/AAA/Desktop/UWScan2/src)
- 公共头文件：
  [include](/C:/Users/AAA/Desktop/UWScan2/include)
- DLL 导出接口：
  [src/scanner/uwscan3d_algorithms.h](/C:/Users/AAA/Desktop/UWScan2/src/scanner/uwscan3d_algorithms.h:1)
- DLL 实现：
  [src/scanner/uwscan3d_algorithms.cpp](/C:/Users/AAA/Desktop/UWScan2/src/scanner/uwscan3d_algorithms.cpp:1)
- 独立 DLL 构建工程：
  [dll_project/CMakeLists.txt](/C:/Users/AAA/Desktop/UWScan2/dll_project/CMakeLists.txt:1)
- Python 包：
  [python/uwscan3d](/C:/Users/AAA/Desktop/UWScan2/python/uwscan3d)

## 当前已生成的 DLL

运行时库：

- [bin/UWScan3DAlgorithms.dll](/C:/Users/AAA/Desktop/UWScan2/bin/UWScan3DAlgorithms.dll)

给 C/C++ 工程链接的导入库：

- [lib/UWScan3DAlgorithms.lib](/C:/Users/AAA/Desktop/UWScan2/lib/UWScan3DAlgorithms.lib)

说明：

- `.dll` 给 Python `ctypes` 或其他运行时加载使用
- `.lib` 给 C/C++ 工程链接使用
- `.pdb` 和 `.exp` 不属于运行必需文件，已经从保留清单里排除

## DLL 能做什么

当前 DLL 暴露的是纯算法接口，不带 Qt UI，不带 Basler SDK 设备控制。已经封装的能力包括：

- 激光中心线提取
- 水下折射射线计算
- 二次曲面拟合
- 四次曲面拟合
- 二次曲面求交
- 四次曲面求交

导出函数声明见：
[src/scanner/uwscan3d_algorithms.h](/C:/Users/AAA/Desktop/UWScan2/src/scanner/uwscan3d_algorithms.h:1)

## Python 包说明

`python/uwscan3d` 目前包含这些模块：

- `basler.py`
  Basler 相机参数模型
- `calibration.py`
  OpenCV YAML 标定文件读取
- `image_processing.py`
  灰度中心法、强度图、合成图
- `underwater.py`
  折射模型
- `surface.py`
  二次/四次曲面拟合与求交的 Python 版本
- `reconstruction.py`
  单帧三维重建流程
- `dll.py`
  `UWScan3DAlgorithms.dll` 的 `ctypes` 封装

依赖文件：
[python/requirements.txt](/C:/Users/AAA/Desktop/UWScan2/python/requirements.txt:1)

最少需要：

- `numpy`
- `opencv-python`

## 如何重新编译 DLL

不要从 Qt 主程序的 `src/CMakeLists.txt` 里再追加 DLL target。当前推荐的唯一 DLL 构建入口是：

- [dll_project/CMakeLists.txt](/C:/Users/AAA/Desktop/UWScan2/dll_project/CMakeLists.txt:1)

### 依赖

- Visual Studio 2026 Developer Command Prompt
- CMake
- Ninja
- OpenCV 4.5.0
- 仓库自带 `lib/ceresLib`

本机当前构建使用的路径是：

- OpenCV:
  `D:\Program Files\opencv_450_vs2019`
- Visual Studio:
  `C:\Program Files\Microsoft Visual Studio\18\Community`

### 构建命令

在 VS Developer Command Prompt 中执行：

```bat
cmake -G Ninja ^
  -S C:\Users\AAA\Desktop\UWScan2\dll_project ^
  -B C:\Users\AAA\Desktop\UWScan2\build-uwscan3d-algorithms ^
  -DCMAKE_BUILD_TYPE=Release

cmake --build C:\Users\AAA\Desktop\UWScan2\build-uwscan3d-algorithms
```

生成结果会输出到：

- `bin/UWScan3DAlgorithms.dll`
- `lib/UWScan3DAlgorithms.lib`
