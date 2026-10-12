这是我软件的点云系统架构的技术提示词，你需要通过阅读本提示词，来了解我的点云系统的架构设计，并协助我继续开发我的点云系统。
后续你协助我对点云系统的每一次开发，都需要告知我需要如何修改本技术提示词。
如果你明白了我的想法，请告知我你已知晓，然后等待接收我的下一个提示词指令。

# 点云系统架构技术提示词
## 系统概述
这是一个基于Qt框架和PCL(Point Cloud Library)构建的点云处理系统，采用模块化设计，主要包括以下核心组件：

### 主要模块
1. **主窗口模块(mainwindow)** - 系统入口和用户界面
2. **点云处理模块(pointcloud)** - 核心点云处理功能

## 核心类设计

### 1. 主窗口类(UWScanMainWindow)
**文件**: `mainwindow/uwscanmainwindow.h`, `mainwindow/uwscanmainwindow.cpp`

这是系统的主界面类，继承自SARibbonMainWindow，提供了基于Ribbon风格的现代化用户界面。

**成员变量**:
- `SARibbonBar* ribbon` - 主窗口的Ribbon栏
- `SARibbonCategory * fileCategory` - 文件Tab
- `SARibbonPanel * filePanel` - 文件Tab的Panel
- `QAction * newAction` - 新建项目Action
- `QAction * openAction` - 打开文件Action
- `QMenu * saveMenu` - 保存文件菜单
- `QAction * savePointcloudAction` - 保存文件Action
- `QAction * saveImageAction` - 保存图像Action
- `QAction * saveDepthAction` - 保存深度图Action
- `SARibbonCategory * scanCategory` - 扫描Tab
- `SARibbonPanel * connPanel` - 扫描Tab的通信Panel
- `QAction * connAction` - 连接Action
- `SARibbonPanel * scanPanel` - 扫描Tab的扫描Panel
- `QAction * previewAction` - 预览Action
- `QAction * stopPreviewAction` - 停止预览Action
- `QAction * scanAction` - 扫描Action
- `SARibbonCategory * pointcloudCategory` - 点云Tab
- `SARibbonPanel * selectPanel` - 点云Tab的选择操作Panel
- `QAction * selectAllAction` - 全选Action
- `QAction * selectNoneAction` - 取消选中Action
- `QAction * deleteAction` - 删除选中Action
- `QAction * reverseAction` - 反选Action
- `QAction * rotateCenterAction` - 旋转中心Action
- `QMenu * selectToolMenu` - 选择工具Action
- `QAction * rectToolAction` - 矩形选择工具Action
- `QAction * lassoToolAction` - 套索选择工具Action
- `QAction * polyToolAction` - 多边形选择工具Action
- `SARibbonPanel * featurePanel` - 点云Tab的特征提取Panel
- `QAction * ballAction` - 球变换Action
- `QAction * planeAction` - 平面变换Action
- `QAction * cylinderAction` - 圆柱体变换Action
- `QAction * sphereAction` - 球体变换Action
- `QAction * extractAction` - 提取特征Action
- `SARibbonCategory * settingCategory` - 设置Tab
- `SARibbonPanel * settingPanel` - 设置Tab的Panel
- `QAction * settingAction` - 设置Action
- `SARibbonCategory * helpCategory` - 帮助Tab
- `SARibbonPanel * helpPanel` - 帮助Tab的Panel
- `QAction * helpAction` - 帮助Action
- `QTabWidget * centerTabWidget` - 中央标签页控件
- `pointcloud::CloudWindow * cloudWidget` - 点云Tab的Widget
- `ImageWidget * imageWidget` - 图像Tab的Widget
- `InfofullStatusBar * statusBar` - 状态栏
- `pointcloud::CloudTreeWidget* treeWidget` - 点云Tab的树状Widget
- `QDockWidget * treeDockWidget` - 点云Tab的树状Widget的DockWidget
- `ScannerInterface* scanner` - 扫描仪接口
- `QDockWidget * scannerDockWidget` - 扫描Tab的通信Widget的DockWidget

**成员函数**:
- `UWScanMainWindow(QWidget *parent = nullptr)` - 构造函数
- `~UWScanMainWindow() noexcept` - 析构函数
- `void onNewActionTriggered()` - 新建项目Action触发槽函数
- `void onOpenActionTriggered()` - 打开文件Action触发槽函数
- `void onSavePointcloudActionTriggered()` - 保存点云文件Action触发槽函数
- `void onSaveImageActionTriggered()` - 保存图像文件Action触发槽函数
- `void onSaveDepthActionTriggered()` - 保存深度图文件Action触发槽函数
- `void onConnActionTriggered()` - 连接Action触发槽函数
- `void onPreviewActionTriggered()` - 预览Action触发槽函数
- `void onStopPreviewActionTriggered()` - 停止预览Action触发槽函数
- `void onScanActionTriggered()` - 扫描Action触发槽函数
- `void onSelectAllActionTriggered()` - 全选Action触发槽函数
- `void onSelectNoneActionTriggered()` - 取消选中Action触发槽函数
- `void onDeleteActionTriggered()` - 删除选中Action触发槽函数
- `void onReverseActionTriggered()` - 反选Action触发槽函数
- `void onRotateCenterActionTriggered()` - 旋转中心Action触发槽函数
- `void onSelectToolActionTriggered()` - 选择工具Action触发槽函数
- `void refreshAllClouds()` - 刷新所有点云
- `void onSelectionFinished(const std::vector<QPoint>& polygon)` - 刷新VTK控件

**主要特性**:
- 使用Ribbon界面组织功能，包含文件、扫描、点云、设置、帮助等Tab
- 集成点云显示窗口(CloudWindow)和图像显示窗口(ImageWidget)
- 管理点云树状结构(CloudTreeWidget)和扫描仪接口(ScannerInterface)
- 实现各种用户操作的槽函数，如文件操作、点云选择、特征提取等

**关键组件**:
- Ribbon栏和各类别面板(Action)
- 中央标签页控件，包含点云和图像Tab
- 状态栏和停靠窗口

### 2. 点云显示窗口(CloudWindow)
**文件**: `pointcloud/cloudwindow.h`, `pointcloud/cloudwindow.cpp`

这是点云可视化的核心组件，基于QVTKOpenGLNativeWidget和PCLVisualizer实现。

**成员变量**:
- `QVTKOpenGLNativeWidget* vtk_widget_` - VTK控件
- `SelectionOverlay* overlay_` - 透明覆盖层
- `QStackedLayout* layout_` - 层叠布局
- `pcl::visualization::PCLVisualizer::Ptr viewer_` - PCL可视化器
- `std::shared_ptr<PointCloudInterface> cloud_manager_` - 点云管理器

**成员函数**:
- `CloudWindow(QWidget *parent = nullptr)` - 构造函数
- `~CloudWindow()` - 析构函数
- `void setInteractionMode(InteractionMode mode)` - 设置当前交互模式
- `pcl::visualization::PCLVisualizer::Ptr getViewer()` - 获取viewer指针以便外部添加点云
- `void setCloud(std::shared_ptr<PointCloudInterface> cloud)` - 设置点云(连接逻辑与UI)
- `void onSelectionFinished(const std::vector<QPoint>& polygon)` - 响应Overlay发来的选择完成信号
- `void onFocusCloud(float x, float y, float z)` - 聚焦到某一点云位置的槽函数

**主要职责**:
- 显示3D点云数据
- 提供多种交互模式：浏览、矩形选择、套索选择、多边形选择
- 处理用户输入和鼠标事件

**内部类**:
- SelectionOverlay: 透明覆盖层，负责绘制选择区域和捕获鼠标事件

**枚举类型**:
- InteractionMode: 定义不同的交互模式(View, RectSelect, LassoSelect, PolySelect)
- SelectMode: 选择模式标识
- LeftButtonStatus: 鼠标左键状态

### 3. 点云接口和实现(PointCloudInterface/PointCloud)
**文件**: `pointcloud/pc.hpp`

采用接口和模板类设计模式，提供统一的点云操作接口。

**PointCloudInterface**:
**成员函数**:
- `virtual void selectAll() = 0` - 全选所有点
- `virtual void selectNone() = 0` - 取消所有点的选择
- `virtual void invertSelection() = 0` - 反选当前选择
- `virtual bool empty() = 0` - 检查点云是否为空
- `virtual QString getName() const = 0` - 获取点云名称
- `virtual void setName(const QString& name) = 0` - 设置点云名称
- `virtual void setVisible(bool visible) = 0` - 设置点云可见性
- `virtual bool isVisible() const = 0` - 检查点云是否可见
- `virtual std::string getId() const = 0` - 获取点云唯一ID
- `virtual void selectByScreenPolygon(const std::vector<QPoint>& polygon, pcl::visualization::PCLVisualizer::Ptr viewer, bool is_add_selection = true) = 0` - 根据屏幕多边形选择点
- `virtual void render(pcl::visualization::PCLVisualizer::Ptr viewer) = 0` - 渲染点云
- `virtual size_t getPointCount() const = 0` - 获取点云总点数
- `virtual size_t getSelectedCount() const = 0` - 获取选中点数
- `virtual void getCentroid(float & x, float & y, float & z) const = 0` - 计算点云质心

**保护成员函数**:
- `bool isPointInPolygon(float x, float y, const std::vector<QPoint>& polygon)` - 判断点是否在多边形内

**PointCloud<T>**:
**成员变量**:
- `CloudPtr cloud_` - 点云数据指针
- `std::vector<int> all_indices_` - 所有点云的索引
- `std::vector<int> selected_indices_` - 当前选中的索引
- `QString name_` - 点云名称
- `std::string id_` - 点云唯一ID
- `bool need_update_` - 渲染脏标记
- `bool visible_` - 点云可见性标志

**成员函数**:
- `PointCloud(CloudPtr cloud, QString name)` - 构造函数
- `~PointCloud()` - 析构函数
- `std::string getId() const override` - 获取点云唯一ID
- `QString getName() const override` - 获取点云名称
- `void setName(const QString& name) override` - 设置点云名称
- `void setVisible(bool visible) override` - 设置点云可见性
- `bool isVisible() const override` - 检查点云是否可见
- `void selectAll() override` - 全选所有点
- `void selectNone() override` - 取消所有点的选择
- `void invertSelection() override` - 反选当前选择
- `bool empty() override` - 检查点云是否为空
- `virtual void getCentroid(float & x, float & y, float & z) const override` - 计算点云质心
- `void selectByScreenPolygon(const std::vector<QPoint>& polygon, pcl::visualization::PCLVisualizer::Ptr viewer, bool is_add_selection = true) override` - 根据屏幕多边形选择点
- `void render(pcl::visualization::PCLVisualizer::Ptr viewer) override` - 渲染点云
- `size_t getPointCount() const override` - 获取点云总点数
- `size_t getSelectedCount() const override` - 获取选中点数

**主要特性**:
- 定义点云的基本操作接口，如选择、渲染、可见性控制等
- 提供选择检测算法(点在多边形内判断)

**模板类实现**:
- 支持不同类型的点云数据(T: PointXYZ, PointXYZRGB等)
- 管理点云数据和选择状态
- 实现渲染逻辑，包括选中点的高亮显示

### 4. 点云树状结构管理(CloudTreeWidget/CloudTreeItem)
**文件**: `pointcloud/cloudtreewidget.h`, `pointcloud/cloudtreeitem.h`

提供GUI层面的点云管理，以树状结构展示点云列表。

**CloudTreeWidget**:
**成员变量**:
- `CloudTreeItem* context_item_` - 临时存储右键点击时的Item

**成员函数**:
- `CloudTreeWidget(QWidget *parent = nullptr)` - 构造函数
- `void addCloud(std::shared_ptr<PointCloudInterface> cloud)` - 添加一个新的点云到树中
- `std::shared_ptr<PointCloudInterface> getCurrentCloud() const` - 获取当前选中的点云
- `std::vector<std::shared_ptr<PointCloudInterface>> getAllClouds() const` - 获取所有点云列表

**信号**:
- `void refreshViewRequested()` - 请求3D视图重绘
- `void cloudDeleted(QString cloudId)` - 点云被删除信号
- `void focusOnCloudRequest(float x, float y, float z)` - 请求聚焦到特定位置

**槽函数**:
- `void onCustomContextMenuRequested(const QPoint &pos)` - 处理右键点击
- `void toggleVisibility()` - 切换可见性
- `void showInfo()` - 显示信息
- `void deleteCloud()` - 删除点云

**CloudTreeItem**:
**成员变量**:
- `std::shared_ptr<PointCloudInterface> cloud_` - 点云对象

**成员函数**:
- `CloudTreeItem(QTreeWidget* parent, std::shared_ptr<PointCloudInterface> cloud)` - 构造函数
- `std::shared_ptr<PointCloudInterface> getCloud() const` - 获取持有的点云对象
- `void updateAppearence()` - 根据点云状态刷新UI

**主要职责**:
- CloudTreeWidget: 继承自QTreeWidget，管理多个点云项，提供添加点云、获取当前选中点云、获取所有点云等功能，实现右键菜单功能(显隐切换、属性查看、视野聚焦、删除等功能)
- CloudTreeItem: 继承自QTreeWidgetItem，代表单个点云项，内部维护一个PointCloudInterface指针，指向一个具体的点云实现对象，根据点云状态更新UI显示

### 5. 点云加载器(CloudLoader)
**文件**: `pointcloud/cloudloader.h`, `pointcloud/cloudloader.cpp`

统一的点云文件加载器，支持多种格式。

**成员函数**:
- `static std::shared_ptr<PointCloudInterface> loadFile(const QString& file_path)` - 通用加载函数

**私有模板函数**:
- `template <typename PointT> static typename pcl::PointCloud<PointT>::Ptr loadTextFile(const QString& file_path)` - 内部辅助函数：读取文本类型的点云

**支持格式**:
- PCD(Point Cloud Data)
- PLY(Polygon File Format)
- TXT/ASC(文本格式)
  
**核心方法**:
- loadFile(): 通用加载函数，根据文件扩展名选择合适的加载器

## 设计模式和架构特点

### 1. 接口隔离原则
使用PointCloudInterface抽象接口，隐藏具体实现细节，便于扩展不同类型点云。

### 2. MV架构
- Model: PointCloudInterface
- View: CloudTreeWidget, CloudWindow

### 3. 事件驱动设计
大量使用Qt信号槽机制实现组件间通信:
- CloudWindow发出selectionFinished信号通知选择完成
- CloudTreeWidget发出refreshViewRequested信号请求视图刷新
- 其它槽函数：如删除点云、切换显隐等

### 4. 模板泛型设计
PointCloud使用模板参数支持不同类型的点云数据，提高代码复用性。

## 系统工作流程

1. **启动**: UWScanMainWindow初始化，创建各个组件
2. **加载点云**: 用户通过界面操作加载点云文件 -> CloudLoader解析文件 -> 创建PointCloudInterface对象 -> CloudTreeWidget显示
3. **显示点云**: ModelController调用PointCloud.render()方法 -> PCLVisualizer渲染 -> CloudWindow显示
4. **交互操作**: 用户在CloudWindow中进行选择 -> SelectionOverlay捕获事件 -> 发送selectionFinished信号 -> 更新PointCloud选择状态 -> 重新渲染
5. **状态同步**: 点云状态变化 -> CloudTreeItem.updateAppearance()更新UI