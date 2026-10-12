#ifndef CLOUDWINDOW_H
#define CLOUDWINDOW_H
#include "utils.h"
#include <QWidget>
#include <vtkRenderWindow.h>
#include <vtkInteractionStyleModule.h>
#include <vtkLine.h>
#include <vtkAxesActor.h>
#include <vtkTextActor.h>
#include <vtkPolyline.h>
#include <vtkPointPicker.h>
#include <vtkActor2DCollection.h>
#include <QVTKRenderWidget.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInformation.h>
#include <vtkSmartPointer.h>
#include "pcl/visualization/pcl_visualizer.h"
#include "pointcloud/pc.hpp"
#include <QStackedLayout>
#include <QLabel>
#include <QDebug>
#include <QHash>
#include <QPointer>
#include <QStringList>
#include "pointcloud/cloudtreewidget.h"

NAMESPACE_BEGIN{ namespace pointcloud{
    /**
     * @brief 交互模式枚举
     */
    enum class InteractionMode{
        View = 0,  //纯浏览操作
        RectSelect = 1,  //矩形框选
        LassoSelect = 2,  //套索框选
        PolySelect = 3,  //多边形框选，按enter结束
    };

    /**
     * @brief 屏幕坐标点结构体
     */
    struct ScreenPoint {
        unsigned int x, y;
    };

    /**
     * @brief 内部辅助类：透明覆盖层，负责画图和捕获鼠标事件
     * SelectionOverlay是一个透明的覆盖层，用于在点云可视化窗口上进行选择操作。
     * 它支持多种选择模式，包括矩形选择、套索选择和多边形选择。
     */
    class SelectionOverlay : public QWidget {
        Q_OBJECT
    public:
        /**
         * @brief 构造函数
         * @param parent 父窗口指针
         */
        explicit SelectionOverlay(QWidget *parent = nullptr);
        
        /**
         * @brief 设置交互模式
         * @param m 交互模式
         */
        void setMode(InteractionMode m);

        /**
         * @brief 清空绘制内容
         */
        void clearShapes(); // 清空绘制内容

    signals:
        /**
         * @brief 选择完成信号
         * @param polygon 屏幕上的多边形区域
         */
        void selectionFinished(const std::vector<QPoint>& polygon);

        /**
         * @brief 取消选择信号
         */
        void selectionCanceled();
        
        /**
         * @brief 全选信号
         */
        void selectAll();
        
        /**
         * @brief 反选信号
         */
        void reverseSelect();
        
        /**
         * @brief 删除选中点信号
         */
        void deleteSelectedPoints();

    protected:
        /**
         * @brief 绘制事件处理
         * @param event 绘制事件指针
         */
        void paintEvent(QPaintEvent *event) override;
        
        /**
         * @brief 鼠标按下事件处理
         * @param event 鼠标事件指针
         */
        void mousePressEvent(QMouseEvent *event) override;

        /**
         * @brief 鼠标移动事件处理
         * @param event 鼠标事件指针
         */
        void mouseMoveEvent(QMouseEvent *event) override;

        /**
         * @brief 鼠标滚轮事件处理
         * @param event 滚轮事件指针
         */
        void wheelEvent(QWheelEvent *event) override;

        /**
         * @brief 鼠标释放事件处理
         * @param event 鼠标事件指针
         */
        void mouseReleaseEvent(QMouseEvent *event) override;

        /**
         * @brief 键盘按下事件处理
         * @param event 键盘事件指针
         */
        void keyPressEvent(QKeyEvent *event) override;

    private:
        InteractionMode mode_ = InteractionMode::View;  ///< 当前交互模式
        QVector<QPoint> points_;                        ///< 存储多边形或套索的点
        QPoint current_mouse_pos_;                      ///< 鼠标当前位置 (用于多边形橡皮筋效果)
        bool is_drawing_ = false;                       ///< 是否正在拖拽/绘制中

        /**
         * @brief 完成矩形选择
         */
        void finishRect();

        /**
         * @brief 完成套索选择
         */
        void finishLasso();

        /**
         * @brief 完成多边形选择
         */
        void finishPoly();
    };

    /**
     * @brief 点云可视化窗口类
     * 
     * CloudWindow是点云可视化的主窗口类，集成了VTK渲染窗口和选择覆盖层。
     * 它支持多种交互模式，包括浏览、矩形选择、套索选择和多边形选择。
     */
    class CloudWindow : public QWidget {
        Q_OBJECT
        friend class SelectionOverlay;

    public:
        /**
         * @brief 构造函数
         * @param cloud_tree 点云树控件指针
         * @param parent 父窗口指针
         */
        explicit CloudWindow(CloudTreeWidget* cloud_tree, QWidget *parent = nullptr);

        /**
         * @brief 析构函数
         */
        ~CloudWindow();

        /**
         * @brief 设置当前交互模式
         * @param mode 交互模式
         */
        void setInteractionMode(InteractionMode mode);

        /**
         * @brief 获取 viewer 指针以便外部添加点云
         * @return viewer 指针
         */
        pcl::visualization::PCLVisualizer::Ptr getViewer() { return viewer_; }

        /**
         * @brief 清除当前拟合框架覆盖物
         */
        void clearFitOverlays();

        /**
         * @brief 显示误差色谱图例
         * @param title 图例标题
         * @param maxAbsError 最大绝对误差
         */
        void setErrorLegend(const QString& title, double maxAbsError);

        /**
         * @brief 清除误差色谱图例
         */
        void clearErrorLegend();

        /**
         * @brief 显示圆柱拟合框架
         * @param baseId 覆盖物基础ID
         * @param axisPoint 圆柱轴线上一点
         * @param axis 圆柱轴线方向
         * @param radius 圆柱半径
         * @param minAxisCoord 轴向最小投影
         * @param maxAxisCoord 轴向最大投影
         */
        void showCylinderFitOverlay(const QString& baseId,
                                    const Eigen::Vector3f& axisPoint,
                                    const Eigen::Vector3f& axis,
                                    double radius,
                                    double minAxisCoord,
                                    double maxAxisCoord);

        /**
         * @brief 显示球体拟合框架
         * @param baseId 覆盖物基础ID
         * @param center 球心
         * @param radius 球半径
         */
        void showSphereFitOverlay(const QString& baseId,
                                  const Eigen::Vector3f& center,
                                  double radius);

        /**
         * @brief 登记外部创建的覆盖物ID，使删除/隐藏点云组时能同步清理
         * @param baseId 覆盖物ID
         */
        void registerFitOverlayId(const QString& baseId);

        /**
         * @brief 将点云绑定到某个拟合框架组
         * @param cloudId 点云ID
         * @param fitGroupKey 拟合组Key，通常为 "<sourceCloudId>_fit"
         */
        void setFitOverlayGroupForCloud(const QString& cloudId, const QString& fitGroupKey);

        /**
         * @deprecated 该方法已被弃用，建议使用 CloudTreeWidget 进行点云管理
         * @brief 设置点云 (连接逻辑与UI)
         * @param cloud 点云指针
         */
        void setCloud(std::shared_ptr<PointCloud> cloud);

        /**
         * @brief 更新左下角点云数量显示
         * @param pointCount 当前显示点数
         */
        void setPointCount(size_t pointCount);

        /**
         * @brief 更新左下角当前点云坐标显示
         */
        void setSelectedCloudCoordinates(const QString& name, float x, float y, float z);

        /**
         * @brief 更新左下角当前点云信息显示
         * @param message 要显示的文本
         */
        void setSelectedCloudMessage(const QString& message);

        /**
         * @brief 清空当前点云坐标显示
         */
        void clearSelectedCloudCoordinates();

    public slots:
        /**
         * @brief 响应 Overlay 发来的选择完成信号
         * @param polygon 屏幕上的多边形区域
         */
        void onSelectionFinished(const std::vector<QPoint>& polygon);

        /**
         * @brief 聚焦到某一点云位置的槽函数
         * @param x 点云x坐标
         * @param y 点云y坐标
         * @param z 点云z坐标
         */
        void onFocusCloud(float x, float y, float z);  

        /**
         * @brief 使用正投影视角显示指定中心附近点云
         * @param x 点云中心 X 坐标
         * @param y 点云中心 Y 坐标
         * @param z 点云中心 Z 坐标
         */
        void onFocusCloudOrthographic(float x, float y, float z);

        /**
         * @brief 响应选择工具动作
         */
        void onRectToolActionTriggered() { 
            setInteractionMode(InteractionMode::RectSelect); 
        }

        /**
         * @brief 响应套索选择工具动作
         */
        void onLassoToolActionTriggered() { 
            setInteractionMode(InteractionMode::LassoSelect); 
        }

        /**
         * @brief 响应多边形选择工具动作
         */
        void onPolyToolActionTriggered() { 
            setInteractionMode(InteractionMode::PolySelect); 
        }

        /**
         * @brief 刷新 VTK 界面
         */
        void refreshView(){
            vtk_widget_->GetRenderWindow()->Render();
            vtk_widget_->update();
        }

        /**
         * @brief 全选所有点
         */
        void selectAllPoints();

        /**
         * @brief 反选当前选择
         */
        void reverseSelect();

        /**
         * @brief 取消当前选择
         */
        void clearSelection();

        /**
         * @brief 删除选中点
         */
        void deleteSelectedPoints();

        /**
         * @brief 响应点云删除信号
         * @param cloudId 被删除的点云的唯一标识符
         */
        void onCloudDeleted(QString cloudId);

        /**
         * @brief 响应点云显隐变化，同步拟合框架
         * @param cloudId 点云ID
         * @param visible 是否可见
         */
        void onCloudVisibilityChanged(QString cloudId, bool visible);

    protected:
        void resizeEvent(QResizeEvent* event) override;

    private:
        enum class FitOverlayType {
            Cylinder,
            Sphere
        };

        struct FitOverlayState {
            FitOverlayType type = FitOverlayType::Sphere;
            Eigen::Vector3f axisPoint = Eigen::Vector3f::Zero();
            Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
            double radius = 0.0;
            double minAxisCoord = 0.0;
            double maxAxisCoord = 0.0;
            Eigen::Vector3f center = Eigen::Vector3f::Zero();
        };

        void removeFitOverlayGroup(const QString& baseId);
        QString fitGroupKeyForCloud(const QString& cloudId) const;
        void removeDisplayedFitOverlaysForGroup(const QString& fitGroupKey);
        void restoreFitOverlaysForCloud(const QString& cloudId);
        void removeFitOverlayCacheForCloud(const QString& cloudId);
        void removeFitOverlayGroupBinding(const QString& cloudId);
        void removeFitOverlayBindingsForGroup(const QString& fitGroupKey);
        void updateLegendPlacement();
        void updatePickedPointCoordinates(int x, int y);
        void showPickedPointMarker(float x, float y, float z);
        void clearPickedPointMarker();

        // UI 组件
        QVTKOpenGLNativeWidget* vtk_widget_;    ///< VTK 渲染窗口组件
        SelectionOverlay* overlay_;             ///< 选择覆盖层
        QLabel* point_count_label_;             ///< 点数显示标签
        QLabel* coordinate_label_;              ///< 当前点云坐标显示标签
        QWidget* legend_panel_;                 ///< 误差色谱图例面板
        QLabel* legend_title_label_;            ///< 误差色谱标题
        QLabel* legend_bar_label_;              ///< 误差色谱条
        QLabel* legend_top_label_;              ///< 误差色谱顶部刻度
        QLabel* legend_mid_label_;              ///< 误差色谱中部刻度
        QLabel* legend_bottom_label_;           ///< 误差色谱底部刻度
        QStackedLayout* layout_;                ///< 用于层叠布局的栈布局

        // PCL 组件
        pcl::visualization::PCLVisualizer::Ptr viewer_; ///< PCL 可视化器指针
        
        // 逻辑组件
        QPointer<CloudTreeWidget> cloud_tree_; ///< 点云树控件指针
        QStringList fit_overlay_ids_;          ///< 当前拟合框架覆盖物ID列表

        QHash<QString, FitOverlayState> fit_overlay_cache_;
        QHash<QString, QString> fit_overlay_group_by_cloud_;
        vtkSmartPointer<vtkPointPicker> point_picker_;
        const std::string picked_point_marker_id_ = "picked_point_marker";

    private slots:
        /**
         * @brief 响应自定义上下文菜单请求
         * @param pos 鼠标点击位置
         */
        void onCustomContextMenuRequested(const QPoint &pos);

    signals:
        /**
         * @brief 选择完成信号
         * @param polygon 屏幕上的多边形区域
         */
        void selectionFinished(const std::vector<QPoint>& polygon);
    };
}}

#endif // CLOUDWINDOW_H
