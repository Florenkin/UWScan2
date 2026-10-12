#include "pointcloud/cloudwindow.h"
#include <QVBoxLayout>
#include <QPainter>
#include <QKeyEvent>
#include <QStackedLayout>
#include <QApplication>
#include <QDebug>
#include <QAction>
#include <QMenu>
#include <QFont>
#include <QPixmap>
#include <vtkProperty.h>
#include <cmath>

NAMESPACE_BEGIN{ namespace pointcloud{
    namespace {
        static double quantizedLegendErrorValue(double errorValue) {
            const double clamped = std::clamp(errorValue, -1.0, 1.0);
            return std::round(clamped * 10.0) / 10.0;
        }

        static QColor legendErrorPaletteColor(double quantizedError) {
            const std::array<std::pair<double, QColor>, 4> stops = {{
                {-1.0, QColor(0, 0, 255)},
                {0.0, QColor(0, 255, 0)},
                {0.5, QColor(255, 255, 0)},
                {1.0, QColor(255, 0, 0)}
            }};
            const double value = std::clamp(quantizedError, -1.0, 1.0);
            for (size_t i = 1; i < stops.size(); ++i) {
                if (value <= stops[i].first) {
                    const double leftPos = stops[i - 1].first;
                    const double rightPos = stops[i].first;
                    const double t = (value - leftPos) / std::max(rightPos - leftPos, 1e-9);
                    const QColor& left = stops[i - 1].second;
                    const QColor& right = stops[i].second;
                    const int red = static_cast<int>(std::lround(left.red() + (right.red() - left.red()) * t));
                    const int green = static_cast<int>(std::lround(left.green() + (right.green() - left.green()) * t));
                    const int blue = static_cast<int>(std::lround(left.blue() + (right.blue() - left.blue()) * t));
                    return QColor(red, green, blue);
                }
            }
            return stops.back().second;
        }

        static QColor legendJetColor(double errorValue) {
            const double quantized = quantizedLegendErrorValue(errorValue);
            return legendErrorPaletteColor(quantized);
        }

        static QPixmap createLegendPixmap(int width, int height) {
            QPixmap pixmap(width, height);
            pixmap.fill(Qt::transparent);
            QPainter painter(&pixmap);
            painter.setRenderHint(QPainter::TextAntialiasing, true);

            const int barX = 12;
            const int barWidth = 34;
            const int tickX = barX + barWidth + 6;
            const int labelX = tickX + 8;
            const int segments = 21; // [-1, 1] 每 0.1 一档，共 21 个离散色块
            const double segmentHeight = static_cast<double>(height) / static_cast<double>(segments);
            for (int i = 0; i < segments; ++i) {
                const double errorValue = quantizedLegendErrorValue(1.0 - static_cast<double>(i) * 0.1);
                const int y0 = static_cast<int>(std::round(i * segmentHeight));
                const int y1 = static_cast<int>(std::round((i + 1) * segmentHeight));
                painter.fillRect(barX, y0, barWidth, std::max(1, y1 - y0), legendJetColor(errorValue));
            }

            painter.setPen(QColor(40, 40, 40, 180));
            painter.drawRect(barX, 0, barWidth, height - 1);

            QFont tickFont = painter.font();
            tickFont.setPointSize(8);
            painter.setFont(tickFont);
            for (int i = 0; i < segments; ++i) {
                const double value = quantizedLegendErrorValue(1.0 - static_cast<double>(i) * 0.1);
                const int y0 = static_cast<int>(std::round(i * segmentHeight));
                const int y1 = static_cast<int>(std::round((i + 1) * segmentHeight));
                const int centerY = std::clamp((y0 + y1) / 2, 0, height - 1);
                painter.drawLine(tickX, centerY, tickX + 6, centerY);
                painter.drawText(labelX,
                                 std::clamp(centerY - 6, 0, height - 12),
                                 84,
                                 12,
                                 Qt::AlignLeft | Qt::AlignVCenter,
                                 QString::number(value, 'f', 1));
            }
            return pixmap;
        }
    }

    // ==========================================
    // Section 1: SelectionOverlay 实现
    // ==========================================
    SelectionOverlay::SelectionOverlay(QWidget *parent) : QWidget(parent) {
        //以此设置让 Widget 背景透明，但能接收鼠标事件
        setAttribute(Qt::WA_NoSystemBackground); 
        setAttribute(Qt::WA_TranslucentBackground);
        setFocusPolicy(Qt::StrongFocus); // 必须设置，否则收不到键盘 Enter 事件
        setMouseTracking(true); // 必须设置，以便在多边形模式下追踪鼠标移动
    }

    void SelectionOverlay::setMode(InteractionMode m) {
        mode_ = m;
        clearShapes();
    }

    void SelectionOverlay::clearShapes() {
        points_.clear();
        is_drawing_ = false;
        update(); // 触发重绘
    }

    void SelectionOverlay::paintEvent(QPaintEvent *event) {
        Q_UNUSED(event);
        
        // 如果不是选择模式，或者没在画，就不画
        if (mode_ == InteractionMode::View) return;
        if (points_.isEmpty() && mode_ != InteractionMode::PolySelect) return;

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // 设置画笔风格 (红色边框，半透明填充)
        QPen pen(Qt::red, 2, Qt::SolidLine);
        painter.setPen(pen);
        QBrush brush(QColor(255, 0, 0, 50)); // 红色，透明度50
        painter.setBrush(brush);

        if (mode_ == InteractionMode::RectSelect && is_drawing_) {
            if (points_.size() >= 1) {
                // 画矩形：起点 -> 当前鼠标位置
                QRect rect(points_.first(), current_mouse_pos_);
                painter.drawRect(rect);
            }
        } 
        else if (mode_ == static_cast<InteractionMode>(4) && is_drawing_) {
            if (points_.size() >= 1) {
                QRect rect(points_.first(), current_mouse_pos_);
                painter.drawEllipse(rect.normalized());
            }
        }
        else if (mode_ == InteractionMode::LassoSelect && is_drawing_) {
            // 画不规则路径
            if (points_.size() > 1) {
                painter.drawPolygon(points_);
            }
        }
        else if (mode_ == InteractionMode::PolySelect) {
            // 画多边形路径 + 橡皮筋线
            if (!points_.isEmpty()) {
                // 画已确定的线段
                painter.drawPolyline(points_);
                // 画当前鼠标跟随线
                painter.drawLine(points_.last(), current_mouse_pos_);
                // 画各个顶点
                painter.setBrush(Qt::red);
                for(const auto& p : points_) painter.drawEllipse(p, 3, 3);

                if (points_.size() > 1) {
                    painter.drawLine(current_mouse_pos_, points_.first());
                }
            }
        }
    }

    void SelectionOverlay::mousePressEvent(QMouseEvent *event) {
        //按下鼠标右键时，控制权交给父窗口，用来弹出菜单框
        if (event->button() == Qt::RightButton) {
            qDebug() << "Right Button Pressed";
            QCoreApplication::sendEvent(parent(), event);
            return;
        }

        // 未按下control时，控制权移交到vtk窗口
        if(!is_drawing_ && !(event->modifiers() & Qt::ControlModifier)) {
            CloudWindow * cloudWindow = qobject_cast<CloudWindow*>(parent());
            if(cloudWindow) {
                if (event->button() == Qt::LeftButton) {
                    cloudWindow->updatePickedPointCoordinates(event->pos().x(), event->pos().y());
                }
                QCoreApplication::sendEvent(cloudWindow->vtk_widget_, event);
                return;
            }
        }

        if (event->button() == Qt::LeftButton) {
            if (mode_ == InteractionMode::RectSelect) {
                points_.clear();
                points_.append(event->pos());
                is_drawing_ = true;
            } 
            else if (mode_ == static_cast<InteractionMode>(4)) {
                points_.clear();
                points_.append(event->pos());
                is_drawing_ = true;
            }
            else if (mode_ == InteractionMode::LassoSelect) {
                points_.clear();
                points_.append(event->pos());
                is_drawing_ = true;
            }
            else if (mode_ == InteractionMode::PolySelect) {
                // 多边形模式：点击一次添加一个点
                points_.append(event->pos());
                is_drawing_ = true; 
                update();
            }
        }
        else if (event->button() == Qt::RightButton) {
            // 右键取消选择
            clearShapes();
        }
    }

    void SelectionOverlay::mouseMoveEvent(QMouseEvent *event) {
        current_mouse_pos_ = event->pos();

        // 未按下control时，控制权移交到vtk窗口
        if(!is_drawing_ && !(event->modifiers() & Qt::ControlModifier)) {
            CloudWindow * cloudWindow = qobject_cast<CloudWindow*>(parent());
            if(cloudWindow) {
                QCoreApplication::sendEvent(cloudWindow->vtk_widget_, event);
                return;
            }
        }

        if ((mode_ == InteractionMode::RectSelect || mode_ == static_cast<InteractionMode>(4)) && is_drawing_) {
            update(); // 刷新矩形
        }
        else if (mode_ == InteractionMode::LassoSelect && is_drawing_) {
            points_.append(event->pos()); // 记录轨迹
            update();
        }
        else if (mode_ == InteractionMode::PolySelect) {
            update(); // 刷新橡皮筋线
        }
    }

    void SelectionOverlay::mouseReleaseEvent(QMouseEvent *event) {
        current_mouse_pos_ = event->pos();
        // 未按下control时，控制权移交到vtk窗口
        if(!is_drawing_ && !(event->modifiers() & Qt::ControlModifier)) {
            CloudWindow * cloudWindow = qobject_cast<CloudWindow*>(parent());
            if(cloudWindow) {
                QCoreApplication::sendEvent(cloudWindow->vtk_widget_, event);
                return;
            }
        }

        if (event->button() == Qt::LeftButton) {
            if (mode_ == InteractionMode::RectSelect) {
                is_drawing_ = false;
                finishRect();
            }
            else if (mode_ == static_cast<InteractionMode>(4)) {
                is_drawing_ = false;
                if (!points_.isEmpty()) {
                    const QRect normalized = QRect(points_.first(), current_mouse_pos_).normalized();
                    if (normalized.width() > 0 && normalized.height() > 0) {
                        constexpr int kEllipseSegments = 72;
                        constexpr double kTwoPi = 6.28318530717958647692;
                        const double centerX = normalized.center().x();
                        const double centerY = normalized.center().y();
                        const double radiusX = normalized.width() / 2.0;
                        const double radiusY = normalized.height() / 2.0;
                        std::vector<QPoint> poly;
                        poly.reserve(kEllipseSegments);
                        for (int i = 0; i < kEllipseSegments; ++i) {
                            const double angle = (kTwoPi * i) / kEllipseSegments;
                            const int x = static_cast<int>(std::lround(centerX + std::cos(angle) * radiusX));
                            const int y = static_cast<int>(std::lround(centerY + std::sin(angle) * radiusY));
                            poly.emplace_back(x, y);
                        }
                        emit selectionFinished(poly);
                    }
                }
                clearShapes();
            }
            else if (mode_ == InteractionMode::LassoSelect) {
                is_drawing_ = false;
                finishLasso();
            }
            // PolySelect 模式下松开鼠标不代表结束，需要等待 Enter 键
        }
    }

    void SelectionOverlay::keyPressEvent(QKeyEvent *event) {
        if (mode_ == InteractionMode::PolySelect) {
            if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
                finishPoly();
            } else if (event->key() == Qt::Key_Escape) {
                clearShapes();
            }
        }
        if (event->key() == Qt::Key_Delete) {
            emit deleteSelectedPoints();
            clearShapes();
        } else if(event->key() == Qt::Key_R && event->modifiers() == Qt::ControlModifier) {
            emit reverseSelect();
            clearShapes();
        } else if(event->key() == Qt::Key_C && event->modifiers() == Qt::ControlModifier) {
            emit selectionCanceled();
            clearShapes();
        } else if(event->key() == Qt::Key_A && event->modifiers() == Qt::ControlModifier) {
            emit selectAll();
            clearShapes();
        }
        QWidget::keyPressEvent(event);
    }

    void SelectionOverlay::wheelEvent(QWheelEvent *event){
        CloudWindow * cloudWindow = qobject_cast<CloudWindow*>(parent());
        if(cloudWindow) {
            QCoreApplication::sendEvent(cloudWindow->vtk_widget_, event);
            return;
        }
    }

    // --- 结束逻辑 ---
    void SelectionOverlay::finishRect() {
        if (points_.isEmpty()) return;
        QPoint p1 = points_.first();
        QPoint p2 = current_mouse_pos_; // 终点
        
        // 构造矩形的四个顶点 (顺时针)
        std::vector<QPoint> poly;
        poly.push_back(QPoint(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()))); // 左上
        poly.push_back(QPoint(std::max(p1.x(), p2.x()), std::min(p1.y(), p2.y()))); // 右上
        poly.push_back(QPoint(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()))); // 右下
        poly.push_back(QPoint(std::min(p1.x(), p2.x()), std::max(p1.y(), p2.y()))); // 左下

        emit selectionFinished(poly);
        clearShapes();
    }

    void SelectionOverlay::finishLasso() {
        // 简化套索点（可选：抽稀算法），这里直接转
        if (points_.size() < 3) return;
        std::vector<QPoint> poly;
        for(const auto& p : points_) poly.push_back(p);
        
        emit selectionFinished(poly);
        clearShapes();
    }

    void SelectionOverlay::finishPoly() {
        if (points_.size() < 3) return;
        std::vector<QPoint> poly;
        for(const auto& p : points_) poly.push_back(p);

        emit selectionFinished(poly);
        clearShapes();
    }

    // ==========================================
    // Section 2: CloudWindow 实现
    // ==========================================

    CloudWindow::CloudWindow(CloudTreeWidget* cloud_tree, QWidget *parent) : QWidget(parent) {
        setMinimumSize(0, 0);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        // 初始化布局
        layout_ = new QStackedLayout(this);
        layout_->setStackingMode(QStackedLayout::StackAll); // 允许控件重叠

        // 初始化 VTK Widget
        vtk_widget_ = new QVTKOpenGLNativeWidget(this);
        auto rendererWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        vtk_widget_->setRenderWindow(rendererWindow);

        auto renderer = vtkSmartPointer<vtkRenderer>::New();

        rendererWindow->AddRenderer(renderer);
        this->viewer_.reset(new pcl::visualization::PCLVisualizer(renderer, rendererWindow, "viewer", false));
        viewer_->setShowFPS(false);
        viewer_->addCoordinateSystem(20.0);

        renderer->SetBackground2(0.5, 0.5, 0.5);
        renderer->SetBackground(1.0, 1.0, 1.0);
        renderer->SetGradientBackground(1);

        // viewer->initCameraParameters();
        renderer->ResetCameraScreenSpace();
        viewer_->setupInteractor(rendererWindow->GetInteractor(), rendererWindow);
        renderer->GetActiveCamera()->ParallelProjectionOn();
        viewer_->getRenderWindow()->GlobalWarningDisplayOff();
        point_picker_ = vtkSmartPointer<vtkPointPicker>::New();
        point_picker_->SetTolerance(0.01);
        // 初始化 Overlay
        overlay_ = new SelectionOverlay(this);
        point_count_label_ = new QLabel(tr("点数：0"), this);
        point_count_label_->setObjectName("pointCloudCountLabel");
        point_count_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        point_count_label_->setStyleSheet("QLabel#pointCloudCountLabel { color: #202020; background: rgba(255,255,255,190); padding: 4px 8px; border: 1px solid rgba(0,0,0,50); }");
        point_count_label_->setMinimumWidth(110);
        point_count_label_->setMaximumHeight(28);
        point_count_label_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        coordinate_label_ = new QLabel(tr("坐标：--"), this);
        coordinate_label_->setObjectName("pointCloudCoordinateLabel");
        coordinate_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        coordinate_label_->setStyleSheet("QLabel#pointCloudCoordinateLabel { color: #202020; background: rgba(255,255,255,190); padding: 4px 8px; border: 1px solid rgba(0,0,0,50); }");
        coordinate_label_->setMinimumWidth(260);
        coordinate_label_->setMaximumHeight(28);
        coordinate_label_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        legend_panel_ = new QWidget(this);
        legend_panel_->setObjectName("pointCloudErrorLegendPanel");
        legend_panel_->setStyleSheet("QWidget#pointCloudErrorLegendPanel { background: rgba(255,255,255,210); border: 1px solid rgba(0,0,0,50); }");
        legend_panel_->setFixedSize(188, 430);
        legend_panel_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        legend_title_label_ = new QLabel(tr("误差色谱"), legend_panel_);
        legend_title_label_->setGeometry(12, 10, 164, 22);
        legend_title_label_->setAlignment(Qt::AlignCenter);
        legend_bar_label_ = new QLabel(legend_panel_);
        legend_bar_label_->setGeometry(12, 40, 164, 350);
        legend_top_label_ = new QLabel(legend_panel_);
        legend_top_label_->setGeometry(12, 396, 164, 20);
        legend_mid_label_ = new QLabel(legend_panel_);
        legend_mid_label_->hide();
        legend_bottom_label_ = new QLabel(legend_panel_);
        legend_bottom_label_->hide();
        for (QLabel* label : {legend_top_label_}) {
            label->setAlignment(Qt::AlignCenter);
            label->setStyleSheet("color: #202020; background: transparent;");
        }
        legend_panel_->hide();
        
        // 添加到布局 (顺序很重要：后添加的在上层)
        layout_->addWidget(vtk_widget_); 
        layout_->addWidget(overlay_);
        point_count_label_->raise();
        point_count_label_->move(12, height() - point_count_label_->height() - 12);
        coordinate_label_->raise();
        coordinate_label_->move(12, height() - point_count_label_->height() - coordinate_label_->height() - 18);
        updateLegendPlacement();

        // 6. 连接信号
        connect(overlay_, &SelectionOverlay::selectionFinished, this, &CloudWindow::onSelectionFinished);
        connect(overlay_, &SelectionOverlay::selectionCanceled, this, &CloudWindow::clearSelection);
        connect(overlay_, &SelectionOverlay::selectAll, this, &CloudWindow::selectAllPoints);
        connect(overlay_, &SelectionOverlay::reverseSelect, this, &CloudWindow::reverseSelect);
        connect(overlay_, &SelectionOverlay::deleteSelectedPoints, this, &CloudWindow::deleteSelectedPoints);
        
        // 默认模式
        setInteractionMode(InteractionMode::View);

        // 7. 关联 CloudTreeWidget
        cloud_tree_ = cloud_tree;
        connect(cloud_tree, &CloudTreeWidget::cloudDeleted, this, &CloudWindow::onCloudDeleted);
        connect(cloud_tree, &CloudTreeWidget::cloudVisibilityChanged, this, &CloudWindow::onCloudVisibilityChanged);

        // 8. 设置上下文菜单策略
        setContextMenuPolicy(Qt::CustomContextMenu);
        connect(this, &CloudWindow::customContextMenuRequested,
                this, &CloudWindow::onCustomContextMenuRequested);
        
    }

    CloudWindow::~CloudWindow() {
        // PCL智能指针会自动管理，但最好显式置空
        viewer_ = nullptr; 
    }

    void CloudWindow::setCloud(std::shared_ptr<PointCloud> cloud) {
        // cloud_manager_ = cloud;
    }

    void CloudWindow::setPointCount(size_t pointCount) {
        if (!point_count_label_) {
            return;
        }
        point_count_label_->setText(tr("点数：%1").arg(pointCount));
        point_count_label_->adjustSize();
        point_count_label_->move(12, height() - point_count_label_->height() - 12);
        point_count_label_->raise();
    }

    void CloudWindow::setSelectedCloudCoordinates(const QString& name, float x, float y, float z) {
        setSelectedCloudMessage(tr("%1 坐标：X=%2  Y=%3  Z=%4")
            .arg(name)
            .arg(x, 0, 'f', 3)
            .arg(y, 0, 'f', 3)
            .arg(z, 0, 'f', 3));
    }

    void CloudWindow::setSelectedCloudMessage(const QString& message) {
        if (!coordinate_label_) {
            return;
        }
        coordinate_label_->setText(message);
        coordinate_label_->adjustSize();
        coordinate_label_->move(12, height() - point_count_label_->height() - coordinate_label_->height() - 18);
        coordinate_label_->raise();
    }

    void CloudWindow::clearSelectedCloudCoordinates() {
        if (!coordinate_label_) {
            return;
        }
        coordinate_label_->setText(tr("坐标：--"));
        coordinate_label_->adjustSize();
        coordinate_label_->move(12, height() - point_count_label_->height() - coordinate_label_->height() - 18);
        coordinate_label_->raise();
        clearPickedPointMarker();
    }

    void CloudWindow::setErrorLegend(const QString& title, double maxAbsError) {
        if (!legend_panel_ || !legend_title_label_ || !legend_bar_label_ ||
            !legend_top_label_ || !legend_mid_label_ || !legend_bottom_label_) {
            return;
        }
        Q_UNUSED(maxAbsError);
        legend_title_label_->setText(title);
        legend_bar_label_->setPixmap(createLegendPixmap(legend_bar_label_->width(),
                                                        legend_bar_label_->height()));
        legend_top_label_->setText(QString("范围: -1.0 ~ +1.0"));
        updateLegendPlacement();
        legend_panel_->show();
        legend_panel_->raise();
    }

    void CloudWindow::clearErrorLegend() {
        if (!legend_panel_) {
            return;
        }
        legend_panel_->hide();
    }

    void CloudWindow::updateLegendPlacement() {
        if (!legend_panel_) {
            return;
        }
        const int x = std::max(12, width() - legend_panel_->width() - 12);
        const int y = std::max(12, (height() - legend_panel_->height()) / 2);
        legend_panel_->move(x, y);
        legend_panel_->raise();
    }

    void CloudWindow::updatePickedPointCoordinates(int x, int y) {
        if (!point_picker_ || !viewer_ || !vtk_widget_ || !cloud_tree_) {
            return;
        }
        vtkRenderer* renderer = viewer_->getRenderWindow()->GetRenderers()->GetFirstRenderer();
        if (!renderer) {
            return;
        }
        const int pickY = std::max(0, vtk_widget_->height() - y - 1);
        if (!point_picker_->Pick(x, pickY, 0.0, renderer)) {
            clearPickedPointMarker();
            return;
        }
        double pickedPoint[3] = {0.0, 0.0, 0.0};
        point_picker_->GetPickPosition(pickedPoint);
        showPickedPointMarker(static_cast<float>(pickedPoint[0]),
                              static_cast<float>(pickedPoint[1]),
                              static_cast<float>(pickedPoint[2]));
        auto currentCloud = cloud_tree_->getCurrentCloud();
        if (currentCloud) {
            setSelectedCloudCoordinates(currentCloud->getName(),
                                        static_cast<float>(pickedPoint[0]),
                                        static_cast<float>(pickedPoint[1]),
                                        static_cast<float>(pickedPoint[2]));
            return;
        }
        setSelectedCloudMessage(tr("坐标：X=%1  Y=%2  Z=%3")
            .arg(pickedPoint[0], 0, 'f', 3)
            .arg(pickedPoint[1], 0, 'f', 3)
            .arg(pickedPoint[2], 0, 'f', 3));
    }

    void CloudWindow::showPickedPointMarker(float x, float y, float z) {
        if (!viewer_) {
            return;
        }
        viewer_->removePointCloud(picked_point_marker_id_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pickedPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pickedPointCloud->push_back(pcl::PointXYZ(x, y, z));
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(pickedPointCloud, 255, 140, 0);
        viewer_->addPointCloud<pcl::PointXYZ>(pickedPointCloud, colorHandler, picked_point_marker_id_);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  2,
                                                  picked_point_marker_id_);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                  1.0,
                                                  picked_point_marker_id_);
        refreshView();
    }

    void CloudWindow::clearPickedPointMarker() {
        if (!viewer_) {
            return;
        }
        viewer_->removePointCloud(picked_point_marker_id_);
        refreshView();
    }

    void CloudWindow::resizeEvent(QResizeEvent* event) {
        QWidget::resizeEvent(event);
        if (point_count_label_) {
            point_count_label_->move(12, height() - point_count_label_->height() - 12);
            point_count_label_->raise();
        }
        if (coordinate_label_) {
            coordinate_label_->move(12, height() - point_count_label_->height() - coordinate_label_->height() - 18);
            coordinate_label_->raise();
        }
        updateLegendPlacement();
    }

    void CloudWindow::removeFitOverlayGroup(const QString& baseId) {
        if (!viewer_) {
            return;
        }
        viewer_->removeShape(baseId.toStdString());
        const QStringList suffixes = {
            "_axis", "_body", "_center",
            "_x_axis", "_y_axis", "_z_axis"
        };
        for (const auto& suffix : suffixes) {
            viewer_->removeShape((baseId + suffix).toStdString());
        }
    }

    void CloudWindow::clearFitOverlays() {
        if (!viewer_) {
            return;
        }
        for (const auto& baseId : fit_overlay_ids_) {
            removeFitOverlayGroup(baseId);
        }
        fit_overlay_ids_.clear();
        refreshView();
    }

    void CloudWindow::showCylinderFitOverlay(const QString& baseId,
                                             const Eigen::Vector3f& axisPoint,
                                             const Eigen::Vector3f& axis,
                                             double radius,
                                             double minAxisCoord,
                                             double maxAxisCoord) {
        if (!viewer_ || radius <= 0.0 || maxAxisCoord <= minAxisCoord) {
            return;
        }

        removeFitOverlayGroup(baseId);
        fit_overlay_ids_.removeAll(baseId);
        fit_overlay_ids_.append(baseId);
        FitOverlayState state;
        state.type = FitOverlayType::Cylinder;
        state.axisPoint = axisPoint;
        state.axis = axis;
        state.radius = radius;
        state.minAxisCoord = minAxisCoord;
        state.maxAxisCoord = maxAxisCoord;
        fit_overlay_cache_.insert(baseId, state);

        const Eigen::Vector3f axisDir = axis.normalized();
        const Eigen::Vector3f start = axisPoint + axisDir * static_cast<float>(minAxisCoord);
        const Eigen::Vector3f end = axisPoint + axisDir * static_cast<float>(maxAxisCoord);
        pcl::ModelCoefficients coeffs;
        coeffs.values.resize(7);
        coeffs.values[0] = start.x();
        coeffs.values[1] = start.y();
        coeffs.values[2] = start.z();
        coeffs.values[3] = axisDir.x();
        coeffs.values[4] = axisDir.y();
        coeffs.values[5] = axisDir.z();
        coeffs.values[6] = radius;

        viewer_->addLine(pcl::PointXYZ(start.x(), start.y(), start.z()),
                         pcl::PointXYZ(end.x(), end.y(), end.z()),
                         0.1, 0.1, 0.1,
                         (baseId + "_axis").toStdString());
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                             3.0,
                                             (baseId + "_axis").toStdString());

        viewer_->addCylinder(coeffs, (baseId + "_body").toStdString());
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                             pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                             (baseId + "_body").toStdString());
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                             0.1, 0.8, 0.2,
                                             (baseId + "_body").toStdString());
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                             0.85,
                                             (baseId + "_body").toStdString());
        refreshView();
    }

    void CloudWindow::showSphereFitOverlay(const QString& baseId,
                                           const Eigen::Vector3f& center,
                                           double radius) {
        if (!viewer_ || radius <= 0.0) {
            return;
        }

        removeFitOverlayGroup(baseId);
        fit_overlay_ids_.removeAll(baseId);
        fit_overlay_ids_.append(baseId);
        FitOverlayState state;
        state.type = FitOverlayType::Sphere;
        state.center = center;
        state.radius = radius;
        fit_overlay_cache_.insert(baseId, state);

        const double axisLength = std::max(radius * 1.2, 1.0);
        viewer_->addSphere(pcl::PointXYZ(center.x(), center.y(), center.z()),
                           radius,
                           0.2, 0.6, 1.0,
                           (baseId + "_body").toStdString());
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                             pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                             (baseId + "_body").toStdString());
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                             0.85,
                                             (baseId + "_body").toStdString());

        viewer_->addSphere(pcl::PointXYZ(center.x(), center.y(), center.z()),
                           radius * 0.03,
                           1.0, 0.2, 0.2,
                           (baseId + "_center").toStdString());
        viewer_->addLine(pcl::PointXYZ(center.x() - axisLength, center.y(), center.z()),
                         pcl::PointXYZ(center.x() + axisLength, center.y(), center.z()),
                         1.0, 0.2, 0.2,
                         (baseId + "_x_axis").toStdString());
        viewer_->addLine(pcl::PointXYZ(center.x(), center.y() - axisLength, center.z()),
                         pcl::PointXYZ(center.x(), center.y() + axisLength, center.z()),
                         0.2, 0.8, 0.2,
                         (baseId + "_y_axis").toStdString());
        viewer_->addLine(pcl::PointXYZ(center.x(), center.y(), center.z() - axisLength),
                         pcl::PointXYZ(center.x(), center.y(), center.z() + axisLength),
                         0.2, 0.4, 1.0,
                         (baseId + "_z_axis").toStdString());
        refreshView();
    }

    void CloudWindow::registerFitOverlayId(const QString& baseId) {
        if (baseId.isEmpty()) {
            return;
        }
        fit_overlay_ids_.removeAll(baseId);
        fit_overlay_ids_.append(baseId);
    }

    void CloudWindow::setFitOverlayGroupForCloud(const QString& cloudId, const QString& fitGroupKey) {
        if (cloudId.isEmpty() || fitGroupKey.isEmpty()) {
            return;
        }
        fit_overlay_group_by_cloud_.insert(cloudId, fitGroupKey);
    }

    void CloudWindow::setInteractionMode(InteractionMode mode) {
        // 1. 设置 Overlay 的模式
        overlay_->setMode(mode);
        // qDebug() << ">>> setInteractionMode" << (int)mode;
        // 2. 控制事件穿透
        if (mode == InteractionMode::View) {
            // 浏览模式：Overlay 隐藏或不拦截事件，让 VTK 接收鼠标
            overlay_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
            overlay_->setFocusPolicy(Qt::NoFocus);
            vtk_widget_->raise();
            vtk_widget_->repaint();
            qDebug() << ">>> Mouse events transparent:" << overlay_->testAttribute(Qt::WA_TransparentForMouseEvents);
        } else {
            // 选择模式：Overlay 拦截事件，VTK 暂停交互
            overlay_->setAttribute(Qt::WA_TransparentForMouseEvents, false);
            overlay_->setFocusPolicy(Qt::StrongFocus); // 获取键盘焦点(为了多边形Enter)
            overlay_->setFocus();
            overlay_->raise();
            overlay_->repaint();
            qDebug() << ">>> Mouse events transparent:" << overlay_->testAttribute(Qt::WA_TransparentForMouseEvents);
        }
    }

    void CloudWindow::onSelectionFinished(const std::vector<QPoint>& polygon) {
        if (!cloud_tree_) {
            qDebug() << ">>> Error: CloudTree not set or destructed!";
            return;
        }

        qDebug() << ">>> Selection finished with " << polygon.size() << " points.";

        auto cloud = cloud_tree_->getCurrentCloud();
        if (!cloud) {
            qDebug() << ">>> Error: No cloud selected in CloudTree!";
            return;
        }

        // 检测 Shift 键是否按下 (实现加选)
        bool is_shift = QApplication::keyboardModifiers() & Qt::ShiftModifier;
        cloud->selectByScreenPolygon(polygon, viewer_, is_shift);
        cloud->render(viewer_);
        
        // 刷新 VTK 界面
        refreshView();
    }

    //聚焦到某一点云位置的槽函数
    void CloudWindow::onFocusCloud(float x, float y, float z){
        //聚焦到新的点云需要保证两点：
        //1. pcl_viewer的相机位置和焦点位置不变
        //2. 保持实现方向和距离不变

        //获取当前的相机参数
        std::vector<pcl::visualization::Camera> cameras;
        viewer_->getCameras(cameras);
        if (cameras.empty()) return;
        auto camera = cameras.front(); 

        //计算当前的视线向量
        double dx = camera.focal[0] - camera.pos[0];
        double dy = camera.focal[1] - camera.pos[1];
        double dz = camera.focal[2] - camera.pos[2];
        // double distance = std::sqrt(dx * dx + dy * dy + dz * dz);  //距离

        //保持实现方向和距离不变，平移相机
        //新的焦点 = (x, y, z)
        //新的相机位置 = 点云中心 - 视线向量 * 距离
        double newPos[3];
        newPos[0] = x - dx;
        newPos[1] = y - dy;
        newPos[2] = z - dz;

        viewer_->setCameraPosition(
            newPos[0], newPos[1], newPos[2],
            x, y, z,
            camera.view[0], camera.view[1], camera.view[2]
        );
        refreshView();
    }

    void CloudWindow::onFocusCloudOrthographic(float x, float y, float z){
        vtkRenderer* renderer = viewer_->getRenderWindow()->GetRenderers()->GetFirstRenderer();
        if (!renderer) {
            return;
        }
        renderer->GetActiveCamera()->ParallelProjectionOn();
        const double distance = 1000.0;
        viewer_->setCameraPosition(
            x, y, z + distance,
            x, y, z,
            0.0, 1.0, 0.0
        );
        viewer_->resetCamera();
        renderer->GetActiveCamera()->ParallelProjectionOn();
        refreshView();
    }

    //全选
    void CloudWindow::selectAllPoints(){
        if(cloud_tree_){
            auto cloud = cloud_tree_->getCurrentCloud();
            if(cloud){
                cloud->selectAll();
                cloud->render(viewer_);
                refreshView();
            }
        }
    }

    //反选
    void CloudWindow::reverseSelect(){
        if(cloud_tree_){
            auto cloud = cloud_tree_->getCurrentCloud();
            if(cloud){
                cloud->invertSelection();
                cloud->render(viewer_);
                refreshView();
            }
        }
    }

    //取消选择
    void CloudWindow::clearSelection(){
        if(cloud_tree_){
            auto cloud = cloud_tree_->getCurrentCloud();
            if(cloud){
                cloud->selectNone();
                cloud->render(viewer_);
                refreshView();
            }
        }
    }

    //删除选中点
    void CloudWindow::deleteSelectedPoints(){
        if(cloud_tree_){
            auto cloud = cloud_tree_->getCurrentCloud();
            if(cloud){
                cloud->deleteSelectedPoints();
                cloud->render(viewer_);
                refreshView();
            }
        }
    }

    void CloudWindow::onCustomContextMenuRequested(const QPoint &pos){
        // 创建菜单
        QMenu menu(this);
        
        // 反选
        QAction* reverse_select_action = menu.addAction(QIcon(":/icons/选择 反向.png"), "反选");
        connect(reverse_select_action, &QAction::triggered, this, &CloudWindow::reverseSelect);

        // 取消选择
        QAction* clear_select_action = menu.addAction(QIcon(":/icons/取消选择.png"), "取消选择");
        connect(clear_select_action, &QAction::triggered, this, &CloudWindow::clearSelection);

        // 删除选中点
        QAction* delete_select_action = menu.addAction(QIcon(":/icons/删除选中.png"), "删除");
        connect(delete_select_action, &QAction::triggered, this, &CloudWindow::deleteSelectedPoints);

        // 全选
        QAction* select_all_action = menu.addAction(QIcon(":/icons/全选.png"), "全选");
        connect(select_all_action, &QAction::triggered, this, &CloudWindow::selectAllPoints);

        // 在鼠标位置弹出菜单
        menu.exec(mapToGlobal(pos));
    }

    void CloudWindow::onCloudDeleted(QString cloudId){
        qDebug() << ">>> Cloud deleted: " << cloudId;
        std::string main_id = cloudId.toStdString();
        std::string sel_id = main_id + "_selected";
        viewer_->removePointCloud(main_id);
        viewer_->removePointCloud(sel_id);
        clearPickedPointMarker();
        const QString fitGroupKey = fitGroupKeyForCloud(cloudId);
        removeDisplayedFitOverlaysForGroup(fitGroupKey);
        removeFitOverlayCacheForCloud(cloudId);
        removeFitOverlayGroupBinding(cloudId);

        refreshView();
    }

    void CloudWindow::onCloudVisibilityChanged(QString cloudId, bool visible) {
        if (visible) {
            restoreFitOverlaysForCloud(cloudId);
            refreshView();
            return;
        }

        const QString fitGroupKey = fitGroupKeyForCloud(cloudId);
        removeDisplayedFitOverlaysForGroup(fitGroupKey);
        refreshView();
    }

    QString CloudWindow::fitGroupKeyForCloud(const QString& cloudId) const {
        return fit_overlay_group_by_cloud_.value(cloudId);
    }

    void CloudWindow::removeDisplayedFitOverlaysForGroup(const QString& fitGroupKey) {
        if (fitGroupKey.isEmpty()) {
            return;
        }
        for (int i = fit_overlay_ids_.size() - 1; i >= 0; --i) {
            if (!fit_overlay_ids_.at(i).startsWith(fitGroupKey)) {
                continue;
            }
            removeFitOverlayGroup(fit_overlay_ids_.at(i));
            fit_overlay_ids_.removeAt(i);
        }
    }

    void CloudWindow::restoreFitOverlaysForCloud(const QString& cloudId) {
        const QString fitPrefix = fitGroupKeyForCloud(cloudId);
        if (fitPrefix.isEmpty()) {
            return;
        }
        const auto cacheKeys = fit_overlay_cache_.keys();
        for (const auto& baseId : cacheKeys) {
            if (!baseId.startsWith(fitPrefix)) {
                continue;
            }
            const FitOverlayState state = fit_overlay_cache_.value(baseId);
            if (state.type == FitOverlayType::Cylinder) {
                showCylinderFitOverlay(baseId,
                                       state.axisPoint,
                                       state.axis,
                                       state.radius,
                                       state.minAxisCoord,
                                       state.maxAxisCoord);
            } else {
                showSphereFitOverlay(baseId,
                                     state.center,
                                     state.radius);
            }
        }
    }

    void CloudWindow::removeFitOverlayCacheForCloud(const QString& cloudId) {
        const QString fitPrefix = fitGroupKeyForCloud(cloudId);
        if (fitPrefix.isEmpty()) {
            removeFitOverlayGroupBinding(cloudId);
            return;
        }
        const auto cacheKeys = fit_overlay_cache_.keys();
        for (const auto& baseId : cacheKeys) {
            if (baseId.startsWith(fitPrefix)) {
                fit_overlay_cache_.remove(baseId);
            }
        }
        removeFitOverlayBindingsForGroup(fitPrefix);
    }

    void CloudWindow::removeFitOverlayGroupBinding(const QString& cloudId) {
        fit_overlay_group_by_cloud_.remove(cloudId);
    }

    void CloudWindow::removeFitOverlayBindingsForGroup(const QString& fitGroupKey) {
        const auto cloudIds = fit_overlay_group_by_cloud_.keys();
        for (const auto& id : cloudIds) {
            if (fit_overlay_group_by_cloud_.value(id) == fitGroupKey) {
                fit_overlay_group_by_cloud_.remove(id);
            }
        }
    }
}}
