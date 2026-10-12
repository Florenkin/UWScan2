#include "pointcloud/cloudtreewidget.h"
#include "pointcloud/filterdialog.h"

#include <QApplication>
#include <QColorDialog>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QHeaderView>
#include <QMenu>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QStyle>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

NAMESPACE_BEGIN { namespace pointcloud {
    struct PointCloudResolutionStats {
        bool ok = false;
        double average = 0.0;
        double minimum = 0.0;
        double maximum = 0.0;
        int sampleCount = 0;
    };

    static PointCloudResolutionStats estimatePointCloudResolution(const std::vector<Eigen::Vector3f>& points,
                                                                  int maxSamples = 5000) {
        PointCloudResolutionStats stats;
        if (points.size() < 2 || maxSamples <= 0) {
            return stats;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclCloud->reserve(points.size());
        for (const auto& point : points) {
            if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
                continue;
            }
            pclCloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        }
        if (pclCloud->size() < 2) {
            return stats;
        }
        pclCloud->width = static_cast<uint32_t>(pclCloud->size());
        pclCloud->height = 1;
        pclCloud->is_dense = false;

        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(pclCloud);

        const size_t stride = std::max<size_t>(1, pclCloud->size() / static_cast<size_t>(maxSamples));
        double sum = 0.0;
        double minimum = std::numeric_limits<double>::max();
        double maximum = 0.0;
        int sampleCount = 0;
        std::vector<int> indices(2);
        std::vector<float> squaredDistances(2);
        for (size_t i = 0; i < pclCloud->size() && sampleCount < maxSamples; i += stride) {
            const int found = tree.nearestKSearch(pclCloud->points[i], 2, indices, squaredDistances);
            if (found < 2 || squaredDistances[1] < 0.0f) {
                continue;
            }
            const double distance = std::sqrt(static_cast<double>(squaredDistances[1]));
            sum += distance;
            minimum = std::min(minimum, distance);
            maximum = std::max(maximum, distance);
            ++sampleCount;
        }

        if (sampleCount <= 0) {
            return stats;
        }
        stats.ok = true;
        stats.average = sum / static_cast<double>(sampleCount);
        stats.minimum = minimum;
        stats.maximum = maximum;
        stats.sampleCount = sampleCount;
        return stats;
    }

    CloudTreeWidget::CloudTreeWidget(QWidget *parent) : QTreeWidget(parent) {
        setHeaderHidden(true);
        setColumnCount(1);
        setIndentation(10);
        setSelectionMode(QAbstractItemView::ExtendedSelection);
        setSelectionBehavior(QAbstractItemView::SelectRows);
        setContextMenuPolicy(Qt::CustomContextMenu);

        connect(this, &QTreeWidget::customContextMenuRequested,
                this, &CloudTreeWidget::onCustomContextMenuRequested);
        connect(this, &QTreeWidget::itemDoubleClicked,
                this, &CloudTreeWidget::focusCloudFromItem);
    }

    void CloudTreeWidget::addCloud(std::shared_ptr<PointCloud> cloud) {
        if (!cloud) {
            return;
        }
        auto* item = new CloudTreeItem(this, cloud);
        setCurrentItem(item);
    }

    bool CloudTreeWidget::appendCloud(std::shared_ptr<PointCloud> src, const QString& dst_cloud_name) {
        if (!src) {
            return false;
        }

        QTreeWidgetItemIterator it(const_cast<CloudTreeWidget*>(this));
        while (*it) {
            auto* cItem = dynamic_cast<CloudTreeItem*>(*it);
            if (cItem) {
                auto cloud = cItem->getCloud();
                if (cloud && cloud->getName().contains(dst_cloud_name, Qt::CaseInsensitive)) {
                    cloud->merge(src);
                    cItem->updateAppearence();
                    setCurrentItem(cItem);
                    return true;
                }
            }
            ++it;
        }

        addCloud(src);
        return false;
    }

    std::shared_ptr<PointCloud> CloudTreeWidget::getCurrentCloud() const {
        auto* item = dynamic_cast<CloudTreeItem*>(currentItem());
        return item ? item->getCloud() : nullptr;
    }

    std::vector<std::shared_ptr<PointCloud>> CloudTreeWidget::getSelectedClouds() const {
        std::vector<std::shared_ptr<PointCloud>> list;
        for (auto* item : selectedItems()) {
            auto* cItem = dynamic_cast<CloudTreeItem*>(item);
            if (cItem) {
                list.push_back(cItem->getCloud());
            }
        }
        return list;
    }

    std::vector<std::shared_ptr<PointCloud>> CloudTreeWidget::getAllClouds() const {
        std::vector<std::shared_ptr<PointCloud>> list;
        QTreeWidgetItemIterator it(const_cast<CloudTreeWidget*>(this));
        while (*it) {
            auto* cItem = dynamic_cast<CloudTreeItem*>(*it);
            if (cItem) {
                list.push_back(cItem->getCloud());
            }
            ++it;
        }
        return list;
    }

    void CloudTreeWidget::setCloudExtraInfo(const QString& cloudId, const QString& info) {
        cloud_extra_infos_.insert(cloudId, info);
    }

    void CloudTreeWidget::clearCloudExtraInfo(const QString& cloudId) {
        cloud_extra_infos_.remove(cloudId);
    }

    void CloudTreeWidget::updateCloudItemAppearance(const QString& cloudId) {
        QTreeWidgetItemIterator it(this);
        while (*it) {
            auto* cItem = dynamic_cast<CloudTreeItem*>(*it);
            if (cItem) {
                auto cloud = cItem->getCloud();
                if (cloud && QString::fromStdString(cloud->getId()) == cloudId) {
                    cItem->updateAppearence();
                    return;
                }
            }
            ++it;
        }
    }

    void CloudTreeWidget::setCloudGroupForCloud(const QString& cloudId, const QString& groupKey) {
        if (cloudId.isEmpty()) {
            return;
        }
        cloud_groups_.insert(cloudId, groupKey.isEmpty() ? cloudId : groupKey);
    }

    bool CloudTreeWidget::setCloudVisibility(const QString& cloudId, bool visible) {
        QTreeWidgetItemIterator it(this);
        while (*it) {
            auto* cloudItem = dynamic_cast<CloudTreeItem*>(*it);
            if (!cloudItem) {
                ++it;
                continue;
            }

            auto cloud = cloudItem->getCloud();
            if (!cloud || QString::fromStdString(cloud->getId()) != cloudId) {
                ++it;
                continue;
            }

            if (cloud->isVisible() == visible) {
                return false;
            }

            cloud->setVisible(visible);
            cloudItem->updateAppearence();
            emit cloudVisibilityChanged(cloudId, visible);
            emit refreshViewRequested();
            return true;
        }
        return false;
    }

    bool CloudTreeWidget::setSelectedCloudsVisibility(bool visible) {
        QList<QTreeWidgetItem*> selected = selectedItems();
        if (selected.isEmpty() && context_item_) {
            selected.push_back(context_item_);
        }

        bool changed = false;
        for (auto* item : selected) {
            auto* cloudItem = dynamic_cast<CloudTreeItem*>(item);
            if (!cloudItem) {
                continue;
            }
            auto cloud = cloudItem->getCloud();
            if (!cloud) {
                continue;
            }
            changed = setCloudVisibility(QString::fromStdString(cloud->getId()), visible) || changed;
        }
        return changed;
    }

    QString CloudTreeWidget::cloudGroupKey(const QString& cloudId) const {
        const auto it = cloud_groups_.find(cloudId);
        return it == cloud_groups_.end() ? cloudId : it.value();
    }

    void CloudTreeWidget::removeCloudGroupBinding(const QString& cloudId) {
        cloud_groups_.remove(cloudId);
    }

    void CloudTreeWidget::removeCloudGroupBindingsByGroup(const QString& groupKey) {
        if (groupKey.isEmpty()) {
            return;
        }
        for (auto it = cloud_groups_.begin(); it != cloud_groups_.end();) {
            if (it.value() == groupKey) {
                it = cloud_groups_.erase(it);
            } else {
                ++it;
            }
        }
    }

    void CloudTreeWidget::onCustomContextMenuRequested(const QPoint& pos) {
        auto* item = itemAt(pos);
        context_item_ = dynamic_cast<CloudTreeItem*>(item);
        if (!context_item_) {
            return;
        }

        if (!context_item_->isSelected()) {
            clearSelection();
            context_item_->setSelected(true);
            setCurrentItem(context_item_);
        }

        {
            QMenu menu(this);
            const int selectedCloudCount = selectedItems().size();

            QAction* actShowSelected = menu.addAction(selectedCloudCount > 1 ? tr("显示选中点云") : tr("显示点云"));
            connect(actShowSelected, &QAction::triggered, this, &CloudTreeWidget::showSelectedClouds);

            QAction* actHideSelected = menu.addAction(selectedCloudCount > 1 ? tr("隐藏选中点云") : tr("隐藏点云"));
            connect(actHideSelected, &QAction::triggered, this, &CloudTreeWidget::hideSelectedClouds);

            QAction* actInfo = menu.addAction(tr("属性信息"));
            connect(actInfo, &QAction::triggered, this, &CloudTreeWidget::showInfo);

            QAction* actFocus = menu.addAction(tr("聚焦查看"));
            actFocus->setIcon(QApplication::style()->standardIcon(QStyle::SP_FileDialogContentsView));
            connect(actFocus, &QAction::triggered, this, [this]() {
                if (!context_item_) {
                    return;
                }
                auto cloud = context_item_->getCloud();
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;
                cloud->getCentroid(x, y, z);
                emit focusOnCloudRequest(x, y, z);
            });

            menu.addSeparator();

            QAction* actDel = menu.addAction(selectedCloudCount > 1 ? tr("删除选中点云") : tr("删除点云"));
            actDel->setIcon(QApplication::style()->standardIcon(QStyle::SP_TrashIcon));
            connect(actDel, &QAction::triggered, this, &CloudTreeWidget::deleteSelectedClouds);

            QAction* actColor = menu.addAction(tr("修改颜色"));
            connect(actColor, &QAction::triggered, this, [this]() {
                if (!context_item_) {
                    return;
                }
                auto cloud = context_item_->getCloud();
                auto cloudColor = cloud->getColor();
                const QColor currentColor(cloudColor.components.r, cloudColor.components.g, cloudColor.components.b);
                const QColor color = QColorDialog::getColor(currentColor, this, tr("选择颜色"));
                if (!color.isValid()) {
                    return;
                }
                const ColorRGB newColor {color.red(), color.green(), color.blue(), 255};
                cloud->setColor(newColor);
                context_item_->updateAppearence();
                emit refreshViewRequested();
            });

            QAction* actSave = menu.addAction(tr("保存点云"));
            connect(actSave, &QAction::triggered, this, [this]() {
                if (!context_item_) {
                    return;
                }
                auto cloud = context_item_->getCloud();
                const QString savePath = QFileDialog::getSaveFileName(
                    this,
                    tr("保存点云"),
                    QDir::homePath() + "/" + cloud->getName(),
                    tr("点云文件(*.ply *.pcd *.xyz *.las *.txt *.asc)")
                );
                if (savePath.isEmpty()) {
                    return;
                }
                cloud->saveToFile(savePath);
                QMessageBox::information(this, tr("提示"), tr("点云保存完成"));
            });

            QAction* actFilter = menu.addAction(tr("滤波"));
            connect(actFilter, &QAction::triggered, this, &CloudTreeWidget::filterSelectedClouds);

            QAction* actMerge = menu.addAction(tr("合并点云"));
            connect(actMerge, &QAction::triggered, this, &CloudTreeWidget::mergeSelectedClouds);
            if (selectedCloudCount < 2) {
                actMerge->setEnabled(false);
            }

            QAction* actSaveError = menu.addAction(tr("保存误差数据"));
            connect(actSaveError, &QAction::triggered, this, [this]() {
                if (!context_item_) {
                    return;
                }
                auto cloud = context_item_->getCloud();
                if (!cloud) {
                    return;
                }
                emit saveErrorDataRequested(QString::fromStdString(cloud->getId()));
            });

            QAction* actModifyStandard = menu.addAction(tr("修改标准直径"));
            connect(actModifyStandard, &QAction::triggered, this, [this]() {
                if (!context_item_) {
                    return;
                }
                auto cloud = context_item_->getCloud();
                if (!cloud) {
                    return;
                }
                emit modifyStandardDiameterRequested(QString::fromStdString(cloud->getId()));
            });

            menu.exec(mapToGlobal(pos));
            return;
        }

        QMenu menu(this);
        const int selectedCloudCount = selectedItems().size();
        const bool isVisible = context_item_->getCloud()->isVisible();

        QAction* actToggle = menu.addAction(isVisible ? tr("隐藏点云") : tr("显示点云"));
        connect(actToggle, &QAction::triggered, this, &CloudTreeWidget::toggleVisibility);

        QAction* actShowSelected = menu.addAction(selectedCloudCount > 1 ? tr("显示选中点云") : tr("显示点云"));
        connect(actShowSelected, &QAction::triggered, this, &CloudTreeWidget::showSelectedClouds);

        QAction* actHideSelected = menu.addAction(selectedCloudCount > 1 ? tr("隐藏选中点云") : tr("隐藏点云"));
        connect(actHideSelected, &QAction::triggered, this, &CloudTreeWidget::hideSelectedClouds);

        QAction* actInfo = menu.addAction(tr("属性信息"));
        connect(actInfo, &QAction::triggered, this, &CloudTreeWidget::showInfo);

        QAction* actFocus = menu.addAction(tr("聚焦查看"));
        actFocus->setIcon(QApplication::style()->standardIcon(QStyle::SP_FileDialogContentsView));
        connect(actFocus, &QAction::triggered, this, [this]() {
            if (!context_item_) {
                return;
            }
            auto cloud = context_item_->getCloud();
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            cloud->getCentroid(x, y, z);
            emit focusOnCloudRequest(x, y, z);
        });

        menu.addSeparator();

        QAction* actDel = menu.addAction(selectedCloudCount > 1 ? tr("删除选中点云") : tr("删除点云"));
        actDel->setIcon(QApplication::style()->standardIcon(QStyle::SP_TrashIcon));
        connect(actDel, &QAction::triggered, this, &CloudTreeWidget::deleteSelectedClouds);

        QAction* actColor = menu.addAction(tr("修改颜色"));
        connect(actColor, &QAction::triggered, this, [this]() {
            if (!context_item_) {
                return;
            }
            auto cloud = context_item_->getCloud();
            auto cloudColor = cloud->getColor();
            const QColor currentColor(cloudColor.components.r, cloudColor.components.g, cloudColor.components.b);
            const QColor color = QColorDialog::getColor(currentColor, this, tr("选择颜色"));
            if (!color.isValid()) {
                return;
            }
            const ColorRGB newColor {color.red(), color.green(), color.blue(), 255};
            cloud->setColor(newColor);
            context_item_->updateAppearence();
            emit refreshViewRequested();
        });

        QAction* actSave = menu.addAction(tr("保存点云"));
        connect(actSave, &QAction::triggered, this, [this]() {
            if (!context_item_) {
                return;
            }
            auto cloud = context_item_->getCloud();
            const QString savePath = QFileDialog::getSaveFileName(
                this,
                tr("保存点云"),
                QDir::homePath() + "/" + cloud->getName(),
                tr("点云文件(*.ply *.pcd *.xyz *.las *.txt *.asc)")
            );
            if (savePath.isEmpty()) {
                return;
            }
            cloud->saveToFile(savePath);
            QMessageBox::information(this, tr("提示"), tr("点云保存完成"));
        });

        QAction* actFilter = menu.addAction(tr("滤波"));
        connect(actFilter, &QAction::triggered, this, &CloudTreeWidget::filterSelectedClouds);

        QAction* actMerge = menu.addAction(tr("合并点云"));
        connect(actMerge, &QAction::triggered, this, &CloudTreeWidget::mergeSelectedClouds);
        if (selectedCloudCount < 2) {
            actMerge->setEnabled(false);
        }

        QAction* actSaveError = menu.addAction(tr("保存误差数据"));
        connect(actSaveError, &QAction::triggered, this, [this]() {
            if (!context_item_) {
                return;
            }
            auto cloud = context_item_->getCloud();
            if (!cloud) {
                return;
            }
            emit saveErrorDataRequested(QString::fromStdString(cloud->getId()));
        });

        QAction* actModifyStandard = menu.addAction(tr("修改标准直径"));
        connect(actModifyStandard, &QAction::triggered, this, [this]() {
            if (!context_item_) {
                return;
            }
            auto cloud = context_item_->getCloud();
            if (!cloud) {
                return;
            }
            emit modifyStandardDiameterRequested(QString::fromStdString(cloud->getId()));
        });

        menu.exec(mapToGlobal(pos));
    }

    void CloudTreeWidget::toggleVisibility() {
        if (!context_item_) {
            return;
        }
        auto cloud = context_item_->getCloud();
        if (!cloud) {
            return;
        }
        setCloudVisibility(QString::fromStdString(cloud->getId()), !cloud->isVisible());
    }

    void CloudTreeWidget::showSelectedClouds() {
        setSelectedCloudsVisibility(true);
    }

    void CloudTreeWidget::hideSelectedClouds() {
        setSelectedCloudsVisibility(false);
    }

    void CloudTreeWidget::showInfo() {
        if (!context_item_) {
            return;
        }
        auto cloud = context_item_->getCloud();

        QString info;
        info += tr("名称: ") + cloud->getName() + "\n";
        info += tr("ID: ") + QString::fromStdString(cloud->getId()) + "\n";
        info += tr("总点数: ") + QString::number(cloud->getPointCount()) + "\n";
        info += tr("选中点数: ") + QString::number(cloud->getSelectedCount()) + "\n";
        info += tr("状态: ") + QString(cloud->isVisible() ? tr("可见") : tr("隐藏")) + "\n";

        std::vector<Eigen::Vector3f> points;
        cloud->getPoints(points);
        const PointCloudResolutionStats resolution = estimatePointCloudResolution(points);
        if (resolution.ok) {
            info += tr("点云分辨率(平均最近邻距离): %1\n")
                .arg(resolution.average, 0, 'f', 6);
            info += tr("分辨率范围: 最小 %1, 最大 %2\n")
                .arg(resolution.minimum, 0, 'f', 6)
                .arg(resolution.maximum, 0, 'f', 6);
            info += tr("分辨率采样点数: %1\n")
                .arg(resolution.sampleCount);
        } else {
            info += tr("点云分辨率: 点数不足，无法计算\n");
        }
        const QString cloudId = QString::fromStdString(cloud->getId());
        if (cloud_extra_infos_.contains(cloudId) && !cloud_extra_infos_.value(cloudId).isEmpty()) {
            info += "\n" + cloud_extra_infos_.value(cloudId);
        }

        QDialog dialog(this);
        dialog.setWindowTitle(tr("点云属性"));
        dialog.setMinimumSize(420, 320);
        dialog.resize(720, 520);

        auto* layout = new QVBoxLayout(&dialog);
        auto* textEdit = new QPlainTextEdit(&dialog);
        textEdit->setReadOnly(true);
        textEdit->setPlainText(info);
        textEdit->setLineWrapMode(QPlainTextEdit::NoWrap);
        textEdit->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        textEdit->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        layout->addWidget(textEdit);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok, &dialog);
        connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
        layout->addWidget(buttons);

        dialog.exec();
    }

    void CloudTreeWidget::deleteCloud() {
        deleteSelectedClouds();
    }

    void CloudTreeWidget::deleteSelectedClouds() {
        QList<QTreeWidgetItem*> selected = selectedItems();
        if (selected.isEmpty() && context_item_) {
            selected.push_back(context_item_);
        }

        QList<CloudTreeItem*> seedItems;
        for (auto* item : selected) {
            if (auto* cloudItem = dynamic_cast<CloudTreeItem*>(item)) {
                if (!seedItems.contains(cloudItem)) {
                    seedItems.push_back(cloudItem);
                }
            }
        }
        if (seedItems.isEmpty()) {
            return;
        }

        QString message;
        if (seedItems.size() == 1) {
            auto* item = seedItems.first();
            auto cloud = item ? item->getCloud() : nullptr;
            message = cloud
                ? tr("确定要删除点云 \"%1\" 吗?").arg(cloud->getName())
                : tr("确定要删除当前点云吗?");
        } else {
            message = tr("确定要删除选中的 %1 个点云吗?").arg(seedItems.size());
        }

        const auto ret = QMessageBox::question(this,
                                               tr("确认删除"),
                                               message,
                                               QMessageBox::Yes | QMessageBox::No);
        if (ret != QMessageBox::Yes) {
            return;
        }

        for (auto* item : seedItems) {
            if (!item) {
                continue;
            }
            auto cloud = item->getCloud();
            if (cloud) {
                const QString cloudId = QString::fromStdString(cloud->getId());
                cloud->setVisible(false);
                cloud_extra_infos_.remove(cloudId);
                removeCloudGroupBinding(cloudId);
                emit cloudDeleted(cloudId);
            }
            delete item;
        }
        context_item_ = nullptr;
        emit refreshViewRequested();
    }

    void CloudTreeWidget::mergeSelectedClouds() {
        auto selectedItems = this->selectedItems();
        if (selectedItems.empty()) {
            QMessageBox::warning(this, tr("警告"), tr("请先选择要合并的点云"));
            return;
        }
        if (selectedItems.size() < 2 || !context_item_) {
            return;
        }

        std::shared_ptr<PointCloud> targetCloud = context_item_->getCloud();
        std::vector<std::shared_ptr<PointCloud>> sourceClouds;
        for (auto& item : selectedItems) {
            if (item == context_item_) {
                continue;
            }
            auto* cloudItem = dynamic_cast<CloudTreeItem*>(item);
            if (!cloudItem) {
                continue;
            }
            auto cloud = cloudItem->getCloud();
            if (cloud) {
                sourceClouds.push_back(cloud);
            }
        }

        auto mergedCloud = targetCloud->merge(sourceClouds);
        if (mergedCloud.empty()) {
            QMessageBox::warning(this, tr("警告"), tr("合并点云失败"));
            return;
        }

        std::unordered_set<std::string> mergedIds;
        for (const auto& cloud : mergedCloud) {
            mergedIds.insert(cloud->getId());
        }

        for (auto& item : selectedItems) {
            auto* cloudItem = dynamic_cast<CloudTreeItem*>(item);
            if (!cloudItem) {
                continue;
            }
            auto cloud = cloudItem->getCloud();
            if (!cloud || mergedIds.count(cloud->getId()) == 0) {
                continue;
            }
            cloud->setVisible(false);
            const QString cloudId = QString::fromStdString(cloud->getId());
            cloud_extra_infos_.remove(cloudId);
            removeCloudGroupBinding(cloudId);
            emit cloudDeleted(cloudId);
            delete item;
            item = nullptr;
        }

        emit refreshViewRequested();
    }

    void CloudTreeWidget::filterSelectedClouds() {
        auto selectedItems = this->selectedItems();
        if (selectedItems.empty()) {
            QMessageBox::warning(this, tr("警告"), tr("请先选择要滤波的点云"));
            return;
        }
        if (!context_item_) {
            return;
        }

        FilterDialog filterDialog(context_item_->getCloud(), this);
        filterDialog.exec();

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        context_item_->getCloud()->getCentroid(x, y, z);
        emit focusOnCloudRequest(x, y, z);
        emit refreshViewRequested();
    }

    void CloudTreeWidget::focusCloudFromItem(QTreeWidgetItem* item, int column) {
        Q_UNUSED(column);
        auto* cloudItem = dynamic_cast<CloudTreeItem*>(item);
        if (!cloudItem) {
            return;
        }
        auto cloud = cloudItem->getCloud();
        if (!cloud) {
            return;
        }
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        cloud->getCentroid(x, y, z);
        emit focusOnCloudRequest(x, y, z);
    }
}}
