#include "pointcloud/cloudtreeitem.h"
NAMESPACE_BEGIN { namespace pointcloud {
    CloudTreeItem::CloudTreeItem(QTreeWidget* parent, std::shared_ptr<PointCloud> cloud)
        : QTreeWidgetItem(parent), cloud_(cloud) {
        setText(0, cloud_->getName());
        updateAppearence();
    }

    // 根据点云状态刷新UI
    void CloudTreeItem::updateAppearence() {
        if(!cloud_) return;

        setText(0, QString("%1 (%2)")
            .arg(cloud_->getName())
            .arg(cloud_->getPointCount()));

        bool isVisible = cloud_->isVisible();
        if (isVisible) {
            setIcon(0, QIcon(":/icons/visible.png"));
            setTextColor(0, QColor(Qt::black));
        } else {
            setIcon(0, QIcon(":/icons/invisible.png"));
            setTextColor(0, QColor(Qt::gray));
        }
    }
}}
