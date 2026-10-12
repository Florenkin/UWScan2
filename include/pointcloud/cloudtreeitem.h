#ifndef CLOUDTREEITEM_H
#define CLOUDTREEITEM_H

#include <QTreeWidgetItem>
#include <memory>
#include "pc.hpp"
#include "utils.h"

NAMESPACE_BEGIN { namespace pointcloud {
    class CloudTreeItem : public QTreeWidgetItem {
    public:
        // 构造函数：传入 TreeWidget 父对象和点云管理器
        CloudTreeItem(QTreeWidget* parent, std::shared_ptr<PointCloud> cloud);

        // 获取持有的点云对象
        std::shared_ptr<PointCloud> getCloud() const { return cloud_; }

        // 根据点云状态刷新UI
        void updateAppearence();

    private:
        std::shared_ptr<PointCloud> cloud_;
    };
}}
#endif // CLOUDTREEITEM_H