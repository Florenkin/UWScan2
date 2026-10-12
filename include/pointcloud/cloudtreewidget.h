#ifndef CLOUDTREEWIDGET_H
#define CLOUDTREEWIDGET_H
#include <QTreeWidget>
#include <QHash>
#include "pointcloud/cloudtreeitem.h"

NAMESPACE_BEGIN { namespace pointcloud {
    class CloudTreeWidget : public QTreeWidget {
        Q_OBJECT

    public:
        explicit CloudTreeWidget(QWidget *parent = nullptr);

        /**
         * @brief 添加一个新的点云到树中
         * @param cloud 要添加的点云对象
         */
        void addCloud(std::shared_ptr<PointCloud> cloud);

        /**
         * @brief 将 src 点云追加到 dst_cloud_name 点云末尾
         * @param src 要追加的点云对象
         * @param dst_cloud_name 目标点云名称
         */
        bool appendCloud(std::shared_ptr<PointCloud> src, const QString & dst_cloud_name);
        
        /**
         * @brief 获取当前选中的点云 (用于主窗口判断对谁进行框选)
         * @return 当前选中的点云对象
         */
        std::shared_ptr<PointCloud> getCurrentCloud() const;

        /**
         * @brief 获取所有选中的点云 (用于批量操作)
         * @return 所有选中点云对象的向量
         */
        std::vector<std::shared_ptr<PointCloud>> getSelectedClouds() const;
        
        /**
         * @brief 获取所有点云列表 (用于遍历渲染)
         * @return 所有点云对象的向量
         */
        std::vector<std::shared_ptr<PointCloud>> getAllClouds() const;

        /**
         * @brief 设置点云额外属性信息
         * @param cloudId 点云ID
         * @param info 额外信息文本
         */
        void setCloudExtraInfo(const QString& cloudId, const QString& info);

        /**
         * @brief 清除点云额外属性信息
         * @param cloudId 点云ID
         */
        void clearCloudExtraInfo(const QString& cloudId);

        /**
         * @brief 刷新指定点云条目的外观
         * @param cloudId 点云ID
         */
        void updateCloudItemAppearance(const QString& cloudId);
        void setCloudGroupForCloud(const QString& cloudId, const QString& groupKey);
        bool setCloudVisibility(const QString& cloudId, bool visible);
        bool setSelectedCloudsVisibility(bool visible);

    public slots:
        /**
         * @brief 删除当前选中的一个或多个点云
         */
        void deleteSelectedClouds();

    signals:
        /**
         * @brief 信号：请求 3D 视图重绘 (当显隐状态改变、删除点云时触发)
         */
        void refreshViewRequested();
        
        /**
         * @brief 信号：点云被删除 (可选，如果主窗口需要做额外清理)
         * @param cloudId 被删除的点云的唯一标识符
         */
        void cloudDeleted(QString cloudId);

        /**
         * @brief 信号：请求将点云查看器聚焦到特定位置
         * @param x 目标位置的 X 坐标
         * @param y 目标位置的 Y 坐标
         * @param z 目标位置的 Z 坐标
         */
        void focusOnCloudRequest(float x, float y, float z);

        /**
         * @brief 信号：点云显隐状态发生变化
         * @param cloudId 点云ID
         * @param visible 是否可见
         */
        void cloudVisibilityChanged(QString cloudId, bool visible);
        void saveErrorDataRequested(QString cloudId);
        void modifyStandardDiameterRequested(QString cloudId);

    private slots:
        /**
         * @brief 自定义上下文菜单请求槽函数
         * @param pos 鼠标点击位置
         */
        void onCustomContextMenuRequested(const QPoint &pos);
        
        /**
         * @brief 菜单动作槽函数：切换点云可见性
         */
        void toggleVisibility();
        void showSelectedClouds();
        void hideSelectedClouds();
        
        /**
         * @brief 菜单动作槽函数：显示点云信息
         */
        void showInfo();
        
        /**
         * @brief 菜单动作槽函数：删除选中的点云
         */
        void deleteCloud();

        /**
         * @brief 菜单动作槽函数：合并选中的点云
         * @details 合并选中的点云，将它们合并到第一个选中的点云中，调用PointCloud的merge函数
         */
        void mergeSelectedClouds();

        /**
         * @brief 菜单动作槽函数：对选中的点云进行滤波
         */
        void filterSelectedClouds();

        /**
         * @brief 双击左侧点云条目时快速定位
         * @param item 被双击的树节点
         * @param column 列号
         */
        void focusCloudFromItem(QTreeWidgetItem* item, int column);

    private:
        // 临时存储右键点击时的 Item，供菜单动作使用
        QString cloudGroupKey(const QString& cloudId) const;
        void removeCloudGroupBinding(const QString& cloudId);
        void removeCloudGroupBindingsByGroup(const QString& groupKey);

        CloudTreeItem* context_item_ = nullptr;
        QHash<QString, QString> cloud_extra_infos_;
        QHash<QString, QString> cloud_groups_;
    };
}}

#endif
