#ifndef PC_H
#define PC_H
#include "utils.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>  //用以计算点云质心
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <QPoint>  //Qt二维平面点
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <Eigen/Geometry>
#include <QUuid>
#include <QApplication>
#include <QFileInfo>
#include <QDebug>
#include <QTextStream>
#include <fstream>
#include "pointcloud/importoptiondialog.h"


NAMESPACE_BEGIN { namespace pointcloud {
    /**
     * @brief 点云颜色结构体
     * 
     * 该结构体定义了点云的颜色，包括四个通道
     */
    struct ColorRGB{
        union {
            struct {
#ifdef BIG_ENDIAN 
                uint8_t a;
                uint8_t b;
                uint8_t g;
                uint8_t r;
#else 
                uint8_t r;
                uint8_t g;
                uint8_t b;
                uint8_t a;
#endif
            } components;
            uint32_t rgba = 0x000000FF;
        };
    };

    /**
     * @brief 点云接口类
     * 
     * 该类定义了点云的基本操作接口，包括选择、删除、渲染等。
     */
    class PointCloud {
    public:
        PointCloud() = default;
        virtual ~PointCloud() = default;

        /**
         * @brief 
         */
        virtual void getPoints(std::vector<Eigen::Vector3f>& points) = 0;

        /**
         * @brief 获取当前选中的点
         */
        virtual void getSelectedPoints(std::vector<Eigen::Vector3f>& points) = 0;

        /**
         * @brief 选择所有点
         */
        virtual void selectAll() = 0;
        
        /**
         * @brief 选择无点
         */
        virtual void selectNone() = 0;
        
        /**
         * @brief 反选点云
         */
        virtual void invertSelection() = 0;
        
        /**
         * @brief 删除选中点
         */
        virtual void deleteSelectedPoints() = 0;

        /**
         * @brief 仅保留当前选中的点
         * @return true 保留成功
         */
        virtual bool keepSelectedOnly() = 0;

        /**
         * @brief 仅保留指定索引点
         * @param indices 要保留的内部点索引
         * @return true 保留成功
         */
        virtual bool retainIndices(const std::vector<int>& indices) = 0;

        /**
         * @brief 检查点云是否为空
         * @return true 点云为空
         * @return false 点云不为空
         */
        virtual bool empty() = 0;
        
        /**
         * @brief 获取点云名称
         * @return QString 点云名称
         */
        virtual QString getName() const = 0;

        /**
         * @brief 设置点云名称
         * @param name 点云名称
         */
        virtual void setName(const QString& name) = 0;

        /**
         * @brief 设置点云可见性
         * @param visible 可见性
         */
        virtual void setVisible(bool visible) = 0;
       
        /**
         * @brief 检查点云是否可见
         * @return true 点云可见
         * @return false 点云不可见
         */
        virtual bool isVisible() const = 0;

        /**
         * @brief 获取点云唯一标识符,id是随机生成的唯一标识符
         * @return std::string 点云唯一标识符
         */
        virtual std::string getId() const = 0;

        /**
         * @brief 按屏幕多边形选择点云
         * @param polygon 屏幕中选中的多边形顶点列表
         * @param viewer PCLVisualizer指针
         * @param is_add_selection 是否添加到当前选择，默认为true
         */
        virtual void selectByScreenPolygon(const std::vector<QPoint>& polygon, 
                                           pcl::visualization::PCLVisualizer::Ptr viewer,
                                           bool is_add_selection = true) = 0;

        /**
         * @brief 渲染点云
         * @param viewer PCLVisualizer指针
         */
        virtual void render(pcl::visualization::PCLVisualizer::Ptr viewer) = 0;
        
        /**
         * @brief 获取点云中点的数量
         * @return size_t 点云中点的数量
         */
        virtual size_t getPointCount() const = 0;
        
        /**
         * @brief 获取当前选中的点的数量
         * @return size_t 当前选中的点的数量
         */
        virtual size_t getSelectedCount() const = 0;
        
        /**
         * @brief 获取点云质心坐标
         * @param x 质心x坐标
         * @param y 质心y坐标
         * @param z 质心z坐标
         */
        virtual void getCentroid(float & x, float & y, float & z) const = 0;
        
        /**
         * @brief 获取点云颜色
         * @return ColorRGB 点云颜色
         */
        virtual ColorRGB getColor() const = 0;
        
        /**
         * @brief 设置点云颜色
         * @param color 点云颜色
         */
        virtual void setColor(const ColorRGB& color) = 0;

        /**
         * @brief 设置点云是否处于高亮选中态
         * @param highlighted 是否高亮
         */
        virtual void setHighlighted(bool highlighted) = 0;

        /**
         * @brief 获取点云是否处于高亮选中态
         * @return true 当前高亮
         * @return false 当前非高亮
         */
        virtual bool isHighlighted() const = 0;
        
        /**
         * @brief 合并另一个点云到当前点云
         * @param others 被合并点云的接口指针列表
         * @return std::vector<std::shared_ptr<PointCloud>> 合并成功返回被合并的点云接口指针列表，否则返回空列表
         */
        virtual std::vector<std::shared_ptr<PointCloud>> merge(const std::vector<std::shared_ptr<PointCloud>>& others) = 0;

        /**
         * @brief 合并另一个点云到当前点云
         * @param other 被合并点云的接口指针
         * @return std::shared_ptr<PointCloud> 合并成功返回被合并的点云接口指针，否则返回空指针
         */
        virtual std::shared_ptr<PointCloud> merge(const std::shared_ptr<PointCloud>& other) = 0;

        /**
         * @brief 将保存点云到文件
         * @param file_path 文件路径
         */
        virtual void saveToFile(const QString& file_path) = 0;

        /**
         * @brief 体素下采样滤波
         * @param leaf_size 体素大小 (单位: mm, PCL通常假定单位一致，这里假设输入也是mm)
         */
        virtual void voxelFilter(float leaf_size) = 0;

        /**
         * @brief 统计滤波去噪
         * @param mean_K 均值估计的邻居点数
         * @param stddev_mul_thresh 标准差倍数阈值 (超过该阈值被视为离群点)
         */
        virtual void sorFilter(int mean_K, double stddev_mul_thresh) = 0;

        /**
         * @brief 直通滤波（范围裁剪）
         * @param axis 过滤字段 (如 "z", "x", "y")
         * @param min 最小值
         * @param max 最大值
         */
        virtual void passThroughFilter(const std::string& axis, float min, float max) = 0;

    protected:
        bool isPointInPolygon(float x, float y, const std::vector<QPoint>& polygon){
            bool c = false;
            int nvert = polygon.size();
            for (int i = 0, j = nvert - 1; i < nvert; j = i++) {
                if (((polygon[i].y() > y) != (polygon[j].y() > y)) &&
                    (x < (polygon[j].x() - polygon[i].x()) * (y - polygon[i].y()) / 
                    (float)(polygon[j].y() - polygon[i].y()) + polygon[i].x())) {
                    c = !c;
                }
            }
            return c;
        }
    };

    /**
     * @brief 具体的点云类
     * 
     * 该类实现了点云的基本操作，包括选择、删除、渲染等。
     * @tparam T 点云类型，必须是pcl::PointXYZ或pcl::PointXYZRGB等类型
     */
    template<typename T>
    class PointCloudConcrete : public PointCloud {
        using CloudPtr = typename pcl::PointCloud<T>::Ptr;
        CloudPtr cloud_;                            ///< 点云指针
        std::vector<int> all_indices_;              ///< 所有点云的索引
        std::vector<int> selected_indices_;         ///< 当前选中的索引
        QString name_;                              ///< 点云名称
        std::string id_;                            ///< 点云ID
        bool need_update_ = true;                   ///< 渲染脏标记，如果脏了才需要重新渲染，否则不浪费时间去渲染
        bool visible_ = true;                       ///< 是否可见
        bool need_rapaint_origin_ = false;          ///< 是否重绘基础点云
        ColorRGB color_ = {0, 0, 0, 255};           ///< 默认点云颜色
        bool highlighted_ = false;                  ///< 是否在界面中高亮显示
    public:
        PointCloudConcrete() = delete;
        
        /**
         * @brief 构造函数
         * 
         * @param cloud 点云指针
         * @param name 点云名称，一般为点云文件名
         */
        PointCloudConcrete(CloudPtr cloud, QString name): cloud_(cloud), name_(name) {
            id_ = "cloud_" + QUuid::createUuid().toString().toStdString();
            all_indices_.resize(cloud_->points.size());
            std::iota(all_indices_.begin(), all_indices_.end(), 0);
        }

        ~PointCloudConcrete() = default;

        std::string getId() const override { return id_;}

        QString getName() const override { return name_;}

        void setName(const QString& name) override { name_ = name; }

        void setVisible(bool visible) override { visible_ = visible; }

        bool isVisible() const override { return visible_; }

        void getPoints(std::vector<Eigen::Vector3f>& points) override {
            points.clear();
            points.reserve(all_indices_.size());
            for (const int idx : all_indices_) {
                if (idx < 0 || idx >= static_cast<int>(cloud_->points.size())) {
                    continue;
                }
                const auto& pt = cloud_->points[idx];
                points.emplace_back(pt.x, pt.y, pt.z);
            }
        }

        void getSelectedPoints(std::vector<Eigen::Vector3f>& points) override {
            points.clear();
            points.reserve(selected_indices_.size());
            for (const int idx : selected_indices_) {
                if (idx < 0 || idx >= static_cast<int>(cloud_->points.size())) {
                    continue;
                }
                const auto& pt = cloud_->points[idx];
                points.emplace_back(pt.x, pt.y, pt.z);
            }
        }

        void selectAll() override {
            selected_indices_ = all_indices_;
            need_update_ = true;
        }

        void selectNone() override {
            selected_indices_.clear();
            need_update_ = true;
        }

        void invertSelection() override {
            std::vector<int> new_selection;

            std::sort(selected_indices_.begin(), selected_indices_.end());
            std::set_difference(all_indices_.begin(), all_indices_.end(),
                                selected_indices_.begin(), selected_indices_.end(),
                                std::back_inserter(new_selection));

            selected_indices_ = new_selection;
            need_update_ = true;
        }

        bool empty() override { return cloud_->points.empty(); }

        void deleteSelectedPoints() override {
            if(selected_indices_.empty()) return;

            //两个索引集合求差
            std::sort(selected_indices_.begin(), selected_indices_.end());
            std::vector<int> new_all_indices;
            std::set_difference(all_indices_.begin(), all_indices_.end(),
                                selected_indices_.begin(), selected_indices_.end(),
                                std::back_inserter(new_all_indices));
            // 更新所有索引
            all_indices_ = std::move(new_all_indices);
            
            qDebug() << ">>> PointCloud" << id_.c_str() << " delete selected points " << selected_indices_.size();
            // 清空选中索引
            selected_indices_.clear();
            
            need_update_ = true;
            need_rapaint_origin_ = true;
        }

        bool keepSelectedOnly() override {
            if (selected_indices_.empty()) {
                return false;
            }
            std::sort(selected_indices_.begin(), selected_indices_.end());
            selected_indices_.erase(std::unique(selected_indices_.begin(), selected_indices_.end()), selected_indices_.end());
            all_indices_ = selected_indices_;
            selected_indices_.clear();
            need_update_ = true;
            need_rapaint_origin_ = true;
            return !all_indices_.empty();
        }

        bool retainIndices(const std::vector<int>& indices) override {
            std::vector<int> validIndices;
            validIndices.reserve(indices.size());
            for (const int idx : indices) {
                if (idx >= 0 && idx < static_cast<int>(cloud_->points.size())) {
                    validIndices.push_back(idx);
                }
            }
            std::sort(validIndices.begin(), validIndices.end());
            validIndices.erase(std::unique(validIndices.begin(), validIndices.end()), validIndices.end());
            if (validIndices.empty()) {
                return false;
            }
            all_indices_ = std::move(validIndices);
            selected_indices_.clear();
            need_update_ = true;
            need_rapaint_origin_ = true;
            return true;
        }

        virtual void getCentroid(float & x, float & y, float & z) const override {
            if (all_indices_.empty()) {
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
                return;
            }
            Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
            pcl::PointIndices indices;
            indices.indices = all_indices_;
            pcl::compute3DCentroid(*cloud_, indices, centroid);
            x = centroid[0];
            y = centroid[1];
            z = centroid[2];
        }

        void selectByScreenPolygon(const std::vector<QPoint>& polygon, 
                                   pcl::visualization::PCLVisualizer::Ptr viewer,
                                   bool is_add_selection = true) override {
            if(!viewer) return;
            if(polygon.size() < 3) return;

            // 获取 4x4 视图投影矩阵 (Projection * View)
            // CompositeProjectionTransformMatrix 包含了从世界坐标到裁剪坐标的所有变换
            vtkRenderer* renderer = viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer();
            vtkCamera* camera = renderer->GetActiveCamera();
            vtkMatrix4x4* vtk_mat = camera->GetCompositeProjectionTransformMatrix(renderer->GetTiledAspectRatio(), 0, 1);
            Eigen::Matrix4f mvp_matrix;
            for(int r=0; r<4; ++r)
                for(int c=0; c<4; ++c)
                    mvp_matrix(r, c) = static_cast<float>(vtk_mat->GetElement(r, c));

            // 获取屏幕尺寸 (用于从 NDC 坐标转换到像素坐标)
            int* size = viewer->getRenderWindow()->GetSize();
            int screen_width = size[0];
            int screen_height = size[1];

            // 临时存储选中的索引
            std::vector<int> new_selection;
            if(is_add_selection)
                new_selection = selected_indices_;  //如果是追加模式，保留原来的

            for(int idx : all_indices_){
                T p = cloud_->points[idx];
                float x = p.x, y = p.y, z = p.z;
                Eigen::Vector4f p_homogeneous(x, y, z, 1.0f);
                Eigen::Vector4f p_ndc = mvp_matrix * p_homogeneous;
                p_ndc /= p_ndc.w();
                int screen_x = (p_ndc.x() + 1.0) * screen_width / 2.0;
                int screen_y = (1.0 - p_ndc.y()) * screen_height / 2.0;
                if(isPointInPolygon(screen_x, screen_y, polygon)){
                    new_selection.push_back(idx);
                }
            }

            if(is_add_selection){
                std::sort(new_selection.begin(), new_selection.end());
                auto last = std::unique(new_selection.begin(), new_selection.end());
                new_selection.erase(last, new_selection.end());
            }

            selected_indices_ = new_selection;
            need_update_ = true;
        }

        void render(pcl::visualization::PCLVisualizer::Ptr viewer) override {
            std::string main_id = id_;
            std::string sel_id = id_ + "_selected";
            if(!viewer) return;
            if(!visible_) {
                viewer->removePointCloud(main_id);
                viewer->removePointCloud(sel_id);
                return;
            }

            //如果此前发生了点云删除，这里需要重新渲染删除后的原始点云，直接从viewer中删去原有点云即可
            if(need_rapaint_origin_){
                viewer->removePointCloud(main_id);
                need_rapaint_origin_ = false;
            }

            if(!viewer->contains(main_id)){
                //在原点云中提取所有的all_indices_
                pcl::PointCloud<T>::Ptr all_cloud = pcl::PointCloud<T>::Ptr(new pcl::PointCloud<T>);
                pcl::ExtractIndices<T> extract;
                extract.setInputCloud(cloud_);
                pcl::PointIndices::Ptr all_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);

                all_indices->indices = all_indices_;
                extract.setIndices(all_indices);
                extract.filter(*all_cloud);

                if(pcl::traits::has_color<T>::value){
                    pcl::visualization::PointCloudColorHandlerRGBField<T> color_handler(all_cloud);
                    viewer->addPointCloud(all_cloud, color_handler, main_id);
                } else{
                    pcl::visualization::PointCloudColorHandlerCustom<T> color_handler(all_cloud, \
                        color_.components.r, color_.components.g, color_.components.b);
                    viewer->addPointCloud(all_cloud, color_handler, main_id);
                }
                qDebug() << "add point cloud";
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, main_id);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                    highlighted_ ? 4 : 1,
                    main_id);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, \
                    color_.components.a / 255.0, main_id);
            } else {
                pcl::PointCloud<T>::Ptr all_cloud = pcl::PointCloud<T>::Ptr(new pcl::PointCloud<T>);
                pcl::ExtractIndices<T> extract;
                extract.setInputCloud(cloud_);
                pcl::PointIndices::Ptr all_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
                all_indices->indices = all_indices_;
                extract.setIndices(all_indices);
                extract.filter(*all_cloud);
                viewer->updatePointCloud(all_cloud, main_id);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, main_id);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                    highlighted_ ? 4 : 1,
                    main_id);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                    color_.components.a / 255.0, main_id);
            }

            if(need_update_){
                // 渲染选中点云
                viewer->removePointCloud(sel_id);

                if(!selected_indices_.empty()){
                    //Step1: 利用pcl::ExtractIndices 提取选中点云
                    pcl::PointCloud<T>::Ptr selected_cloud = pcl::PointCloud<T>::Ptr(new pcl::PointCloud<T>);
                    pcl::ExtractIndices<T> extract;
                    extract.setInputCloud(cloud_);
                    
                    pcl::PointIndices::Ptr selected_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
                    selected_indices->indices = selected_indices_;
                    extract.setIndices(selected_indices);
                    extract.filter(*selected_cloud);
                    
                    //Step2: 渲染选中点云
                    pcl::visualization::PointCloudColorHandlerCustom<T> red_handler(selected_cloud, 255, 0, 0);
                    viewer->addPointCloud(selected_cloud, red_handler, sel_id);
                    qDebug() << "add selected point cloud";
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, sel_id);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, sel_id);
                }

                need_update_ = false;
            }
        }

        size_t getPointCount() const override {return all_indices_.size();}
        size_t getSelectedCount() const override {return selected_indices_.size();}

        ColorRGB getColor() const override {return color_;}
        void setColor(const ColorRGB& color) override {
            color_ = color;
            need_rapaint_origin_ = true;
        }

        void setHighlighted(bool highlighted) override {
            if (highlighted_ == highlighted) {
                return;
            }
            highlighted_ = highlighted;
            need_rapaint_origin_ = true;
        }

        bool isHighlighted() const override { return highlighted_; }

        std::vector<std::shared_ptr<PointCloud>> merge(const std::vector<std::shared_ptr<PointCloud>>& others) override {
            std::vector<std::shared_ptr<PointCloud>> merged_clouds;
            size_t new_size = all_indices_.size();
            for(auto& other : others){
                if(other->getId() == id_) continue; // 不能合并自己

                //向下类型转换检查
                auto other_concrete = std::dynamic_pointer_cast<PointCloudConcrete<T>>(other);
                if(other_concrete){
                    merged_clouds.push_back(other_concrete);
                    new_size += other_concrete->cloud_->points.size();
                    *this->cloud_ += *other_concrete->cloud_;
                } else {
                    //类型不匹配
                    qDebug() << ">>> Merge failed: type mismatch";
                }
            }

            this->all_indices_.resize(new_size);
            std::iota(this->all_indices_.begin(), this->all_indices_.end(), 0);
            assert(new_size == this->cloud_->points.size());

            this->need_update_ = true;
            this->need_rapaint_origin_ = true;
            return merged_clouds;
        }

        std::shared_ptr<PointCloud> merge(const std::shared_ptr<PointCloud>& other) override{
            if(other->getId() == id_) return nullptr; // 不能合并自己
            //向下类型转换检查
            auto other_concrete = std::dynamic_pointer_cast<PointCloudConcrete<T>>(other);
            if(other_concrete){
                *this->cloud_ += *other_concrete->cloud_;
                this->all_indices_.resize(this->cloud_->points.size());
                std::iota(this->all_indices_.begin(), this->all_indices_.end(), 0);
                assert(this->cloud_->points.size() == this->all_indices_.size());

                this->need_update_ = true;
                this->need_rapaint_origin_ = true;
                return other_concrete;
            } else {
                //类型不匹配
                qDebug() << ">>> Merge failed: type mismatch";
                return nullptr;
            }
        }

        void saveToFile(const QString& file_path) override {
            CloudPtr cloud_copy(new pcl::PointCloud<T>);
            pcl::ExtractIndices<T> extract;
            extract.setInputCloud(cloud_);
            pcl::PointIndices::Ptr all_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
            all_indices->indices = all_indices_;
            extract.setIndices(all_indices);
            extract.filter(*cloud_copy);
            // 确保 cloud_copy 不为空
            if (!cloud_copy || cloud_copy->points.empty()) {
                qDebug() << ">>> Save failed: empty cloud";
                return;
            }
            QFileInfo fileInfo(file_path);
            QString ext = fileInfo.suffix().toLower();

            if(ext == "ply"){
                pcl::io::savePLYFile(file_path.toStdString(), *cloud_copy);
            } else if(ext == "pcd"){
                pcl::io::savePCDFile(file_path.toStdString(), *cloud_copy);
            } else if(ext == "las"){
                // TODO:las类型需要特殊处理，稍后实现
                qDebug() << ">>> Save failed: text file format not supported";
            } else if(ext == "xyz" || ext == "txt" || ext == "asc"){
                // 写入 xyz 格式
                std::ofstream out_file(file_path.toStdString());
                if (!out_file.is_open()) {
                    qDebug() << ">>> Save failed: cannot open file for writing";
                    return;
                }

                for (const auto& point : cloud_copy->points) {
                    out_file << point.x << " " << point.y << " " << point.z << "\n";
                }
                out_file.close();
            } else {
                qDebug() << ">>> Save failed: unknown file format";
            }
        }

        void voxelFilter(float leaf_size) override {
            if(leaf_size <= 0.0){
                qDebug() << ">>> Voxel filter failed: leaf size must be positive";
                return;
            }
            selectNone();
            auto temp = cloud_;
            pcl::VoxelGrid<T> voxel_grid;
            voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel_grid.setInputCloud(temp);
            voxel_grid.filter(*temp);
            
            size_t new_size = temp->points.size();
            this->all_indices_.resize(new_size);
            std::iota(this->all_indices_.begin(), this->all_indices_.end(), 0);

            this->need_update_ = true;
            this->need_rapaint_origin_ = true;
        }

        void sorFilter(int mean_K, double stddev_mul_thresh) override {
            if(mean_K <= 0 || stddev_mul_thresh <= 0.0){
                qDebug() << ">>> SOR filter failed: mean_K and stddev_mul_thresh must be positive";
                return;
            }
            selectNone();
            auto temp = cloud_;

            pcl::StatisticalOutlierRemoval<T> sor;
            sor.setMeanK(mean_K);
            sor.setStddevMulThresh(stddev_mul_thresh);
            sor.setInputCloud(temp);
            sor.filter(*temp);
            
            size_t new_size = cloud_->points.size();
            this->all_indices_.resize(new_size);
            std::iota(this->all_indices_.begin(), this->all_indices_.end(), 0);
            
            this->need_update_ = true;
            this->need_rapaint_origin_ = true;
        }
        void passThroughFilter(const std::string& axis, float min, float max) override {
            if(min >= max){
                qDebug() << ">>> PassThrough filter failed: min must be less than max";
                return;
            }
            selectNone();
            auto temp = cloud_;

            pcl::PassThrough<T> pass;
            pass.setFilterFieldName(axis);  //设置过滤轴
            pass.setFilterLimits(min, max);  //设置过滤范围
            pass.setInputCloud(temp);
            pass.filter(*temp);
            
            size_t new_size = cloud_->points.size();
            this->all_indices_.resize(new_size);
            std::iota(this->all_indices_.begin(), this->all_indices_.end(), 0);

            this->need_update_ = true;
            this->need_rapaint_origin_ = true;
        }

    };

    // 这里我们需要决定默认加载的点类型。
    // 根据需求（txt/asc 读取 xyz），最通用的是 pcl::PointXYZ。
    // 如果需要支持颜色，逻辑会稍微复杂一点（需要预读判断字段）。
    using DefaultPointT = pcl::PointXYZ;

    /**
     * @brief 内部辅助函数：读取文本类型的点云 (txt, asc)
     * @param file_path 文件绝对路径
     * @return 成功返回 PCL 点云指针，失败返回 nullptr
     */
    template <typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr createFromTextFile(const QString& file_path) {
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

        QFile file(file_path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qDebug() << "Cannot open file:" << file_path;
            return nullptr;
        }

        QTextStream in(&file);
        
        // 预先保留空间以提高性能（假设至少有1万点，如果文件很大，vector会自动扩容）
        cloud->points.reserve(10000);

        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();
            
            // 跳过空行或注释（假设以 # 开头为注释）
            if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) {
                continue;
            }

            // 使用 QTextStream 或 split 解析数据
            // 假设格式为：x y z [其他数据]
            QTextStream lineStream(&line);
            float x, y, z;
            
            // 尝试读取前三个浮点数
            lineStream >> x >> y >> z;

            if (lineStream.status() == QTextStream::Ok) {
                PointT p;
                p.x = x;
                p.y = y;
                p.z = z;
                cloud->points.push_back(p);
            }
        }

        file.close();
        
        // 设置基本属性
        cloud->width = cloud->points.size();
        cloud->height = 1; // 无序点云
        cloud->is_dense = true;

        return cloud;        
    }

    /**
     * @brief 通用加载函数，用于加载不同格式的点云文件
     * @param file_path 文件绝对路径
     * @param point_scale 点云缩放比例
     * @param sampling_rate 采样率
     * @return 成功返回点云指针，失败返回 nullptr
     */
    static std::shared_ptr<PointCloud> createFromFile(const QString& file_path){
        QFileInfo fileInfo(file_path);
        QString ext = fileInfo.suffix().toLower();
        QString name = fileInfo.baseName();

        // 创建空的点云对象
        pcl::PointCloud<DefaultPointT>::Ptr cloud(new pcl::PointCloud<DefaultPointT>);
        int result = -1;

        if (ext == "pcd") {
            result = pcl::io::loadPCDFile(file_path.toStdString(), *cloud);
        } 
        else if (ext == "ply") {
            result = pcl::io::loadPLYFile(file_path.toStdString(), *cloud);
        } 
        else if (ext == "stl") {
            pcl::PolygonMesh mesh;
            result = pcl::io::loadPolygonFileSTL(file_path.toStdString(), mesh);
            if (result > 0) {
                pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
                result = cloud->empty() ? -1 : 0;
            }
        }
        else if (ext == "txt" || ext == "asc") {
            cloud = createFromTextFile<DefaultPointT>(file_path);
            if (cloud && !cloud->empty()) {
                result = 0; // 成功
            }
        } else {
            qDebug() << "Unsupported file extension:" << ext;
            return nullptr;
        }

        // 检查是否加载成功
        if (result < 0 || cloud->empty()) {
            qDebug() << "Failed to load file or file is empty:" << file_path;
            return nullptr;
        }

        double point_scale, sampling_rate;
        if(!ImportOptionDialog::getImportOptions(point_scale, sampling_rate)){
            qDebug() << "Import options dialog canceled.";
            return nullptr;
        }

        //根据选中的单位执行缩放
        if(std::abs(point_scale - 1.0) > EPSILON){
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.scale(point_scale);
            pcl::transformPointCloud(*cloud, *cloud, transform);
        }

        //根据选中的采样率执行随机采样
        if(sampling_rate < 1.0 && sampling_rate > 0.0){
            pcl::RandomSample<DefaultPointT> sampler;
            sampler.setInputCloud(cloud);

            // 计算目标采样点数量
            unsigned int target_size = static_cast<unsigned int>(cloud->size() * sampling_rate);
            sampler.setSample(target_size);
            sampler.filter(*cloud);
        }

        qDebug() << "Successfully loaded" << cloud->size() << "points from" << file_path;

        // --- 核心：实例化模板类并返回接口指针 ---
        return std::make_shared<PointCloudConcrete<DefaultPointT>>(cloud, name);        
    }

    using PointCloudPtr = std::shared_ptr<PointCloud>;
}}
#endif
