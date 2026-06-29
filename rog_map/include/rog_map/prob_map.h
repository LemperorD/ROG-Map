/**
* 本文件是 ROG-Map 的一部分
*
* 版权所有 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* 由 Yunfan REN <renyf at connect dot hku dot hk> 开发
* 更多信息请参见 <https://github.com/hku-mars/ROG-Map>.
* 如果您使用此代码，请引用上述网站上列出的相应出版物。
*
* ROG-Map 是自由软件：您可以按照自由软件基金会发布的
* GNU Lesser General Public License 的条款重新分发和/或修改它，
* 无论是许可证的第3版，还是（按您选择）任何后续版本。
*
* ROG-Map 的分发是希望它能有用，
* 但不提供任何担保；甚至没有对适销性或
* 适用于特定目的的默示担保。有关更多详细信息，请参阅
* GNU General Public License。
*
* 您应该已随 ROG-Map 收到一份 GNU Lesser General Public License 的副本。
* 如果没有，请参见 <http://www.gnu.org/licenses/>.
*/


#pragma once

#include <queue>
#include <rog_map/inf_map.h>
#include <rog_map/free_cnt_map.h>
#include <rog_map/esdf_map.h>
#include <utils/raycaster.h>

namespace rog_map {

    /// 概率地图类
    /// 核心地图实现：基于对数几率(log-odds)的概率占用栅格地图
    /// 使用射线投射进行概率更新，支持批量更新、增量膨胀和前沿提取
    ///
    /// 状态判断阈值（对数几率空间）：
    /// - 已知空闲: prob < l_free
    /// - 占用: prob >= l_occ
    /// - 未知: l_free <= prob < l_occ
    class ProbMap : public SlidingMap {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<ProbMap> Ptr;

        ProbMap() = default;
        ~ProbMap() = default;

        /// 初始化概率地图（只能调用一次）
        void initProbMap();

        // ===== 状态查询（世界坐标） =====
        bool isOccupied(const Vec3f &pos) const;
        bool isUnknown(const Vec3f &pos) const;
        bool isKnownFree(const Vec3f &pos) const;
        bool isOccupiedInflate(const Vec3f &pos) const;
        bool isUnknownInflate(const Vec3f &pos) const;
        bool isKnownFreeInflate(const Vec3f & pos) const;
        bool isFrontier(const Vec3f &pos) const;
        bool isFrontier(const Vec3i &id_g) const;

        /// 获取栅格类型
        GridType getGridType(Vec3i &id_g) const;
        GridType getGridType(const Vec3f &pos) const;
        GridType getInfGridType(const Vec3f &pos) const;

        /// 获取地图值（对数几率）
        double getMapValue(const Vec3f &pos) const;

        /// 盒子搜索：在包围盒内搜索指定类型的所有栅格
        void boxSearch(const Vec3f &_box_min, const Vec3f &_box_max,
                       const GridType &gt, vec_E<Vec3f> &out_points) const;

        /// 膨胀地图盒子搜索
        void boxSearchInflate(const Vec3f &box_min, const Vec3f &box_max,
                              const GridType &gt, vec_E<Vec3f> &out_points) const;

        /// 将包围盒限制在局部地图范围内
        void boundBoxByLocalMap(Vec3f &box_min, Vec3f &box_max) const;

        Vec3f getLocalMapOrigin() const;
        Vec3f getLocalMapSize() const;

        double getResolution() const{
            return sc_.resolution;
        }

        double getInfResolution()const {
            return inf_map_->getResolution();
        }

        /// 直接从PCD点云更新地图（无射线投射，仅将点数视为占用命中）
        void updateOccPointCloud(const PointCloud &input_cloud);

        /// 将耗时数据输出到日志文件
        void writeTimeConsumingToLog(std::ofstream &log_file);

        /// 将地图信息输出到日志文件
        void writeMapInfoToLog(std::ofstream &log_file);

        /// 更新概率地图的主入口
        /// @param cloud 输入点云
        /// @param pose 机器人当前位姿
        void updateProbMap(const PointCloud &cloud, const Pose &pose);

    protected:
        rog_map::Config cfg_;                     ///< 全局配置
        InfMap::Ptr inf_map_;                     ///< 膨胀地图指针
        FreeCntMap::Ptr fcnt_map_;                ///< 前沿计数地图指针
        ESDFMap::Ptr esdf_map_;                   ///< ESDF地图指针
        std::vector<float> occupancy_buffer_;     ///< 占用概率缓冲区（对数几率值）

        bool map_empty_{true};                    ///< 地图是否为空

        /// 射线投射数据
        struct RaycastData {
            raycaster::RayCaster raycaster;                           ///< 3D DDA射线投射器
            std::queue<Vec3i> update_cache_id_g;                      ///< 待更新栅格的全局索引队列
            std::vector<uint16_t> operation_cnt;                      ///< 每个栅格的操作计数（命中+未命中总次数）
            std::vector<uint16_t> hit_cnt;                            ///< 每个栅格的命中计数
            Vec3f cache_box_max, cache_box_min;                       ///< 缓存盒子的边界（世界坐标）
            Vec3f local_update_box_max, local_update_box_min;         ///< 局部更新盒子的边界
            int batch_update_counter{0};                              ///< 批量更新计数器
            std::mutex raycast_range_mtx;                             ///< 射线投射范围的互斥锁
        } raycast_data_;

        vector<double> time_consuming_;                               ///< 各步骤耗时记录
        vector<string> time_consuming_name_{"Total", "Raycast", "Update_cache",
                                            "Inflation", "PointCloudNumber",
                                            "CacheNumber", "InflationNumber"};  ///< 耗时统计标签

        // ===== 标准化查询（对数几率阈值判断） =====
        bool isKnownFree(const double &prob) const {
            return prob < cfg_.l_free;
        }
        bool isOccupied(const double &prob) const {
            return prob >= cfg_.l_occ;
        }
        bool isUnknown(const double &prob) const {
            return prob >= cfg_.l_free && prob < cfg_.l_occ;
        }

        /// 滑动所有子地图（概率地图 + 膨胀地图 + 前沿地图 + ESDF地图）
        void slideAllMap(const Vec3f &pos);

        // ===== 基于全局索引的状态查询 =====
        bool isOccupied(const Vec3i &id_g) const;
        bool isUnknown(const Vec3i &id_g) const;
        bool isKnownFree(const Vec3i &id_g) const;

        void resetCell(const int &hash_id) override;

        /// 从缓存中批量应用概率更新
        void probabilisticMapFromCache();

        /// 命中点更新：增加对数几率（趋向占用）
        void hitPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        /// 未命中点更新：减少对数几率（趋向已知空闲）
        void missPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        /// 射线投射处理：对输入点云执行射线投射，将结果缓存
        void raycastProcess(const PointCloud &input_cloud, const Vec3f &cur_odom);

        /// 将栅格插入更新候选队列
        /// @param id_g 栅格全局索引
        /// @param is_hit 是否为命中（true）或射线穿透（false）
        void insertUpdateCandidate(const Vec3i &id_g, bool is_hit);

        /// 更新局部更新盒子的位置（跟随机器人移动）
        void updateLocalBox(const Vec3f &cur_odom);

        void resetLocalMap() override;
    };
}