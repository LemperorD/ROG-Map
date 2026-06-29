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

#include <rog_map/rog_map_core/counter_map.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

//#define ESDF_MAP_DEBUG  // 取消注释以启用ESDF调试输出

namespace rog_map {

    /// ESDF（欧几里得符号距离场）地图类
    /// 继承自 CounterMap，实现增量更新的欧几里得符号距离场
    ///
    /// 算法：基于独立扫描抛物线法的6遍 1D DP 算法
    /// - 正向（障碍物外部距离）：计算最近障碍物的距离
    /// - 负向（障碍物内部距离）：计算从障碍物内部到边界的距离
    /// - 符号距离 = 正向距离 - 负向距离
    ///
    /// 支持三线性插值获取任意位置的连续距离值和梯度
    class ESDFMap : public CounterMap {

    public:
        typedef std::shared_ptr<ESDFMap> Ptr;
        ESDFMap() = default;
        ~ESDFMap() = default;

        /// 初始化ESDF地图
        /// @param half_prob_map_size_i 基础概率地图半尺寸（确保奇数）
        /// @param prob_map_resolution 基础概率地图分辨率
        /// @param temp_counter_map_resolution ESDF地图期望分辨率
        /// @param local_update_box ESDF局部更新范围
        /// @param map_sliding_en 是否启用地图滑动
        /// @param sliding_thresh 滑动触发阈值
        /// @param fix_map_origin 固定地图原点
        /// @param unk_thresh 未知判定阈值
        void initESDFMap(
                const Vec3i &half_prob_map_size_i,
                const double &prob_map_resolution,
                const double &temp_counter_map_resolution,
                const Vec3f &local_update_box,
                const bool &map_sliding_en,
                const double &sliding_thresh,
                const Vec3f &fix_map_origin,
                const double &unk_thresh);

        /// 获取最近更新的ESDF包围盒
        void getUpdatedBbox(Vec3f & box_min, Vec3f & box_max) const;

        void resetLocalMap() override;

        /// 获取指定位置的距离值（世界坐标）
        double getDistance(const Vec3f &pos) const;

        /// 获取指定位置的距离值（全局索引）
        double getDistance(const Vec3i &id_g) const;

        /// 增量更新3D ESDF（核心函数）
        /// 使用6遍扫掠算法在局部更新范围内重算距离场
        /// @param cur_odom 机器人当前位置
        void updateESDF3D(const Vec3f &cur_odom);

        void resetOneCell(const int & hash_id) override;

        // ===== ESDF 环境：三线性插值查询 =====

        /// 查询任意位置的距离值（三线性插值）
        void evaluateEDT(const Eigen::Vector3d& pos, double& dist);

        /// 查询任意位置的距离梯度（一阶导数）
        void evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);

        /// 查询任意位置的二阶梯度和（三线性插值）
        void evaluateSecondGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);

        /// 可视化ESDF梯度场
        void visEDTGrad(const Vec3f &box_min_d,
                        const Vec3f &box_max_d,
                        const double &visualize_z,
                        visualization_msgs::MarkerArray &mk_arr);

        // ===== 可视化用函数 =====

        /// 获取ESDF占用栅格的点云（用于RViz可视化）
        void getESDFOccPC2(const Vec3f &box_min_d,
                           const Vec3f &box_max_d,
                           sensor_msgs::PointCloud2 &pc2);

        /// 获取正距离值的ESDF点云（障碍物外部，距离 > 0）
        void getPositiveESDFPC2(const Vec3f &box_min_d,
                                const Vec3f &box_max_d,
                                const double &visualize_z,
                                sensor_msgs::PointCloud2 &pc2);

        /// 获取负距离值的ESDF点云（障碍物内部，距离 < 0）
        void getNegativeESDFPC2(const Vec3f &box_min_d,
                                const Vec3f &box_max_d,
                                const double &visualize_z,
                                sensor_msgs::PointCloud2 &pc2);

    private:

        /// 通用的 1D 距离变换函数（ESDF核心算法）
        /// 基于 Felzenszwalb & Huttenlocher 的抛物线下界算法
        ///
        /// @tparam F_get_val 获取原始距离值的函数：int z -> double
        /// @tparam F_set_val 设置计算结果值的函数：int z, double val -> void
        /// @param start 扫描起始索引
        /// @param end 扫描结束索引
        /// @param dim 当前扫描的维度（0=X, 1=Y, 2=Z）
        /// @param id_l 当前维度的局部偏移量
        template<typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val,
                      const int &start, const int &end, const int &dim, const int &id_l);

        bool had_been_initialized{false};         ///< 防重复初始化
        bool map_empty_{true};                    ///< 地图是否为空
        std::vector<double> distance_buffer;      ///< ESDF距离值缓冲区
        vector<double> tmp_buffer1_, tmp_buffer2_;///< 临时缓冲区（用于6遍扫掠的中间结果）
        Vec3i half_local_update_box_i_;           ///< ESDF局部更新盒半尺寸（索引单位）
        Vec3i update_local_map_min_i_, update_local_map_max_i_;  ///< 最近更新的范围
        pcl::PointCloud<pcl::PointXYZI> pcl_pc;   ///< 用于可视化输出的PCL点云
        std::mutex update_esdf_mtx;               ///< ESDF更新的互斥锁

        /// ESDF 不使用跳跃边触发（空实现）
        void triggerJumpingEdge(const rog_map::Vec3i &id_g, const rog_map::GridType &from_type,
                                const rog_map::GridType &to_type) override {}

        // ===== 三线性插值辅助函数 =====

        /// 获取目标点周围 2x2x2 立方体的8个顶点坐标
        void getSurroundPts(const Vec3f& pos, Vec3f pts[2][2][2], Vec3f & diff);

        /// 获取8个顶点的距离值
        void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);

        /// 获取8个顶点的一阶梯度
        void getSurroundFirstGrad(Eigen::Vector3d pts[2][2][2], double first_grad[2][2][2][3]);

        /// 三线性插值：距离值
        void interpolateTrilinearEDT(double values[2][2][2], const Eigen::Vector3d& diff, double& value);

        /// 三线性插值：一阶梯度
        void interpolateTrilinearFirstGrad(double values[2][2][2], const Eigen::Vector3d& diff,
                                           Eigen::Vector3d& grad);

        /// 三线性插值：二阶梯度和
        void interpolateTrilinearSecondGrad(double first_grad[2][2][2][3], const Eigen::Vector3d& diff,
                                            Eigen::Vector3d& grad);


    };

}
