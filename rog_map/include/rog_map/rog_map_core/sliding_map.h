/**
* 本文件是 ROG-Map 的一部分
* ... (license header, same as above) ...
*/

#pragma once

#include <rog_map/rog_map_core/config.hpp>
#include <utils/scope_timer.hpp>

namespace rog_map {
    /// ORIGIN_AT_CORNER 策略说明：
    /// 对于所有栅格，位置定义在栅格中心，包括地图边界。
    /// [原点] 对于原点，它定义在栅格的角上。
    ///
    ///      bd_min       origin           bd_max
    ///        |              |              |
    ///        v              V              V
    ///        -2        -1        0         1
    ///   |---------|---------|---------|---------|
    ///  -2  -1.5  -1    0.5  0   0.5   1   1.5   2
    ///
    /// 即全局索引 id_g = floor(pos / resolution)
    /// 栅格中心位置 pos = (id_g + 0.5) * resolution

    /// 滑动地图基类
    /// 实现零拷贝的3D循环缓冲区地图
    /// 使用取模索引使得地图可以随机器人移动而"滑动"，无需重新分配内存
    /// 所有地图类型（概率图、膨胀图、ESDF图、前沿计数图）都继承自此类
    class SlidingMap {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// 构造函数：初始化滑动地图
        /// @param half_map_size_i 半地图尺寸（网格数），确保总尺寸为奇数
        /// @param resolution 栅格分辨率（米/栅格）
        /// @param map_sliding_en 是否启用地图滑动
        /// @param sliding_thresh 滑动触发距离阈值（米）
        /// @param fix_map_origin 地图滑动禁用时的固定原点
        SlidingMap(const Vec3i &half_map_size_i,
                   const double &resolution,
                   const bool &map_sliding_en,
                   const double &sliding_thresh,
                   const Vec3f &fix_map_origin);

        SlidingMap() = default;

        /// 初始化滑动地图（可在构造函数外调用，但只能调用一次）
        void initSlidingMap(const Vec3i &half_map_size_i,
                  const double &resolution,
                  const bool &map_sliding_en,
                  const double &sliding_thresh,
                  const Vec3f &fix_map_origin);

        /// 打印地图信息（分辨率、尺寸等）
        void printMapInformation();

        /// 地图滑动：将地图原点移动到里程计位置
        /// 实现零拷贝循环缓冲区滑动，清除超出范围的内存
        /// @param odom 新的地图中心位置（通常是机器人当前位置）
        void mapSliding(const Vec3f &odom);

        /// 检查世界坐标是否在局部地图范围内
        bool insideLocalMap(const Vec3f &pos) const;

        /// 检查全局索引是否在局部地图范围内
        bool insideLocalMap(const Vec3i &id_g) const;

    protected:
        /// 滑动地图的内部配置
        struct SlidingConfig {
            double resolution{0.0};               ///< 栅格分辨率（米）
            double resolution_inv{0.0};           ///< 分辨率的倒数（用于快速除法）
            double sliding_thresh{0.0};           ///< 滑动触发阈值
            bool map_sliding_en{false};           ///< 是否启用滑动
            Vec3f fix_map_origin{};               ///< 固定地图原点
            Vec3i visualization_range_i{};        ///< 可视化范围（索引单位）
            Vec3i map_size_i{};                   ///< 地图总尺寸（栅格数）
            Vec3i half_map_size_i{};              ///< 半地图尺寸（栅格数）
            int virtual_ceil_height_id_g{0};      ///< 虚拟天花板全局索引Z
            int virtual_ground_height_id_g{0};    ///< 虚拟地板全局索引Z
            int map_vox_num{0};                   ///< 地图体素总数（= map_size_i.prod()）
        } sc_;

        // ===== 局部地图的边界信息（世界坐标和索引坐标） =====
        Vec3f local_map_origin_d_;                ///< 局部地图原点（世界坐标，米）
        Vec3f local_map_bound_min_d_;             ///< 局部地图下边界（世界坐标）
        Vec3f local_map_bound_max_d_;             ///< 局部地图上边界（世界坐标）
        Vec3i local_map_origin_i_;                ///< 局部地图原点（全局索引）
        Vec3i local_map_bound_min_i_;             ///< 局部地图下边界（全局索引）
        Vec3i local_map_bound_max_i_;             ///< 局部地图上边界（全局索引）

        /// 重置整个局部地图（纯虚函数，子类实现具体清理逻辑）
        virtual void resetLocalMap() = 0;

        /// 重置单个栅格（纯虚函数，子类实现具体清理逻辑）
        /// 在地图滑动时，超出范围的栅格会被调用此函数清理
        /// @param hash_id 栅格的一维哈希索引
        virtual void resetCell(const int & hash_id) = 0;

        /// 清除地图范围外的内存
        /// 在地图沿某个轴滑动后，清除超出边界的"旧"数据
        /// @param clear_id 需要清除的局部索引列表（沿当前轴向）
        /// @param i 当前滑动的轴向（0=X, 1=Y, 2=Z）
        void clearMemoryOutOfMap(const vector<int> &clear_id, const int &i);

        // ===== 坐标转换函数 =====

        /// 将局部索引（相对于地图原点的偏移）转换为一维哈希索引
        /// @param id_in 局部索引（范围 [-half_map_size_i, half_map_size_i]）
        /// @return 一维哈希索引（范围 [0, map_vox_num-1]）
        int getLocalIndexHash(const Vec3i &id_in) const;

        /// 世界坐标 → 全局索引
        /// ORIGIN_AT_CORNER: id = floor(pos / resolution)
        void posToGlobalIndex(const Vec3f &pos, Vec3i &id) const;

        /// 单轴世界坐标 → 全局索引
        void posToGlobalIndex(const double &pos, int &id) const;

        /// 全局索引 → 世界坐标（栅格中心）
        /// ORIGIN_AT_CORNER: pos = (id + 0.5) * resolution
        void globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const;

        /// 全局索引 → 局部索引（取模变换，论文公式(7)-(8)）
        /// 将无界的全局索引映射到有界的局部缓冲区
        void globalIndexToLocalIndex(const Vec3i &id_g, Vec3i &id_l) const;

        /// 局部索引 → 全局索引（仅用于 clearMemoryOutOfMap）
        void localIndexToGlobalIndex(const Vec3i &id_l, Vec3i &id_g) const;

        /// 局部索引 → 世界坐标
        void localIndexToPos(const Vec3i &id_l, Vec3f &pos) const;

        /// 一维哈希索引 → 局部三维索引
        void hashIdToLocalIndex(const int &hash_id, Vec3i &id) const;

        /// 一维哈希索引 → 世界坐标
        void hashIdToPos(const int &hash_id, Vec3f &pos) const;

        /// 一维哈希索引 → 全局索引
        void hashIdToGlobalIndex(const int &hash_id, Vec3i &id_g) const;

        /// 世界坐标 → 一维哈希索引（快捷方法）
        int getHashIndexFromPos(const Vec3f &pos) const;

        /// 全局索引 → 一维哈希索引（快捷方法）
        int getHashIndexFromGlobalIndex(const Vec3i &id_g) const;

        /// 更新局部地图原点和边界
        /// @param new_origin_d 新原点世界坐标
        /// @param new_origin_i 新原点全局索引
        void updateLocalMapOriginAndBound(const Vec3f &new_origin_d,
                                          const Vec3i &new_origin_i);


    private:
        bool had_been_initialized{false};  ///< 防止重复初始化标志

    };


}