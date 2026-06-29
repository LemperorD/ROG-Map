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

#include <rog_map/rog_map_core/sliding_map.h>

// #define COUNTER_MAP_DEBUG  // 取消注释以启用计数器地图调试检查
namespace rog_map {

    /// 计数器地图（抽象基类）
    /// 实现多分辨率计数器的核心逻辑：
    /// 一个CounterMap栅格包含多个基础地图的栅格（子栅格）
    /// 通过计数占用和未知子栅格的数量来判断该栅格的状态
    ///
    /// 判断规则：
    /// 1) 任何子栅格被占用 → 此栅格为 OCCUPIED
    /// 2) 如果没有子栅格被占用：
    ///    2.1) 未知子栅格数 ≥ unk_thresh → 此栅格为 UNKNOWN
    ///    2.2) 否则 → 此栅格为 KNOWN_FREE
    class CounterMap : public SlidingMap {
    public:
        typedef std::shared_ptr<CounterMap> Ptr;

        CounterMap() = default;

        ~CounterMap() = default;

        /// 更新栅格计数器
        /// 当基础概率地图的子栅格状态改变时调用此函数
        /// @param pos 子栅格的世界坐标
        /// @param from_type 子栅格变化前的类型
        /// @param to_type 子栅格变化后的类型
        void updateGridCounter(const Vec3f &pos,
                               const GridType &from_type,
                               const GridType &to_type);


    protected:
        bool map_empty_{true};

        /// 计数器地图的内部数据
        struct MapData {
            std::vector<int16_t> occupied_cnt;   ///< 每个CounterMap栅格的占用子栅格计数
            std::vector<int16_t> unknown_cnt;    ///< 每个CounterMap栅格的未知子栅格计数
            int sub_grid_num;                     ///< 每个CounterMap栅格包含的子栅格总数
            int unk_thresh;                       ///< 判定为UNKNOWN的未知子栅格阈值
        } md_;

        /// 初始化计数器地图
        /// @param half_prob_map_size_i 基础概率地图的半尺寸（确保总尺寸为奇数）
        /// @param prob_map_resolution 基础概率地图的分辨率
        /// @param temp_counter_map_resolution 期望的计数器地图分辨率（会调整为概率分辨率的整数倍）
        /// @param inflation_step 膨胀步长（用于预留边界）
        /// @param map_sliding_en 是否启用地图滑动
        /// @param sliding_thresh 滑动触发阈值
        /// @param fix_map_origin 固定地图原点
        /// @param unk_thresh 未知判定阈值（0~1之间，表示未知子栅格占比）
        void initCounterMap(
                const Vec3i &half_prob_map_size_i,
                const double &prob_map_resolution,
                const double &temp_counter_map_resolution,
                const int &inflation_step,
                const bool &map_sliding_en,
                const double &sliding_thresh,
                const Vec3f &fix_map_origin,
                const double &unk_thresh);

        /// 获取哈希索引对应的栅格类型
        /// 实现论文中的三级判断逻辑
        /// @param hash_id 栅格的一维哈希索引
        /// @return 栅格类型（OCCUPIED/UNKNOWN/KNOWN_FREE）
        GridType getGridType(const int &hash_id) const {
            if (isOccupied(hash_id)) {
                return GridType::OCCUPIED;  // 规则1: 任何子栅格被占用
            } else {
                if (isUnknown(hash_id)) {
                    return GridType::UNKNOWN;  // 规则2.1: 足够多的未知子栅格
                } else {
                    return GridType::KNOWN_FREE;  // 规则2.2: 已知空闲
                }
            }
        }

        /// 跳跃边触发（纯虚函数）
        /// 当CounterMap栅格的状态发生变化时调用
        /// 子类（InfMap等）实现具体的膨胀/收缩逻辑
        /// @param id_g 状态发生变化栅格的全局索引
        /// @param from_type 变化前的栅格类型
        /// @param to_type 变化后的栅格类型
        virtual void triggerJumpingEdge(const Vec3i &id_g,
                                        const GridType &from_type,
                                        const GridType &to_type) = 0;

        /// 重置单个栅格（纯虚函数）
        /// 在地图滑动时调用，子类实现具体的清理逻辑
        virtual void resetOneCell(const int &hash_id) = 0;

        // ===== 查询函数（基于全局索引） =====
        bool isUnknown(const Vec3i &id_g) const;
        bool isOccupied(const Vec3i &id_g) const;
        bool isKnownFree(const Vec3i &id_g) const;

        // ===== 查询函数（基于哈希索引） =====
        bool isUnknown(const int &hash_id) const {
            return md_.unknown_cnt[hash_id] >= md_.unk_thresh;
        }

        bool isOccupied(const int &hash_id) const {
            return md_.occupied_cnt[hash_id] > 0;
        }

        bool isKnownFree(const int &hash_id) const {
            return (md_.unknown_cnt[hash_id] < md_.unk_thresh) &&  // 条件1: 不够未知
                   (md_.occupied_cnt[hash_id] == 0);                // 条件2: 没有占用
        }

    private:
        /// 重置单个栅格（重写基类纯虚函数）
        /// 将占用计数归零，未知计数设为最大值（即完全未知）
        /// 同时调用子类的 resetOneCell 处理额外清理
        void resetCell(const int &hash_id) override {
            md_.occupied_cnt[hash_id] = 0;
            md_.unknown_cnt[hash_id] = md_.sub_grid_num;
            resetOneCell(hash_id);
        }

        bool had_been_initialized{false};  ///< 防重复初始化标志
    };
}
