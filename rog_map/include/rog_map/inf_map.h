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

namespace rog_map {

    /// 膨胀地图类
    /// 继承自 CounterMap，实现球面膨胀逻辑
    /// 维护占用膨胀计数器和未知膨胀计数器（可选），
    /// 提供膨胀后的占用/未知/已知空闲查询
    class InfMap : public CounterMap {
    public:
        typedef std::shared_ptr<InfMap> Ptr;

        InfMap(rog_map::Config &cfg);
        ~InfMap() = default;

        double getResolution()const {
            return sc_.resolution;
        }

        /// 查询：该位置是否被膨胀占用
        bool isOccupiedInflate(const Vec3f &pos) const;

        /// 查询：该位置是否被膨胀未知
        bool isUnknownInflate(const Vec3f &pos) const;

        /// 查询：该位置是否为膨胀后的已知空闲
        bool isKnownFreeInflate(const Vec3f &pos) const;

        /// 获取膨胀操作的统计信息（数量和耗时）
        void getInflationNumAndTime(double &inf_n, double &inf_t);

        /// 输出膨胀地图信息到日志
        void writeMapInfoToLog(std::ofstream &log_file);

        /// 在膨胀地图中进行盒子搜索
        void boxSearch(const Vec3f &box_min, const Vec3f &box_max,
                       const GridType &gt, vec_E<Vec3f> &out_points) const;

        void resetLocalMap() override;

        GridType getGridType(const Vec3f &pos) const;
        GridType getGridType(const Vec3i &id_g) const ;

    private:
        /// 膨胀地图的内部数据
        struct InfMapData {
            std::vector<int16_t> occ_inflate_cnt;    ///< 占用膨胀计数器（每个栅格有多少邻居是占用的）
            std::vector<int16_t> unk_inflate_cnt;    ///< 未知膨胀计数器（每个栅格有多少邻居是未知的）
            int unk_neighbor_num;                     ///< 未知膨胀邻域大小
            int occ_neighbor_num;                     ///< 占用膨胀邻域大小
        } imd_;

        rog_map::Config cfg_;
        int inf_num_{0};          ///< 膨胀操作计数（用于性能统计）
        double inf_t_{0.0};       ///< 膨胀操作耗时累计（秒）

        /// 跳跃边触发：当CounterMap栅格状态改变时，
        /// 更新该栅格及其球面邻域栅格的膨胀计数器
        void triggerJumpingEdge(const rog_map::Vec3i &id_g,
                                const rog_map::GridType &from_type,
                                const rog_map::GridType &to_type) override;

        /// 基于全局索引查询：是否为膨胀占用
        bool isOccupiedInflate(const Vec3i &id_g) const;

        /// 更新占用膨胀：在球面邻域内增加或减少占用膨胀计数
        /// @param id_g 中心栅格的全局索引
        /// @param is_hit true=增加膨胀（变为占用），false=减少膨胀（不再占用）
        void updateInflation(const Vec3i &id_g, const bool is_hit);

        /// 更新未知膨胀：在球面邻域内增加或减少未知膨胀计数
        /// @param id_g 中心栅格的全局索引
        /// @param is_add true=增加膨胀，false=减少膨胀
        void updateUnkInflation(const Vec3i &id_g, const bool is_add);

        /// 重置单个栅格：恢复为UNKNOWN状态
        void resetOneCell(const int &hash_id) override;
    };
}
