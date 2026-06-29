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

/// 前沿计数地图类（Free Count Map）
/// 统计每个栅格的已知空闲邻居数量（3x3x3=27邻域）
/// 用于前沿点（Frontier）提取：一个未知栅格如果相邻至少一个已知空闲栅格，则为前沿
///
/// TODO: 继承自 counter map
#pragma once

#include <rog_map/rog_map_core/sliding_map.h>

namespace rog_map {

    class FreeCntMap : public SlidingMap {
    public:
        typedef std::shared_ptr<FreeCntMap> Ptr;

        FreeCntMap(const Vec3i &half_map_size_i,
                   const double &resolution,
                   const bool &sliding_en,
                   const double &sliding_thresh,
                   const Vec3f &fix_map_origin) : SlidingMap(half_map_size_i,
                                                         resolution,
                                                         sliding_en,
                                                         sliding_thresh,
                                                         fix_map_origin) {
            int map_size = sc_.map_size_i.prod();
            neighbor_free_cnt.resize(map_size, 0);           // 初始化空闲邻居计数数组
            resetLocalMap();
            std::cout << GREEN << " -- [InfMap] Init successfully -- ." << RESET << std::endl;
            printMapInformation();
        }

        ~FreeCntMap() = default;

        /// 重置整个前沿计数地图
        void resetLocalMap() override {
            std::cout << RED << " -- [Fro-Map] Clear all local map."<<RESET << std::endl;
            std::fill(neighbor_free_cnt.begin(), neighbor_free_cnt.end(), 0);
        }

        /// 获取指定位置的已知空闲邻居数量
        /// @param pos 世界坐标位置
        /// @return 已知空闲邻居数（0~27）
        int getFreeCnt(const Vec3f &pos) {
            return neighbor_free_cnt[getHashIndexFromPos(pos)];
        }

        /// 获取指定全局索引的已知空闲邻居数量
        int getFreeCnt(const Vec3i &id_g) {
            return neighbor_free_cnt[getHashIndexFromGlobalIndex(id_g)];
        }

        /// 更新前沿计数器
        /// 当某个栅格变为已知空闲或不再为空闲时调用
        /// 影响其 3x3x3 邻居的已知空闲计数
        /// @param id_g 状态变化栅格的全局索引
        /// @param add true=增加邻居空闲计数（变为空闲），false=减少（不再为空闲）
        void updateFrontierCounter(const Vec3i &id_g, bool add) {
            if (!insideLocalMap(id_g)) {
                return;  // 超出地图范围则忽略
            }
            Vec3i neighbor_id_g;
            // 遍历 3x3x3 = 27 邻居
            for (int i = -1; i <= 1; ++i) {
                neighbor_id_g[0] = id_g[0] + i;
                for (int j = -1; j <= 1; ++j) {
                    neighbor_id_g[1] = id_g[1] + j;
                    for (int k = -1; k <= 1; ++k) {
                        neighbor_id_g[2] = id_g[2] + k;
                        int hash_id = getHashIndexFromGlobalIndex(neighbor_id_g);

                        if (add) {
                            neighbor_free_cnt[hash_id] += 1;
                            // 调试：检查溢出（最多27个邻居）
                            if (neighbor_free_cnt[hash_id] > 27) {
                                throw std::runtime_error("Frontier counter overflow with larger than 26");
                            }
                        } else {
                            neighbor_free_cnt[hash_id] -= 1;
                            // 调试：检查下溢
                            if (neighbor_free_cnt[hash_id] < 0) {
                                throw std::runtime_error("Frontier counter overflow with smaller than 0");
                            }
                        }
                    }
                }
            }
        }

        /// 重置单个栅格（地图滑动时调用）
        void resetCell(const int &hash_id) override {
            neighbor_free_cnt[hash_id] = 0;
        }

    private:
        bool map_empty_{true};
        std::vector<int16_t> neighbor_free_cnt;   ///< 每个栅格的已知空闲邻居计数
        rog_map::Config cfg_;

    };

}
