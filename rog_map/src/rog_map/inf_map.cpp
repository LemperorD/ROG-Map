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

#include <rog_map/inf_map.h>

namespace rog_map {

// ===== 公开查询函数 =====

    /// 查询世界坐标位置是否为膨胀占用
    /// 检查流程：1) 是否在地图内 2) 是否超出天花板/地板 3) 占用膨胀计数是否>0
    bool InfMap::isOccupiedInflate(const Vec3f &pos) const {
        if (!insideLocalMap(pos)) return false;                    // 地图外
        if (pos.z() > cfg_.virtual_ceil_height) return false;     // 超出虚拟天花板
        if (pos.z() < cfg_.virtual_ground_height) return false;   // 低于虚拟地板
        return imd_.occ_inflate_cnt[getHashIndexFromPos(pos)] > 0;
    }

    /// 基于全局索引查询：是否为膨胀占用
    bool InfMap::isOccupiedInflate(const Vec3i &id_g) const {
        if (!insideLocalMap(id_g)) return false;
        if (id_g.z() > cfg_.virtual_ceil_height_id_g) return false;
        if (id_g.z() < cfg_.virtual_ground_height_id_g) return false;
        return imd_.occ_inflate_cnt[getHashIndexFromGlobalIndex(id_g)] > 0;
    }

    /// 查询是否为膨胀后的已知空闲
    /// 如果未启用未知膨胀：已知空闲 = 非膨胀占用
    /// 如果启用了未知膨胀：已知空闲 = 非膨胀占用 且 非膨胀未知
    bool InfMap::isKnownFreeInflate(const Vec3f& pos) const {
        if(!cfg_.unk_inflation_en) {
            return !isOccupiedInflate(pos);
        } else {
            return (!isOccupiedInflate(pos) && !isUnknownInflate(pos));
        }
    }

    /// 查询是否为膨胀未知
    /// 仅在启用未知膨胀时有效
    bool InfMap::isUnknownInflate(const Vec3f &pos) const {
        if (!cfg_.unk_inflation_en) {
            throw std::runtime_error(
                    "Unknown inflation is not enabled, but the isUnknownInflate API is called, which should not happen.");
        }
        // 检查虚拟天花板和地板（考虑膨胀分辨率）
        if (pos.z() >= cfg_.virtual_ceil_height - cfg_.inflation_resolution ||
            pos.z() <= cfg_.virtual_ground_height + cfg_.inflation_resolution) {
            return false;
        }
        return imd_.unk_inflate_cnt[getHashIndexFromPos(pos)] > 0;
    }

    /// 构造函数：初始化膨胀地图
    InfMap::InfMap(rog_map::Config &cfg)  {

        cfg_ = cfg;
        // 计算最大膨胀步长
        int max_step = cfg_.inflation_step;
        if (cfg_.unk_inflation_en) {
            max_step = std::max(max_step, cfg_.unk_inflation_step);
        }

        // 初始化基类 CounterMap
        initCounterMap(cfg.half_map_size_i,
                       cfg.resolution,
                       cfg.inflation_resolution,
                       max_step,
                       cfg.map_sliding_en,
                       cfg.map_sliding_thresh,
                       cfg.fix_map_origin,
                       cfg.unk_thresh);

        posToGlobalIndex(cfg.visualization_range, sc_.visualization_range_i);

        // 计算虚拟天花板和地板的全局索引
        cfg.virtual_ceil_height_id_g =
                int(cfg.virtual_ceil_height / cfg.inflation_resolution + SIGN(cfg.inflation_resolution) * 0.5) -
                cfg.inflation_step;
        cfg.virtual_ground_height_id_g =
                int(cfg.virtual_ground_height / cfg.inflation_resolution + SIGN(cfg.inflation_resolution) * 0.5) +
                cfg.inflation_step;
        cfg.virtual_ceil_height = cfg.virtual_ceil_height_id_g * cfg.inflation_resolution;
        cfg.virtual_ground_height = cfg.virtual_ground_height_id_g * cfg.inflation_resolution;

        // 初始化膨胀计数器数组
        imd_.occ_inflate_cnt.resize(sc_.map_vox_num);
        imd_.occ_neighbor_num = cfg.spherical_neighbor.size();  // 球面邻域大小

        if (cfg.unk_inflation_en) {
            imd_.unk_neighbor_num = cfg.unk_spherical_neighbor.size();
            // 初始时所有栅格都是未知的，因此膨胀未知计数器初始化为满值
            imd_.unk_inflate_cnt.resize(sc_.map_vox_num);
        }
        posToGlobalIndex(cfg.visualization_range, sc_.visualization_range_i);

        resetLocalMap();
        cfg_ = cfg;
        std::cout << GREEN << " -- [InfMap] Init successfully -- ." << RESET << std::endl;
        printMapInformation();
    }

    /// 获取膨胀操作的统计信息并重置计数器
    void InfMap::getInflationNumAndTime(double &inf_n, double &inf_t) {
        inf_n = inf_num_;
        inf_num_ = 0;
        inf_t = inf_t_;
        inf_t_ = 0;
    }

    void InfMap::writeMapInfoToLog(std::ofstream &log_file) {
        log_file << "[InfMap]" << std::endl;
        log_file << "\tresolution: " << sc_.resolution << std::endl;
        log_file << "\tmap_size_i: " << sc_.map_size_i.transpose() << std::endl;
        log_file << "\tmap_size_d: " << (sc_.map_size_i.cast<double>() * sc_.resolution).transpose() << std::endl;
    }

    /// 盒子搜索：在给定包围盒内搜索指定类型的栅格
    void InfMap::boxSearch(const Vec3f &box_min, const Vec3f &box_max, const GridType &gt,
                           vec_E<Vec3f> &out_points) const {
        out_points.clear();
        if (map_empty_) {
            std::cout << RED << " -- [ROG] Map is empty, cannot perform box search." << RESET << std::endl;
            return;
        }
        Vec3i box_min_id_g, box_max_id_g;
        posToGlobalIndex(box_min, box_min_id_g);
        posToGlobalIndex(box_max, box_max_id_g);
        Vec3i box_size = box_max_id_g - box_min_id_g;

        if (gt == UNKNOWN) {
            // 搜索未知栅格（需要未知膨胀启用）
            if (!cfg_.unk_inflation_en) {
                out_points.clear();
                std::cout << RED << " -- [ROG] Unknown inflation is not enabled, cannot perform box search." << RESET
                          << std::endl;
                return;
            }
            out_points.reserve(box_size.prod());
            for (int i = box_min_id_g.x(); i <= box_max_id_g.x(); i++) {
                for (int j = box_min_id_g.y(); j <= box_max_id_g.y(); j++) {
                    for (int k = box_min_id_g.z(); k <= box_max_id_g.z(); k++) {
                        Vec3i id_g(i, j, k);
                        if (isUnknown(id_g)) {
                            Vec3f pos;
                            globalIndexToPos(id_g, pos);
                            out_points.push_back(pos);
                        }
                    }
                }
            }
        } else if (gt == OCCUPIED) {
            // 搜索膨胀占用栅格
            out_points.reserve(box_size.prod() / 3);
            for (int i = box_min_id_g.x(); i <= box_max_id_g.x(); i++) {
                for (int j = box_min_id_g.y(); j <= box_max_id_g.y(); j++) {
                    for (int k = box_min_id_g.z(); k <= box_max_id_g.z(); k++) {
                        Vec3i id_g(i, j, k);
                        if (isOccupiedInflate(id_g)) {
                            Vec3f pos;
                            globalIndexToPos(id_g, pos);
                            out_points.push_back(pos);
                        }
                    }
                }
            }
        } else {
            throw std::runtime_error(" -- [ROG-Map] Box search does not support KNOWN_FREE.");
        }

    }

    /// 重置整个膨胀地图
    void InfMap::resetLocalMap() {
        std::cout << RED << " -- [Inf-Map] Clear all local map." << RESET << std::endl;
        // 基类计数器重置：所有子栅格未知，无占用
        std::fill(md_.unknown_cnt.begin(), md_.unknown_cnt.end(), md_.sub_grid_num);
        std::fill(md_.occupied_cnt.begin(), md_.occupied_cnt.end(), 0);
        // 膨胀计数器重置
        std::fill(imd_.occ_inflate_cnt.begin(), imd_.occ_inflate_cnt.end(), 0);
        if (cfg_.unk_inflation_en) {
            // 初始所有栅格都是未知的，满邻居都是未知
            std::fill(imd_.unk_inflate_cnt.begin(), imd_.unk_inflate_cnt.end(), imd_.unk_neighbor_num);
        }
    }

    /// 更新占用膨胀
    /// 遍历球面邻域，增加或减少每个邻居的占用膨胀计数
    void InfMap::updateInflation(const Vec3i &id_g, const bool is_hit) {
        TimeConsuming tc("updateInflation", false);
        for (const auto &nei: cfg_.spherical_neighbor) {
            const Vec3i &id_shift = id_g + nei;  // 邻居的全局索引
#ifdef COUNTER_MAP_DEBUG
            if (!insideLocalMap(id_shift)) {
                throw std::runtime_error(" -- [IM] inflation out of map.");
            }
#endif
            inf_num_++;  // 统计膨胀操作次数
            const int &addr = getHashIndexFromGlobalIndex(id_shift);
            if (is_hit) {
                imd_.occ_inflate_cnt[addr]++;  // 添加占用影响
            } else {
                imd_.occ_inflate_cnt[addr]--;  // 移除占用影响
            }

#ifdef COUNTER_MAP_DEBUG
            // 检查计数器范围合法性
            if (imd_.occ_inflate_cnt[addr] < 0 || imd_.occ_inflate_cnt[addr] > imd_.occ_neighbor_num) {
                imd_.occ_inflate_cnt[addr] = 0;
                throw std::runtime_error(" -- [IM] Negative occupancy counter, which should not happened.!");
            }
#endif

        }
        inf_t_ += tc.stop();
    }

    /// 更新未知膨胀
    /// 与 updateInflation 类似，但作用于未知膨胀计数器
    void InfMap::updateUnkInflation(const Vec3i &id_g, const bool is_add) {
        TimeConsuming tc("updateInflation", false);
        if (!cfg_.unk_inflation_en) {
            std::cout << RED << "Cannot updateUnkInflation of InfMap when unk_inflation_en is false." << RESET
                      << std::endl;
            return;
        }

        for (const auto &nei: cfg_.unk_spherical_neighbor) {
            const Vec3i &id_shift = id_g + nei;
#ifdef COUNTER_MAP_DEBUG
            if (!insideLocalMap(id_shift)) {
                throw std::runtime_error(" -- [IM] Unknown inflation out of map.");
            }
#endif
            inf_num_++;
            const int &addr = getHashIndexFromGlobalIndex(id_shift);
            if (is_add) {
                imd_.unk_inflate_cnt[addr]++;
            } else {
                imd_.unk_inflate_cnt[addr]--;
            }

#ifdef COUNTER_MAP_DEBUG
            if (imd_.unk_inflate_cnt[addr] < 0 || imd_.unk_inflate_cnt[addr] > imd_.unk_neighbor_num) {
                std::cout << "unk_inflate_cnt: " << imd_.unk_inflate_cnt[addr] << " unk_neighbor_num: "
                          << imd_.unk_neighbor_num << std::endl;
                throw std::runtime_error(" -- [IM] Negative occupancy counter, which should not happened.!");
            }
#endif
        }
        inf_t_ += tc.stop();
    }

    /// 跳跃边触发：CounterMap栅格状态改变时更新膨胀
    void InfMap::triggerJumpingEdge(const rog_map::Vec3i& id_g,
        const rog_map::GridType& from_type,
        const rog_map::GridType& to_type) {
        // 如果之前是占用，需要移除占用膨胀
        if (from_type == GridType::OCCUPIED) {
            updateInflation(id_g, false);
        }
        // 如果现在变为占用，需要添加占用膨胀
        if (to_type == GridType::OCCUPIED) {
            updateInflation(id_g, true);
        }
        // 如果启用了未知膨胀
        if (cfg_.unk_inflation_en) {
            // 如果之前是未知，需要移除未知膨胀
            if (from_type == GridType::UNKNOWN) {
                updateUnkInflation(id_g, false);
            }
            // 如果现在变为未知，需要添加未知膨胀
            if (to_type == GridType::UNKNOWN) {
                updateUnkInflation(id_g, true);
            }
        }
    }

    /// 重置单个栅格为未知状态
    void InfMap::resetOneCell(const int& hash_id) {
        GridType cur_grid_type = CounterMap::getGridType(hash_id);
        if (cur_grid_type != GridType::UNKNOWN) {
            Vec3i id_g;
            hashIdToGlobalIndex(hash_id, id_g);
            // 触发状态变化：从当前类型转为UNKNOWN
            triggerJumpingEdge(id_g, cur_grid_type, GridType::UNKNOWN);
        }
    }

    /// 获取栅格类型（基于全局索引）
    GridType InfMap::getGridType(const Vec3i& id_g) const {
        if (!insideLocalMap(id_g)) {
            return OUT_OF_MAP;
        }
        Vec3i id_l;
        globalIndexToLocalIndex(id_g, id_l);
        int addr = getLocalIndexHash(id_l);
        // 膨胀层的判断逻辑
        if (imd_.occ_inflate_cnt[addr] > 0) {
            return OCCUPIED;  // 有占用邻居
        } else if (cfg_.unk_inflation_en && imd_.unk_inflate_cnt[addr] > 0) {
            return UNKNOWN;   // 有未知邻居（且无占用邻居）
        } else {
            return KNOWN_FREE;  // 无占用也无未知邻居
        }
    }

    /// 获取栅格类型（基于世界坐标）
    GridType InfMap::getGridType(const Vec3f& pos) const  {
        Vec3i id_g, id_l;
        // 边界检查：考虑膨胀分辨率
        if (pos.z() >= cfg_.virtual_ceil_height  - cfg_.inflation_resolution ||
            pos.z() <= cfg_.virtual_ground_height  + cfg_.inflation_resolution) {
            return OCCUPIED;
            }
        posToGlobalIndex(pos, id_g);
        return getGridType(id_g);
    }

}