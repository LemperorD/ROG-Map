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

#include <rog_map/rog_map_core/counter_map.h>

using namespace rog_map;


namespace rog_map {

    /// 初始化计数器地图
    /// 根据基础概率地图的尺寸和分辨率，计算计数器地图的尺寸和分辨率
    /// 确保计数器地图:
    /// 1) 分辨率是基础地图分辨率的整数倍（或奇数倍，取决于ORIGIN模式）
    /// 2) 尺寸大于基础地图（包含膨胀步长边界）
    void CounterMap::initCounterMap(
            const Vec3i &half_prob_map_size_i,        ///< 基础概率地图半尺寸（输入，确保奇数）
            const double &prob_map_resolution,         ///< 基础概率地图分辨率
            const double &temp_counter_map_resolution, ///< 期望的计数器地图分辨率（会被调整）
            const int &inflation_step,                 ///< 膨胀步长
            const bool &map_sliding_en,                ///< 是否启用滑动
            const double &sliding_thresh,              ///< 滑动阈值
            const Vec3f &fix_map_origin,               ///< 固定原点
            const double &unk_thresh) {                ///< 未知阈值 [0,1]

        if (had_been_initialized) {
            throw std::runtime_error(" -- [CounterMap]: init can only be called once!");
        }
        had_been_initialized = true;

        /// 1) 计算分辨率比率
        /// 计数器地图的分辨率应为概率地图分辨率的整数倍（CORNER模式）
        /// 或奇数倍（CENTER模式，以保证栅格对齐）
        if (prob_map_resolution > temp_counter_map_resolution) {
            throw std::invalid_argument(
                    " -- [CounterMap]: prob_map_resolution should be smaller than or equal to counter_map_resolution!");
        }

        if (unk_thresh < 0.0 || unk_thresh > 1.0) {
            throw std::invalid_argument(" -- [InfMap]: unk_thresh should be in [0, 1]!");
        }

        // 计算膨胀比例（计数器地图栅格边长 / 基础地图栅格边长）
        int inflation_ratio = std::round(temp_counter_map_resolution / prob_map_resolution);
#ifdef ORIGIN_AT_CENTER
        // CENTER模式下膨胀比必须为奇数，保证栅格中心对齐
    if (inflation_ratio % 2 == 0) {
        inflation_ratio += 1;
    }
    std::cout << RED << " -- [CounterMap] inflation_ratio: " << inflation_ratio << std::endl;
#endif

        /// 2) 计算计数器滑动地图的索引尺寸
        /// 尺寸 = 基础地图尺寸 + 膨胀步长边界
        double counter_map_resolution = prob_map_resolution * inflation_ratio;

        // 将概率地图的半尺寸转换为世界坐标
        Vec3f half_prob_map_size_d = half_prob_map_size_i.cast<double>() * prob_map_resolution;

        // 计算计数器地图的半尺寸（索引单位），加上膨胀边界
        Vec3i half_counter_map_size_i = (half_prob_map_size_d / counter_map_resolution).cast<int>()
                                        + (inflation_step + 1) * Vec3i::Ones();

        /// 3) 初始化基类 SlidingMap
        initSlidingMap(half_counter_map_size_i, counter_map_resolution, map_sliding_en, sliding_thresh, fix_map_origin);

        // 计算每个计数器栅格包含的子栅格数
        // （计数器分辨率和基础分辨率比值的立方）
        int map_size = sc_.map_size_i.prod();
        md_.sub_grid_num = pow(std::round(counter_map_resolution / prob_map_resolution), 3);
        md_.unk_thresh = ceil(unk_thresh * md_.sub_grid_num);  // 未知阈值转换为子栅格计数
        md_.unk_thresh = std::min(std::max(1, md_.unk_thresh), md_.sub_grid_num);  // 限制在 [1, sub_grid_num]

        // 初始化计数器数组
        md_.unknown_cnt.resize(map_size, md_.sub_grid_num);  // 初始所有栅格都是未知的
        md_.occupied_cnt.resize(map_size, 0);                 // 初始没有占用栅格

        resetLocalMap();
        std::cout << GREEN << " -- [CounterMap] Init successfully -- ." << RESET << std::endl;
        printMapInformation();
    }

    /// 更新栅格计数器
    /// 当基础概率地图的某个子栅格状态改变时调用
    /// 更新子栅格所在CounterMap栅格的占用和未知计数器，
    /// 如果计数器导致CounterMap栅格状态改变，则触发跳跃边回调
    ///
    /// @param pos 子栅格的世界坐标
    /// @param from_type 子栅格变化前的类型
    /// @param to_type 子栅格变化后的类型
    void CounterMap::updateGridCounter(const Vec3f &pos, const GridType &from_type, const GridType &to_type) {
        TimeConsuming update_t("updateGridCounter", false);
        map_empty_ = false;
        Vec3i id_l, id_g;
        posToGlobalIndex(pos, id_g);

#ifdef COUNTER_MAP_DEBUG
        // 调试模式：验证输入合法性
        if (!insideLocalMap(id_g)) {
            throw std::runtime_error(" -- [InfMap]: Update a counter which is not inside the local map.");
        }

        if (from_type == to_type) {
            throw std::runtime_error(" -- [InfMap]: From type is equal to to type.");
        }
#endif
        const int addr = getHashIndexFromGlobalIndex(id_g);

        // 记录变化前的CounterMap栅格状态
        GridType counter_cell_from_type = getGridType(addr);

        /// 更新计数器
        /// from_type的计数减少，to_type的计数增加
        if (from_type == GridType::OCCUPIED) {
            md_.occupied_cnt[addr] -= 1;
        }
        if (to_type == GridType::OCCUPIED) {
            md_.occupied_cnt[addr] += 1;
        }
        if (from_type == GridType::UNKNOWN) {
            md_.unknown_cnt[addr] -= 1;
        }
        if (to_type == GridType::UNKNOWN) {
            md_.unknown_cnt[addr] += 1;
        }

#ifdef COUNTER_MAP_DEBUG
        // 调试模式：检查计数器范围
        if (md_.occupied_cnt[addr] < 0 || md_.occupied_cnt[addr] > md_.sub_grid_num) {
            std::cout << RED << "From type: " << GridTypeStr[from_type] << RESET << std::endl;
            std::cout << RED << "To type: " << GridTypeStr[to_type] << RESET << std::endl;
            std::cout << RED << "Occupied counter: " << md_.occupied_cnt[addr] << RESET << std::endl;
            throw std::runtime_error(" -- [CouterMap]: Occupied counter is out of range.");
        }
        if (md_.unknown_cnt[addr] < 0 || md_.unknown_cnt[addr] > md_.sub_grid_num) {
            std::cout << RED << "From type: " << from_type << RESET << std::endl;
            std::cout << RED << "To type: " << to_type << RESET << std::endl;
            std::cout << RED << "Unknown counter: " << md_.unknown_cnt[addr] << RESET << std::endl;
            throw std::runtime_error(" -- [CouterMap]: Unknown counter is out of range.");
        }
#endif

        // 记录变化后的CounterMap栅格状态
        GridType counter_cell_to_type = getGridType(addr);

        // 如果CounterMap栅格状态发生变化，触发跳跃边回调
        if (counter_cell_from_type != counter_cell_to_type) {
            triggerJumpingEdge(id_g, counter_cell_from_type, counter_cell_to_type);
        }
    }

    /// 基于全局索引查询：是否为未知
    bool CounterMap::isUnknown(const Vec3i &id_g) const {
        return isUnknown(getHashIndexFromGlobalIndex(id_g));
    }

    /// 基于全局索引查询：是否为占用
    bool CounterMap::isOccupied(const Vec3i &id_g) const {
        return isOccupied(getHashIndexFromGlobalIndex(id_g));
    }

    /// 基于全局索引查询：是否为已知空闲
    bool CounterMap::isKnownFree(const Vec3i &id_g) const {
        return isKnownFree(getHashIndexFromGlobalIndex(id_g));
    }

}