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

#include <rog_map/rog_map_core/sliding_map.h>

using namespace rog_map;

/// 构造函数：将参数转发给 initSlidingMap
SlidingMap::SlidingMap(const Vec3i &half_map_size_i, const double &resolution, const bool &map_sliding_en,
                       const double &sliding_thresh, const Vec3f &fix_map_origin) {
    std::cout<<"half_map_size_i: "<<half_map_size_i.transpose()<<std::endl;
    std::cout<<"resolution: "<<resolution<<std::endl;
    std::cout<<"map_sliding_en: "<<map_sliding_en<<std::endl;
    std::cout<<"sliding_thresh: "<<sliding_thresh<<std::endl;
    std::cout<<"fix_map_origin: "<<fix_map_origin.transpose()<<std::endl;
    initSlidingMap(half_map_size_i, resolution, map_sliding_en, sliding_thresh, fix_map_origin);
}

/// 初始化滑动地图
/// 设置所有内部状态变量，计算地图尺寸
void SlidingMap::initSlidingMap(const rog_map::Vec3i &half_map_size_i, const double &resolution,
                                const bool &map_sliding_en,
                                const double &sliding_thresh, const rog_map::Vec3f &fix_map_origin) {
    // 防止重复初始化
    if (had_been_initialized) {
        throw std::runtime_error(" -- [SlidingMap]: init can only be called once!");
    }
    // 检查 ORIGIN_AT_CORNER 和 ORIGIN_AT_CENTER 不能同时定义
#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
    throw std::runtime_error(" -- [SlidingMap]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both true!");
#endif
#endif
    // 设置基础配置
    sc_.resolution = resolution;
    sc_.resolution_inv = 1.0 / resolution;  // 预计算倒数，用于加速除法
    sc_.map_sliding_en = map_sliding_en;
    sc_.sliding_thresh = sliding_thresh;
    sc_.fix_map_origin = fix_map_origin;
    sc_.half_map_size_i = half_map_size_i;
    sc_.map_size_i = 2 * sc_.half_map_size_i + Vec3i::Constant(1);  // 确保总尺寸为奇数
    sc_.map_vox_num = sc_.map_size_i.prod();  // 总体素数

    // 如果不启用滑动，固定地图原点到指定位置
    if (!map_sliding_en) {
        local_map_origin_d_ = fix_map_origin;
        posToGlobalIndex(local_map_origin_d_, local_map_origin_i_);
    }
    had_been_initialized = true;
}

/// 打印地图配置信息
void SlidingMap::printMapInformation() {
    std::cout << GREEN << "\tresolution: " << sc_.resolution << RESET << std::endl;
    std::cout << GREEN << "\tmap_sliding_en: " << sc_.map_sliding_en << RESET << std::endl;
    std::cout << GREEN << "\tlocal_map_size_i: " << sc_.map_size_i.transpose() << RESET << std::endl;
    std::cout << GREEN << "\tlocal_map_size_d: " << sc_.map_size_i.cast<double>().transpose() * sc_.resolution << RESET
              << std::endl;
}

/// 检查世界坐标是否在局部地图范围内
bool SlidingMap::insideLocalMap(const Vec3f &pos) const {
    Vec3i id_g;
    posToGlobalIndex(pos, id_g);
    return insideLocalMap(id_g);
}

/// 检查全局索引是否在局部地图范围内
/// 判断标准：索引与地图原点的差值绝对值是否超过半地图尺寸
bool SlidingMap::insideLocalMap(const Vec3i &id_g) const {
    if (((id_g - local_map_origin_i_).cwiseAbs() - sc_.half_map_size_i).maxCoeff() > 0) {
        return false;
    }
    return true;
}

/// 更新局部地图原点和边界
/// 设置地图的中心原点，并计算对应的世界坐标边界
void SlidingMap::updateLocalMapOriginAndBound(const rog_map::Vec3f &new_origin_d, const rog_map::Vec3i &new_origin_i) {
    // 更新索引原点和世界坐标原点
    local_map_origin_i_ = new_origin_i;
    local_map_origin_d_ = new_origin_d;

    // 计算边界的全局索引
    local_map_bound_max_i_ = local_map_origin_i_ + sc_.half_map_size_i;
    local_map_bound_min_i_ = local_map_origin_i_ - sc_.half_map_size_i;

    // 将索引边界转换为世界坐标（考虑栅格中心的偏移）
    globalIndexToPos(local_map_bound_min_i_, local_map_bound_min_d_);
    globalIndexToPos(local_map_bound_max_i_, local_map_bound_max_d_);
}

/// 清除地图范围外的内存
/// 当沿某一轴滑动时，该轴的某些切片会移出地图范围，需要重置这些切片的栅格
/// @param clear_id 需要清除的局部索引列表（沿ids[0]轴）
/// @param i 当前滑动的轴向（用于确定清除哪两个轴的全范围）
void SlidingMap::clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) {
    // ids[0]是滑动轴，ids[1]和ids[2]是需要全范围遍历的轴
    vector<int> ids{i, (i + 1) % 3, (i + 2) % 3};
    for (const auto &idd: clear_id) {
        // 遍历另外两个轴的全范围，清除对应的"切片"
        for (int x = -sc_.half_map_size_i(ids[1]); x <= sc_.half_map_size_i(ids[1]); x++) {
            for (int y = -sc_.half_map_size_i(ids[2]); y <= sc_.half_map_size_i(ids[2]); y++) {
                Vec3i temp_clear_id;
                temp_clear_id(ids[0]) = idd;
                temp_clear_id(ids[1]) = x;
                temp_clear_id(ids[2]) = y;
                resetCell(getLocalIndexHash(temp_clear_id));  // 调用子类实现的resetCell
            }
        }
    }
}

/// 地图滑动核心实现
/// 实现零拷贝循环缓冲区：检测里程计位置变化，计算滑动偏移，
/// 清除超出边界的旧数据，更新地图原点。
///
/// 工作原理（论文第III-B节）：
/// 1. 将里程计坐标转换为全局索引
/// 2. 计算新旧原点的偏移量 shift_num
/// 3. 对于每个轴向，清除移出范围的切片
///     - 正方向滑动：清除低索引端
///     - 负方向滑动：清除高索引端
/// 4. 更新地图原点和边界
void SlidingMap::mapSliding(const Vec3f &odom) {
    TimeConsuming update_local_shift_timer("updateLocalShift", false);

    /// 步骤1: 计算新的地图原点索引和世界坐标
    Vec3i new_origin_i;
    posToGlobalIndex(odom, new_origin_i);
    Vec3f new_origin_d = new_origin_i.cast<double>() * sc_.resolution;

    /// 步骤2: 计算偏移量
    Vec3i shift_num = new_origin_i - local_map_origin_i_;

    // 如果偏移量超过地图总尺寸，直接重置整个地图
    for (long unsigned int i = 0; i < 3; i++) {
        if (fabs(shift_num[i]) > sc_.map_size_i[i]) {
            resetLocalMap();  // 偏移过大，清空全部地图
            updateLocalMapOriginAndBound(new_origin_d, new_origin_i);
            return;
        }
    }

    /// 归一化函数：将值映射到 [a, b] 范围内
    /// 用于将局部索引限制在半地图尺寸范围内
    static auto normalize = [](int x, int a, int b) -> int {
        int range = b - a + 1;
        int y = (x - a) % range;
        return (y < 0 ? y + range : y) + a;
    };

    /// 步骤3: 逐轴清除移出范围的栅格
    for (int i = 0; i < 3; i++) {
        if (shift_num[i] == 0) {
            continue;  // 该轴没有滑动，跳过
        }

        // 计算局部索引的最小全局索引和局部索引
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sc_.map_size_i(i);
        vector<int> clear_id;

        if (shift_num(i) > 0) {
            /// 正向滑动：原点向正方向移动，需要清除低索引端（旧的高索引数据变为低索引）
            /// 清除 min_id_l 到 min_id_l + shift_num(i) - 1 的区域
            for (int k = 0; k < shift_num(i); k++) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sc_.half_map_size_i(i), sc_.half_map_size_i(i));
                clear_id.push_back(temp_id);
            }
        } else {
            /// 反向滑动：原点向负方向移动，需要清除高索引端
            /// 清除 min_id_l + shift_num(i) 到 min_id_l - 1 的区域
            for (int k = -1; k >= shift_num(i); k--) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sc_.half_map_size_i(i), sc_.half_map_size_i(i));
                clear_id.push_back(temp_id);
            }
        }

        if (clear_id.empty()) {
            continue;
        }
        clearMemoryOutOfMap(clear_id, i);
    }

    /// 步骤4: 更新地图原点和边界信息
    updateLocalMapOriginAndBound(new_origin_d, new_origin_i);
}

/// 将局部三维索引压缩为一维哈希索引
/// 用于将3D栅格坐标映射到1D数组索引，
/// 公式: hash = (id_x + hx) * size_y * size_z + (id_y + hy) * size_z + (id_z + hz)
/// 其中 h* 是半地图尺寸
int SlidingMap::getLocalIndexHash(const Vec3i &id_in) const {
    Vec3i id = id_in + sc_.half_map_size_i;  // 偏移使索引非负
    return id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) +
           id(1) * sc_.map_size_i(2) +
           id(2);
}

/// 世界坐标 → 全局索引
/// ORIGIN_AT_CORNER: id = floor(pos / resolution)
/// 原点在栅格角上，栅格中心偏移0.5
void SlidingMap::posToGlobalIndex(const Vec3f &pos, Vec3i &id) const {

#ifdef ORIGIN_AT_CENTER
    // 原点在栅格中心：四舍五入取最近栅格
    id = (sc_.resolution_inv * pos + pos.cwiseSign() * 0.5).cast<int>();
#endif

#ifdef ORIGIN_AT_CORNER
    // 原点在栅格角：向下取整
    id = (pos.array() * sc_.resolution_inv).floor().cast<int>();
#endif
}

/// 单轴世界坐标 → 全局索引
void SlidingMap::posToGlobalIndex(const double &pos, int &id) const {
#ifdef ORIGIN_AT_CENTER
    id = static_cast<int>((sc_.resolution_inv * pos + SIGN(pos) * 0.5));
#endif
#ifdef ORIGIN_AT_CORNER
    id = floor(pos * sc_.resolution_inv);
#endif
}

/// 全局索引 → 世界坐标（栅格中心位置）
void SlidingMap::globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const {

#ifdef ORIGIN_AT_CENTER
    // 原点在栅格中心：pos = id * resolution
    pos = id_g.cast<double>() * sc_.resolution;
#endif
#ifdef ORIGIN_AT_CORNER
    // 原点在栅格角：pos = (id + 0.5) * resolution
    pos = (id_g.cast<double>() + Vec3f(0.5, 0.5, 0.5)) * sc_.resolution;
#endif
}

/// 全局索引 → 局部索引（论文公式(7)-(8)）
/// 通过取模操作将无界的全局索引映射到有界的局部缓冲区
/// 公式(7): i_k = id_g(k) mod M_k，其中 M_k 是第k维的地图尺寸
/// 公式(8): 将 i_k 归一化到 [-half_size, half_size] 范围
void SlidingMap::globalIndexToLocalIndex(const Vec3i &id_g, Vec3i &id_l) const {
    for (int i = 0; i < 3; ++i) {
        /// 公式(7): 计算取模索引 i_k
        id_l(i) = id_g(i) % sc_.map_size_i(i);
        /// 公式(8): 归一化到 [-half_map_size, +half_map_size]
        if (id_l(i) > sc_.half_map_size_i(i)) {
            id_l(i) -= sc_.map_size_i(i);
        } else if (id_l(i) < -sc_.half_map_size_i(i)) {
            id_l(i) += sc_.map_size_i(i);
        }
    }
}

/// 局部索引 → 全局索引
/// 局部索引与地图原点的全局索引相加得到全局索引
void SlidingMap::localIndexToGlobalIndex(const Vec3i &id_l, Vec3i &id_g) const {
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sc_.map_size_i(i);
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        id_g(i) = cur_id;
    }
}

/// 局部索引 → 世界坐标
/// 先将局部索引转为全局索引，再转为世界坐标
void SlidingMap::localIndexToPos(const Vec3i &id_l, Vec3f &pos) const {
#ifdef ORIGIN_AT_CENTER
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);

        int min_id_l = min_id_g % sc_.map_size_i(i);
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;

        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        pos(i) = cur_id * sc_.resolution;
    }
#endif

#ifdef ORIGIN_AT_CORNER
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);

        int min_id_l = min_id_g % sc_.map_size_i(i);
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;

        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        pos(i) = (cur_id + 0.5) * sc_.resolution;  // ORIGIN_AT_CORNER: +0.5偏移
    }
#endif

}

/// 一维哈希索引 → 局部三维索引
/// 三维展开的逆操作：从1D索引恢复3D坐标
void SlidingMap::hashIdToLocalIndex(const int &hash_id, Vec3i &id) const {
    id(0) = hash_id / (sc_.map_size_i(1) * sc_.map_size_i(2));  // X = hash / (size_y * size_z)
    id(1) = (hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2)) / sc_.map_size_i(2);  // Y
    id(2) = hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) - id(1) * sc_.map_size_i(2);  // Z
    id -= sc_.half_map_size_i;  // 减去半尺寸偏移，恢复为有符号局部索引
}

/// 一维哈希索引 → 全局索引
void SlidingMap::hashIdToGlobalIndex(const int &hash_id, rog_map::Vec3i &id_g) const {
    Vec3i id;
    hashIdToLocalIndex(hash_id, id);
    localIndexToGlobalIndex(id, id_g);
}

/// 一维哈希索引 → 世界坐标
void SlidingMap::hashIdToPos(const int &hash_id, Vec3f &pos) const {
    Vec3i id;
    hashIdToLocalIndex(hash_id, id);
    localIndexToPos(id, pos);
}

/// 世界坐标 → 一维哈希索引（快捷转换）
int SlidingMap::getHashIndexFromPos(const Vec3f &pos) const {
    Vec3i id_g, id_l;
    posToGlobalIndex(pos, id_g);
    globalIndexToLocalIndex(id_g, id_l);
    return getLocalIndexHash(id_l);
}

/// 全局索引 → 一维哈希索引（快捷转换）
int SlidingMap::getHashIndexFromGlobalIndex(const Vec3i &id_g) const {
    Vec3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    return getLocalIndexHash(id_l);
}