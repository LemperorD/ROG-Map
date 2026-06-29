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

#include "rog_map/esdf_map.h"

namespace rog_map {

    /// 初始化ESDF地图
    /// 继承CounterMap的初始化逻辑，并分配ESDF专用的缓冲区
    void ESDFMap::initESDFMap(const rog_map::Vec3i &half_prob_map_size_i, const double &prob_map_resolution,
                              const double &temp_counter_map_resolution, const rog_map::Vec3f &local_update_box,
                              const bool &map_sliding_en, const double &sliding_thresh,
                              const rog_map::Vec3f &fix_map_origin, const double &unk_thresh) {

        if (had_been_initialized) {
            throw std::runtime_error(" -- [ESDFMap]: init can only be called once!");
        }
        had_been_initialized = true;

        // 初始化基类CounterMap（no inflation step, step=0）
        initCounterMap(half_prob_map_size_i,
                       prob_map_resolution,
                       temp_counter_map_resolution,
                       0,  // ESDF不需要膨胀边界
                       map_sliding_en,
                       sliding_thresh,
                       fix_map_origin,
                       unk_thresh);

        // 分配距离缓冲区和临时缓冲区
        distance_buffer.resize(sc_.map_vox_num);
        tmp_buffer1_.resize(sc_.map_vox_num);
        tmp_buffer2_.resize(sc_.map_vox_num);

        // 计算ESDF局部更新范围（以索引为单位）
        posToGlobalIndex(local_update_box, half_local_update_box_i_);
        half_local_update_box_i_ /= 2;

        resetLocalMap();
        std::cout << GREEN << " -- [ESDFMap] Init successfully -- ." << RESET << std::endl;
        printMapInformation();
    }

    void ESDFMap::getUpdatedBbox(rog_map::Vec3f &box_min, rog_map::Vec3f &box_max) const {
        globalIndexToPos(update_local_map_min_i_, box_min);
        globalIndexToPos(update_local_map_max_i_, box_max);
    }

    void ESDFMap::resetLocalMap() {
        std::cout << RED << " -- [ESDFMap] Clear all local map." << RESET << std::endl;
        // 所有子栅格未知，无占用（与InfMap不同，ESDF关心的是占用/空闲的边界）
        std::fill(md_.unknown_cnt.begin(), md_.unknown_cnt.end(), md_.sub_grid_num);
        std::fill(md_.occupied_cnt.begin(), md_.occupied_cnt.end(), 0);
    }

    double ESDFMap::getDistance(const rog_map::Vec3f &pos) const {
        return distance_buffer[getHashIndexFromPos(pos)];
    }

    double ESDFMap::getDistance(const Vec3i &id_g) const {
        return distance_buffer[getHashIndexFromGlobalIndex(id_g)];
    }

    /// 增量更新3D ESDF（核心算法）
    ///
    /// 算法概述（6遍扫掠法）：
    /// 1. Z轴正向扫掠：计算每个XY切片内沿Z轴到最近障碍物的距离（正向距离场）
    /// 2. Y轴扫掠：使用步骤1的结果，计算沿Y轴合并的最小距离
    /// 3. X轴扫掠：使用步骤2的结果，计算沿X轴合并的最小距离 → 正向距离完成
    /// 4-6. 重复上述三遍，但方向相反 → 计算负向距离场
    /// 7. 合并：distance = pos_distance - neg_distance → 符号距离场
    ///
    /// 这种增量更新是ESDF在滑动地图中工作的关键，
    /// 只更新机器人周围 half_local_update_box 范围内的距离值
    void ESDFMap::updateESDF3D(const rog_map::Vec3f &cur_odom) {
        std::lock_guard<std::mutex> lck(update_esdf_mtx);
        using namespace std;
        TimeConsuming up_t("updateESDF3D", false);

        Vec3i id_l, cur_odom_i;
        posToGlobalIndex(cur_odom, cur_odom_i);

        // 计算局部地图在缓冲区中的起始偏移 id_l
        globalIndexToLocalIndex(local_map_bound_min_i_, id_l);
        id_l = id_l + sc_.half_map_size_i;

        // mem_end: 局部索引的"回绕"边界（处理循环缓冲区）
        Vec3i mem_end = sc_.map_size_i - Vec3i::Ones() - id_l;

        /// Lambda: 将3D局部索引（带回绕处理）转换为1D索引
        auto getEsdfLocalIndexHash = [&](const int &id_x,
                                         const int &id_y,
                                         const int &id_z, bool print = false) {
            return id_x * sc_.map_size_i(1) * sc_.map_size_i(2) + id_y * sc_.map_size_i(2) + id_z;
        };

        // 计算ESDF更新范围（以机器人位置为中心）
        Vec3i min_esdf = cur_odom_i - half_local_update_box_i_;
        Vec3i max_esdf = cur_odom_i + half_local_update_box_i_;

        // 将更新范围限制在局部地图内
        update_local_map_min_i_ = min_esdf.cwiseMax(local_map_bound_min_i_);
        update_local_map_max_i_ = max_esdf.cwiseMin(local_map_bound_max_i_) - Vec3i::Ones();

        // 转换到局部缓冲区坐标
        min_esdf = update_local_map_min_i_ - local_map_bound_min_i_;
        max_esdf = update_local_map_max_i_ - local_map_bound_min_i_;

        /// ===== 第1遍: Z轴扫掠（正向距离场：障碍物外部的距离） =====
        /// 对于每个 (x,y) 列，沿Z轴计算到最近障碍物的最小距离
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
                fillESDF(
                        [&](int z) {
                            // 对每个z查询：如果是占用则距离=0，否则无穷大
                            return isOccupied(getEsdfLocalIndexHash(
                                    x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                    y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                    z)) ? 0 : std::numeric_limits<double>::max();
                        },
                        [&](int z, double val) {
                            // 存储到临时缓冲区1
                            tmp_buffer1_[getEsdfLocalIndexHash(
                                    x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                    y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1], z)] = val;
                        },
                        min_esdf[2], max_esdf[2], 2, id_l[2]);
            }
        }

        /// ===== 第2遍: Y轴扫掠（使用第1遍结果） =====
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF([&](int y) {
                             return tmp_buffer1_[getEsdfLocalIndexHash(
                                     x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                     y,
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])];
                         },
                         [&](int y, double val) {
                             tmp_buffer2_[getEsdfLocalIndexHash(
                                     x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                     y,
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])] = val;
                         },
                         min_esdf[1], max_esdf[1], 1, id_l[1]);
            }
        }

        /// ===== 第3遍: X轴扫掠（使用第2遍结果）→ 正向距离完成 =====
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF([&](int x) {
                             return tmp_buffer2_[getEsdfLocalIndexHash(
                                     x,
                                     y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])];
                         },
                         [&](int x, double val) {
                             // 存储到距离缓冲区，开方得到欧几里得距离
                             distance_buffer[getEsdfLocalIndexHash(
                                     x,
                                     y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])] =
                                     sc_.resolution * std::sqrt(val);
                         },
                         min_esdf[0], max_esdf[0], 0, id_l[0]);
            }
        }

        /// ===== 第4-6遍: 负向距离场（障碍物内部的距离） =====
        /// 逻辑与正向类似，但初始值相反

        /// 第4遍: Z轴（负向）
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
                fillESDF(
                        [&](int z) {
                            // 如果是非占用则距离=0，否则无穷大（与正向相反）
                            return isOccupied(getEsdfLocalIndexHash(
                                    x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                    y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                    z)) ? std::numeric_limits<double>::max() : 0;
                        },
                        [&](int z, double val) {
                            tmp_buffer1_[getEsdfLocalIndexHash(
                                    x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                    y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1], z)] = val;
                        },
                        min_esdf[2], max_esdf[2], 2, id_l[2]);
            }
        }

        /// 第5遍: Y轴（负向）
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF([&](int y) {
                             return tmp_buffer1_[getEsdfLocalIndexHash(
                                     x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                     y,
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])];
                         },
                         [&](int y, double val) {
                             tmp_buffer2_[getEsdfLocalIndexHash(
                                     x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                     y,
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])] = val;
                         },
                         min_esdf[1], max_esdf[1], 1, id_l[1]);
            }
        }

        /// 第6遍: X轴（负向）→ 负向距离完成
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF([&](int x) {
                             return tmp_buffer2_[getEsdfLocalIndexHash(
                                     x,
                                     y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])];
                         },
                         [&](int x, double val) {
                             tmp_buffer1_[getEsdfLocalIndexHash(
                                     x,
                                     y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])] =
                                     sc_.resolution * std::sqrt(val);
                         },
                         min_esdf[0], max_esdf[0], 0, id_l[0]);
            }
        }

        /// ===== 第7步: 合并正向和负向距离 =====
        /// 对于内部点（负向>0），从正向距离中减去（负向距离 - 分辨率）
        /// 得到有符号距离：正=外部，负=内部
        for (int x = min_esdf(0); x <= max_esdf(0); ++x)
            for (int y = min_esdf(1); y <= max_esdf(1); ++y)
                for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                    const int idx = getEsdfLocalIndexHash(x, y, z);
                    if (tmp_buffer1_[idx] > 0.0) {
                        // 障碍物内部的点：距离减去偏移量变为负值
                        distance_buffer[idx] += (-tmp_buffer1_[idx] + sc_.resolution);
                    }
                }

        // 如果更新耗时过长（>0.1秒），输出警告
        double dt = up_t.stop();
        if(dt > 0.1) {
            std::cout << "updateESDF3D time: " << dt << std::endl;
        }

#ifdef ESDF_MAP_DEBUG
        // 调试模式：统计平均每体素处理时间
        static int cnt = 0;
        static double t = 0.0;
        t += up_t.stop();
        cnt++;
        std::cout << "Average time per point: " << t / cnt / sc_.map_vox_num * 1e6 << "us" << std::endl;
#endif
    }

    // ===== 可视化函数 =====

    /// 获取ESDF中的占用栅格点云
    void ESDFMap::getESDFOccPC2(const rog_map::Vec3f &box_min_d, const rog_map::Vec3f &box_max_d,
                                sensor_msgs::PointCloud2 &pc2) {
        std::lock_guard<std::mutex> lck(update_esdf_mtx);
        pcl_pc.clear();
        Vec3i box_min_i, box_max_i;
        posToGlobalIndex(box_min_d, box_min_i);
        posToGlobalIndex(box_max_d, box_max_i);

        for (int x = box_min_i.x(); x <= box_max_i.x(); x++) {
            for (int y = box_min_i.y(); y <= box_max_i.y(); y++) {
                for (int z = box_min_i.z(); z <= box_max_i.z(); z++) {
                    Vec3i id_g(x, y, z);
                    if (isOccupied(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        pcl::PointXYZI pt;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        pcl_pc.push_back(pt);
                    }
                }
            }
        }

        pcl_pc.width = pcl_pc.points.size();
        pcl_pc.height = 1;
        pcl_pc.is_dense = true;
        pcl::toROSMsg(pcl_pc, pc2);
        pc2.header.frame_id = "world";
    }

    void ESDFMap::resetOneCell(const int &hash_id) {
        // ESDF不需要在栅格重置时做特殊处理
    }

    /// 获取指定高度层的正距离值点云（障碍物外部，距离≥0）
    void ESDFMap::getPositiveESDFPC2(const rog_map::Vec3f &box_min_d, const rog_map::Vec3f &box_max_d,
                                     const double &visualize_z, sensor_msgs::PointCloud2 &pc2) {
        std::lock_guard<std::mutex> lck(update_esdf_mtx);
        pcl_pc.clear();
        Vec3i box_min_i, box_max_i;
        posToGlobalIndex(box_min_d, box_min_i);
        posToGlobalIndex(box_max_d, box_max_i);
        box_min_i = box_min_i.cwiseMax(update_local_map_min_i_);
        box_max_i = box_max_i.cwiseMin(update_local_map_max_i_);

        int z;
        posToGlobalIndex(visualize_z, z);

        for (int x = box_min_i.x(); x <= box_max_i.x(); x++) {
            for (int y = box_min_i.y(); y <= box_max_i.y(); y++) {
                Vec3i id_g(x, y, z);
                double dist = getDistance(id_g);
                Vec3f pos;
                globalIndexToPos(id_g, pos);
                pcl::PointXYZI pt;
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                pt.intensity = dist + 10.0;  // 距离越大，颜色越亮
                pcl_pc.push_back(pt);
            }
        }
        pcl_pc.width = pcl_pc.points.size();
        pcl_pc.height = 1;
        pcl_pc.is_dense = true;
        pcl::toROSMsg(pcl_pc, pc2);
        pc2.header.frame_id = "world";
    }

    /// 获取指定高度层的负距离值点云（障碍物内部，距离≤0）
    void ESDFMap::getNegativeESDFPC2(const rog_map::Vec3f &box_min_d, const rog_map::Vec3f &box_max_d,
                                     const double &visualize_z, sensor_msgs::PointCloud2 &pc2) {
        std::lock_guard<std::mutex> lck(update_esdf_mtx);
        pcl_pc.clear();
        Vec3i box_min_i, box_max_i;
        posToGlobalIndex(box_min_d, box_min_i);
        posToGlobalIndex(box_max_d, box_max_i);
        box_min_i = box_min_i.cwiseMax(update_local_map_min_i_);
        box_max_i = box_max_i.cwiseMin(update_local_map_max_i_);

        int z;
        posToGlobalIndex(visualize_z, z);

        for (int x = box_min_i.x(); x <= box_max_i.x(); x++) {
            for (int y = box_min_i.y(); y <= box_max_i.y(); y++) {
                Vec3i id_g(x, y, z);
                double dist = -getDistance(id_g);
                Vec3f pos;
                globalIndexToPos(id_g, pos);
                pcl::PointXYZI pt;
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                pt.intensity = dist < 0.0 ? 10.0 : dist;
                pcl_pc.push_back(pt);
            }
        }
        pcl_pc.width = pcl_pc.points.size();
        pcl_pc.height = 1;
        pcl_pc.is_dense = true;
        pcl::toROSMsg(pcl_pc, pc2);
        pc2.header.frame_id = "world";
    }

    /// 核心 1D 距离变换（抛物线下界算法）
    ///
    /// 算法基于 Felzenszwalb & Huttenlocher (2012)：
    /// 给定一维数组 f(q) = 初始距离值，计算
    ///   result(q) = min_p [ (p - q)^2 + f(p) ]
    ///
    /// 使用抛物线的下包络（lower envelope）避免 O(n^2) 暴力计算
    /// 时间复杂度 O(n)
    ///
    /// @param f_get_val 获取初始值的函数
    /// @param f_set_val 存储结果的函数
    /// @param start 扫描起始索引
    /// @param end 扫描结束索引
    /// @param dim 扫描维度（0=X, 1=Y, 2=Z）
    /// @param id_l 局部偏移（用于回绕计算）
    template<typename F_get_val, typename F_set_val>
    void ESDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, const int &start, const int &end, const int &dim,
                           const int &id_l) {
        if (end < 0) return;

        const auto &map_size = sc_.map_size_i(dim);

        // v[k]: 第k个抛物线顶点（最小距离点）的索引
        int v[map_size];
        // z[k]: 第k和第k+1条抛物线交点的位置
        double z[map_size + 1];

        const int mem_end = (map_size - 1) - id_l;  // 回绕边界

        // 初始化：第一条抛物线
        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        /// 构建抛物线下包络
        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            // 寻找新抛物线与当前包络最近抛物线的交点
            do {
                k--;
                int a, b;
                a = q + id_l;
                b = v[k] + id_l;
                // 处理循环缓冲区回绕
                if (q > mem_end) a -= map_size;
                if (v[k] > mem_end) b -= map_size;
                // 计算两条抛物线的交点
                s = ((f_get_val(a) + q * q) - (f_get_val(b) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        /// 使用下包络计算每个位置的最小距离
        k = start;
        for (int q = start; q <= end; q++) {
            // 找到当前q对应区间的抛物线
            while (z[k + 1] < q) k++;

            int a = q + id_l;
            int b = v[k] + id_l;
            if (q > mem_end) a -= map_size;
            if (v[k] > mem_end) b -= map_size;

            // 计算距离并存储
            double val = (q - v[k]) * (q - v[k]) + f_get_val(b);
            f_set_val(a, val);
        }
    }

    // ===== 三线性插值环境函数 =====

    /// 获取目标点周围 2x2x2 = 8 个栅格顶点的坐标
    /// @param pos 目标点世界坐标
    /// @param pts 输出8个顶点世界坐标
    /// @param diff 输出目标点相对于（最小角顶点）的归一化偏移 [0,1]
    void ESDFMap::getSurroundPts(const Vec3f& pos, Vec3f pts[2][2][2], Vec3f & diff){
        // 将目标点偏移半个栅格得到"左下角"
        Vec3f pos_m = pos - 0.5 * sc_.resolution * Vec3f::Ones();

        Vec3i idx;
        Vec3f idx_pos;

        posToGlobalIndex(pos_m, idx);
        globalIndexToPos(idx, idx_pos);
        // 计算归一化偏移量 [0, 1]
        diff = (pos - idx_pos) / sc_.resolution;

        // 遍历 2x2x2 立方体获取8个顶点坐标
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    Vec3f current_pos;
                    globalIndexToPos(current_idx, current_pos);
                    pts[x][y][z] = current_pos;
                }
            }
        }
    }

    /// 获取8个顶点的ESDF距离值
    void ESDFMap::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    dists[x][y][z] = getDistance(pts[x][y][z]);
                }
            }
        }
    }

    /// 获取8个顶点的一阶梯度
    void ESDFMap::getSurroundFirstGrad(Eigen::Vector3d pts[2][2][2], double first_grad[2][2][2][3]){
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    Eigen::Vector3d grad;
                    evaluateFirstGrad(pts[x][y][z], grad);
                    first_grad[x][y][z][0] = grad(0);
                    first_grad[x][y][z][1] = grad(1);
                    first_grad[x][y][z][2] = grad(2);
                }
            }
        }
    }

    /// 评估距离值（三线性插值）
    void ESDFMap::evaluateEDT(const Eigen::Vector3d& pos, double& dist) {
        Eigen::Vector3d diff;
        Eigen::Vector3d sur_pts[2][2][2];
        getSurroundPts(pos, sur_pts, diff);

        double dists[2][2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateTrilinearEDT(dists, diff, dist);
    }

    /// 评估一阶梯度（三线性插值）
    void ESDFMap::evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
        Eigen::Vector3d diff;
        Eigen::Vector3d sur_pts[2][2][2];
        getSurroundPts(pos, sur_pts, diff);

        double dists[2][2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateTrilinearFirstGrad(dists, diff, grad);
    }

    /// 评估二阶梯度（三线性插值）
    void ESDFMap::evaluateSecondGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
        Eigen::Vector3d diff;
        Eigen::Vector3d sur_pts[2][2][2];
        getSurroundPts(pos, sur_pts, diff);

        double first_grad[2][2][2][3];
        getSurroundFirstGrad(sur_pts, first_grad);

        interpolateTrilinearSecondGrad(first_grad, diff, grad);
    }

    /// 三线性插值：距离值
    /// 对 2x2x2 立方体顶点进行三线性插值
    void ESDFMap::interpolateTrilinearEDT(double values[2][2][2],
                                                 const Eigen::Vector3d& diff,
                                                 double& value){
        // X方向线性插值：4条边 → 4个点
        double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
        double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
        double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
        double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];

        // Y方向线性插值：4个点 → 2个点
        double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
        double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

        // Z方向线性插值：2个点 → 最终值
        value = (1 - diff(2)) * v0 + diff(2) * v1;
    }

    /// 三线性插值：一阶梯度
    /// 对距离场进行三线性插值的梯度
    void ESDFMap::interpolateTrilinearFirstGrad(double values[2][2][2],
                                                       const Eigen::Vector3d& diff,
                                                       Eigen::Vector3d& grad) {
        double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
        double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
        double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
        double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
        double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
        double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

        // Z梯度
        grad[2] = (v1 - v0) * sc_.resolution_inv;
        // Y梯度
        grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * sc_.resolution_inv;
        // X梯度
        grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad[0] *= sc_.resolution_inv;
    }

    /// 三线性插值：二阶梯度（海森矩阵的主对角线元素）
    void ESDFMap::interpolateTrilinearSecondGrad(double first_grad[2][2][2][3],
                                                        const Eigen::Vector3d& diff,
                                                        Eigen::Vector3d& grad) {
        // d(distance)/dx^2 的二阶混合偏导近似
        grad[0] =  (1 - diff[1]) * ((1 - diff[2]) * (first_grad[1][0][0][0] - first_grad[0][0][0][0])
                                  + diff[2] * (first_grad[1][0][1][0] - first_grad[0][0][1][0]));
        grad[0] += diff[1] * ((1 - diff[2]) * (first_grad[1][1][0][0] - first_grad[0][1][0][0])
                            + diff[2] * (first_grad[1][1][1][0] - first_grad[0][1][1][0]));
        grad[0] *= sc_.resolution_inv;

        // d(distance)/dy^2
        grad[1] =  (1 - diff[2]) * ((1 - diff[0]) * (first_grad[0][1][0][1] - first_grad[0][0][0][1])
                                  + diff[0] * (first_grad[1][1][0][1] - first_grad[1][0][0][1]));
        grad[1] += diff[2] * ((1 - diff[0]) * (first_grad[0][1][1][1] - first_grad[0][0][1][1])
                            + diff[0] * (first_grad[1][1][1][1] - first_grad[1][0][1][1]));
        grad[1] *= sc_.resolution_inv;

        // d(distance)/dz^2
        grad[2] =  (1 - diff[1]) * ((1 - diff[0]) * (first_grad[0][0][1][2] - first_grad[0][0][0][2])
                                  + diff[0] * (first_grad[1][0][1][2] - first_grad[1][0][0][2]));
        grad[2] += diff[1] * ((1 - diff[0]) * (first_grad[0][1][1][2] - first_grad[0][1][0][2])
                            + diff[0] * (first_grad[1][1][1][2] - first_grad[1][1][0][2]));
        grad[2] *= sc_.resolution_inv;

    }

}
