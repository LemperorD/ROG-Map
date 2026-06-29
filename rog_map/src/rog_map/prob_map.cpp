/**
* 本文件是 ROG-Map 的一部分
* ... (license header) ...
*/

#include <rog_map/prob_map.h>
using namespace rog_map;

void ProbMap::initProbMap() {
    static bool init_once{false};
    if(init_once) {
        throw std::runtime_error(" -- [ROGMap] ProbMap can only init once.");
    }
    init_once = true;

    // 初始化滑动地图基类
    initSlidingMap(cfg_.half_map_size_i, cfg_.resolution,
                   cfg_.map_sliding_en, cfg_.map_sliding_thresh,
                   cfg_.fix_map_origin);
    time_consuming_.resize(7);  // 对应7个耗时统计指标

    // 初始化膨胀地图
    inf_map_ = std::make_shared<InfMap>(cfg_);

    // 如果启用前沿提取，初始化前沿计数地图
    if (cfg_.frontier_extraction_en) {
        fcnt_map_ = std::make_shared<FreeCntMap>(cfg_.half_map_size_i + Vec3i::Constant(2),
                                       cfg_.resolution,
                                       cfg_.map_sliding_en,
                                       cfg_.map_sliding_thresh,
                                       cfg_.fix_map_origin);
    }

    // 如果启用ESDF，初始化ESDF地图
    if (cfg_.esdf_en) {
        esdf_map_ = std::make_shared<ESDFMap>();
        esdf_map_->initESDFMap(cfg_.half_map_size_i,
                               cfg_.resolution,
                               cfg_.esdf_resolution,
                               cfg_.esdf_local_update_box,
                               cfg_.map_sliding_en,
                               cfg_.map_sliding_thresh,
                               cfg_.fix_map_origin,
                               cfg_.unk_thresh);
    }

    // 将可视化范围和虚拟边界转换为索引单位
    posToGlobalIndex(cfg_.visualization_range, sc_.visualization_range_i);
    posToGlobalIndex(cfg_.virtual_ceil_height, sc_.virtual_ceil_height_id_g);
    posToGlobalIndex(cfg_.virtual_ground_height, sc_.virtual_ground_height_id_g);

    cfg_.virtual_ceil_height = sc_.virtual_ceil_height_id_g * cfg_.resolution;
    cfg_.virtual_ground_height = sc_.virtual_ground_height_id_g * cfg_.resolution;

    // 如果不启用地图滑动，将地图固定在指定原点
    if (!cfg_.map_sliding_en) {
        cout << YELLOW << " -- [ProbMap] Map sliding disabled, set origin to [" << cfg_.fix_map_origin.transpose()
            << "] -- " << RESET << endl;
        slideAllMap(cfg_.fix_map_origin);
    }

    int map_size = sc_.map_size_i.prod();

    // 分配缓冲区内存
    occupancy_buffer_.resize(map_size, 0);                        // 占用概率缓冲区（0=未知）
    raycast_data_.raycaster.setResolution(cfg_.resolution);
    raycast_data_.operation_cnt.resize(map_size, 0);              // 操作计数
    raycast_data_.hit_cnt.resize(map_size, 0);                    // 命中计数

    resetLocalMap();

    cout << GREEN << " -- [ProbMap] Init successfully -- ." << RESET << endl;
    printMapInformation();
}

// ===== 查询函数 =====

Vec3f ProbMap::getLocalMapOrigin() const { return local_map_origin_d_; }
Vec3f ProbMap::getLocalMapSize() const { return cfg_.map_size_d; }

/// 查询世界坐标位置是否被占用
/// 流程：检查是否在地图内 → 检查是否超出虚拟天花板/地板 → 查询对数几率值
bool ProbMap::isOccupied(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return false;  // 地图外视为非占用
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return true;   // 虚拟天花板以上和地板以下视为占用
    }
    return isOccupied(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

/// 查询世界坐标位置是否未知
bool ProbMap::isUnknown(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return true;   // 地图外视为未知
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return false;  // 虚拟边界外不视为未知
    }
    return isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

/// 查询世界坐标位置是否为已知空闲
bool ProbMap::isKnownFree(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return false;
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return false;
    }
    return isKnownFree(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

/// 查询世界坐标位置是否为前沿（探索边界）
/// 前沿 = 未知栅格 + 相邻已知空闲栅格 > 0
bool ProbMap::isFrontier(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) return false;
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) return false;

    // 如果是未知栅格，检查是否有已知空闲邻居
    if (isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)])) {
        if (fcnt_map_->getFreeCnt(pos)) {
            return true;
        }
    }
    return false;
}

// ===== 基于全局索引的查询（内部使用） =====

bool ProbMap::isOccupied(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) return false;
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g) return true;
    return isOccupied(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isUnknown(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) return true;
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g) return false;
    return isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isKnownFree(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) return false;
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g) return true;
    return isKnownFree(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isFrontier(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) return false;
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g) return false;

    if (isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)])) {
        if (fcnt_map_->getFreeCnt(id_g) > 0) {
            return true;
        }
    }
    return false;
}

// ===== 膨胀查询委托给 InfMap =====

bool ProbMap::isKnownFreeInflate(const Vec3f& pos) const { return inf_map_->isKnownFreeInflate(pos); }
bool ProbMap::isOccupiedInflate(const Vec3f& pos) const { return inf_map_->isOccupiedInflate(pos); }
bool ProbMap::isUnknownInflate(const Vec3f& pos) const { return inf_map_->isUnknownInflate(pos); }

// ===== 日志 =====

void ProbMap::writeTimeConsumingToLog(std::ofstream& log_file) {
    for (long unsigned int i = 0; i < time_consuming_.size(); i++) {
        log_file << time_consuming_[i];
        if (i != time_consuming_.size() - 1) log_file << ", ";
    }
    log_file << endl;
}

void ProbMap::writeMapInfoToLog(std::ofstream& log_file) {
    log_file << "[ProbMap]" << endl;
    log_file << "\tmap_size_d: " << cfg_.map_size_d.transpose() << endl;
    log_file << "\tresolution: " << cfg_.resolution << endl;
    log_file << "\tmap_size_i: " << sc_.map_size_i.transpose() << endl;
    log_file << "\tlocal_update_box_size: " << cfg_.local_update_box_d.transpose() << endl;
    log_file << "\tp_min: " << cfg_.p_min << endl;
    log_file << "\tp_max: " << cfg_.p_max << endl;
    log_file << "\tp_hit: " << cfg_.p_hit << endl;
    log_file << "\tp_miss: " << cfg_.p_miss << endl;
    log_file << "\tp_occ: " << cfg_.p_occ << endl;
    log_file << "\tp_free: " << cfg_.p_free << endl;
    log_file << "\tunk_thresh: " << cfg_.unk_thresh << endl;
    log_file << "\tmap_sliding_thresh: " << cfg_.map_sliding_thresh << endl;
    log_file << "\tmap_sliding_en: " << cfg_.map_sliding_en << endl;
    log_file << "\tfix_map_origin: " << cfg_.fix_map_origin.transpose() << endl;
    log_file << "\tvisualization_range: " << cfg_.visualization_range.transpose() << endl;
    log_file << "\tvirtual_ceil_height: " << cfg_.virtual_ceil_height << endl;
    log_file << "\tvirtual_ground_height: " << cfg_.virtual_ground_height << endl;
    log_file << "\tbatch_update_size: " << cfg_.batch_update_size << endl;
    log_file << "\tfrontier_extraction_en: " << cfg_.frontier_extraction_en << endl;
    inf_map_->writeMapInfoToLog(log_file);
}

/// 从PCD点云直接更新地图
/// 不使用射线投射，将每个点视为占用命中，
/// 主要用于加载预构建的地图（如示例中的森林PCD文件）
void ProbMap::updateOccPointCloud(const PointCloud& input_cloud) {
    const int cloud_in_size = input_cloud.size();
    Vec3f localmap_min = local_map_bound_min_d_;
    Vec3f localmap_max = local_map_bound_max_d_;

    for (int i = 0; i < cloud_in_size; i++) {
        static Vec3f p, ray_pt;
        static Vec3i pt_id_g, pt_id_l;

        // 强度过滤
        if (cfg_.intensity_thresh > 0 &&
            input_cloud[i].intensity < cfg_.intensity_thresh) {
            continue;
        }

        p.x() = input_cloud[i].x;
        p.y() = input_cloud[i].y;
        p.z() = input_cloud[i].z;

        posToGlobalIndex(p, pt_id_g);

        // 跳过虚拟边界外的点
        if (p.z() > cfg_.virtual_ceil_height || p.z() < cfg_.virtual_ground_height) {
            continue;
        }

        if (insideLocalMap(pt_id_g)) {
            // 计算需要多少次命中才能将栅格标记为占用
            const int occ_hit_num = ceil(cfg_.l_occ / cfg_.l_hit);
            for (int j = 0; j < occ_hit_num; j++) {
                insertUpdateCandidate(pt_id_g, true);  // 插入占用命中
            }
            localmap_max = localmap_max.cwiseMax(p);
            localmap_min = localmap_min.cwiseMin(p);
        }
    }

    local_map_bound_min_d_ = localmap_min;
    local_map_bound_max_d_ = localmap_max;
    probabilisticMapFromCache();  // 批量应用更新
    map_empty_ = false;
}

/// 滑动所有子地图（概率地图 + 膨胀地图 + 前沿地图 + ESDF地图）
void ProbMap::slideAllMap(const rog_map::Vec3f& pos) {
    mapSliding(pos);
    inf_map_->mapSliding(pos);
    if (cfg_.frontier_extraction_en) {
        fcnt_map_->mapSliding(pos);
    }
    if (cfg_.esdf_en) {
        esdf_map_->mapSliding(pos);
    }
}

/// 更新概率地图的主入口函数
/// 每帧调用一次，执行：
/// 1) 检查是否需要地图滑动
/// 2) 更新局部更新盒子位置
/// 3) 执行射线投射 → 缓存更新
/// 4) 批量应用概率更新
/// 5) 更新ESDF（如果启用）
void ProbMap::updateProbMap(const PointCloud& cloud, const Pose& pose) {
    TimeConsuming tc("updateMap", false);
    const Vec3f& pos = pose.first;
    time_consuming_[4] = cloud.size();  // 记录输入点云大小

    // 如果地图滑动启用且机器人移出地图，重置整张地图
    if (cfg_.map_sliding_en && !insideLocalMap(pos) && raycast_data_.batch_update_counter == 0) {
        cout << YELLOW << " -- [ROGMapCore] cur_pose out of map range, reset the map." << RESET << endl;
        cout << YELLOW << " -- [ROGMapCore] Sliding to map center at: " << pos.transpose() << RESET << endl;
        slideAllMap(pos);
        return;
    }

    // 检查虚拟边界
    if (pos.z() > cfg_.virtual_ceil_height) {
        cout << RED << " -- [ROGMapCore] Odom above virtual ceil, please check map parameter -- ." << RESET << endl;
        return;
    } else if (pos.z() < cfg_.virtual_ground_height) {
        cout << RED << " -- [ROGMapCore] Odom below virtual ground, please check map parameter -- ." << RESET << endl;
        return;
    }

    // 检查是否需要地图滑动（距离超过阈值）
    if (raycast_data_.batch_update_counter == 0
        && (map_empty_ ||
            (cfg_.map_sliding_en && (pos - local_map_origin_d_).norm() > cfg_.map_sliding_thresh))) {
        slideAllMap(pos);
    }

    // 更新局部更新盒子位置
    updateLocalBox(pos);

    // 执行射线投射
    TimeConsuming t_raycast("raycast", false);
    raycastProcess(cloud, pos);
    time_consuming_[1] = t_raycast.stop();

    // 批量更新计数
    raycast_data_.batch_update_counter++;
    if (raycast_data_.batch_update_counter >= cfg_.batch_update_size) {
        raycast_data_.batch_update_counter = 0;
        time_consuming_[5] = raycast_data_.update_cache_id_g.size();  // 缓存栅格数
        TimeConsuming t_update("update", false);
        probabilisticMapFromCache();  // 批量应用缓存更新
        time_consuming_[2] = t_update.stop();
        map_empty_ = false;
    }

    // 获取膨胀操作的统计信息
    inf_map_->getInflationNumAndTime(time_consuming_[6], time_consuming_[3]);
    time_consuming_[0] = tc.stop();  // 总耗时

    // 更新ESDF地图
    if (cfg_.esdf_en) {
        esdf_map_->updateESDF3D(pos);
    }

    // 第一帧处理：将机器人周围的近场区域强制标记为已知空闲
    // 这确保机器人附近的起始区域始终可通行
    static bool first = true;
    if (first) {
        first = false;
        for (double dx = -cfg_.raycast_range_min; dx <= cfg_.raycast_range_min; dx += cfg_.resolution) {
            for (double dy = -cfg_.raycast_range_min; dy <= cfg_.raycast_range_min; dy += cfg_.resolution) {
                for (double dz = -cfg_.raycast_range_min; dz <= cfg_.raycast_range_min; dz += cfg_.resolution) {
                    Vec3f p(dx, dy, dz);
                    if (p.norm() <= cfg_.raycast_range_min) {
                        Vec3f pp = pos + p;
                        int hash_id = getHashIndexFromPos(pp);
                        missPointUpdate(pp, hash_id, 999);  // 强力标记为已知空闲
                    }
                }
            }
        }
    }
}

/// 获取栅格类型（基于全局索引）
GridType ProbMap::getGridType(Vec3i& id_g) const {
    if (id_g.z() <= sc_.virtual_ground_height_id_g ||
        id_g.z() >= sc_.virtual_ceil_height_id_g) {
        return OCCUPIED;  // 虚拟边界外为占用
    }
    if (!insideLocalMap(id_g)) {
        return OUT_OF_MAP;
    }
    Vec3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    int hash_id = getLocalIndexHash(id_l);
    double ret = occupancy_buffer_[hash_id];

    if (isKnownFree(ret)) return GridType::KNOWN_FREE;
    else if (isOccupied(ret)) return GridType::OCCUPIED;
    else return GridType::UNKNOWN;
}

GridType ProbMap::getGridType(const Vec3f& pos) const {
    if (pos.z() <= cfg_.virtual_ground_height ||
        pos.z() >= cfg_.virtual_ceil_height) return OCCUPIED;
    if (!insideLocalMap(pos)) return OUT_OF_MAP;
    Vec3i id_g, id_l;
    posToGlobalIndex(pos, id_g);
    return getGridType(id_g);
}

GridType ProbMap::getInfGridType(const Vec3f& pos) const {
    return inf_map_->getGridType(pos);
}

double ProbMap::getMapValue(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) return 0;
    return occupancy_buffer_[getHashIndexFromPos(pos)];
}

/// 盒子搜索：在包围盒内搜索指定类型的所有栅格
/// 支持 UNKNOWN（未知）、OCCUPIED（占用）、FRONTIER（前沿）三种类型
void ProbMap::boxSearch(const Vec3f& _box_min, const Vec3f& _box_max, const GridType& gt,
                         vec_E<Vec3f>& out_points) const {
    out_points.clear();
    if (map_empty_) {
        cout << YELLOW << " -- [ROG] Map is empty, cannot perform box search." << RESET << endl;
        return;
    }
    if ((_box_max - _box_min).minCoeff() <= 0) {
        cout << YELLOW << " -- [ROG] Box search failed, box size is zero." << RESET << endl;
        return;
    }

    // 将搜索范围限制在局部地图内
    Vec3f box_min_d = _box_min, box_max_d = _box_max;
    boundBoxByLocalMap(box_min_d, box_max_d);
    if ((box_max_d - box_min_d).minCoeff() <= 0) {
        cout << YELLOW << " -- [ROG] Box search failed, box size is zero." << RESET << endl;
        return;
    }

    Vec3i box_min_id_g, box_max_id_g;
    posToGlobalIndex(box_min_d, box_min_id_g);
    posToGlobalIndex(box_max_d, box_max_id_g);
    Vec3i box_size = box_max_id_g - box_min_id_g;

    if (gt == UNKNOWN) {
        out_points.reserve(box_size.prod());
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
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
        out_points.reserve(box_size.prod() / 3);
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Vec3i id_g(i, j, k);
                    if (isOccupied(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    } else if (gt == FRONTIER) {
        out_points.reserve(box_size.prod() / 3);
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Vec3i id_g(i, j, k);
                    if (isFrontier(id_g)) {
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

void ProbMap::boxSearchInflate(const Vec3f& box_min, const Vec3f& box_max, const GridType& gt,
                               vec_E<Vec3f>& out_points) const {
    inf_map_->boxSearch(box_min, box_max, gt, out_points);
}

/// 将包围盒限制在局部地图范围内
/// 同时考虑虚拟天花板和地板限制
void ProbMap::boundBoxByLocalMap(Vec3f& box_min, Vec3f& box_max) const {
    if ((box_max - box_min).minCoeff() <= 0) {
        box_min = box_max;
        cout << RED << "-- [ROG] Bound box is invalid." << RESET << endl;
        return;
    }

    box_min = box_min.cwiseMax(local_map_bound_min_d_);
    box_max = box_max.cwiseMin(local_map_bound_max_d_);
    box_max.z() = std::min(box_max.z(), cfg_.virtual_ceil_height);
    box_min.z() = std::max(box_min.z(), cfg_.virtual_ground_height);
}

/// 重置单个栅格
/// 在地图滑动时调用，将栅格状态恢复为未知，
/// 同时更新膨胀地图和ESDF地图中的相关计数器
void ProbMap::resetCell(const int& hash_id) {
    float& ret = occupancy_buffer_[hash_id];
    if (isOccupied(ret)) {
        // 如果当前是占用状态 → 恢复为未知
        Vec3f pos;
        hashIdToPos(hash_id, pos);
        inf_map_->updateGridCounter(pos, OCCUPIED, UNKNOWN);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(pos, OCCUPIED, UNKNOWN);
        }
    } else if (isKnownFree(ret)) {
        // 如果当前是已知空闲状态 → 恢复为未知
        Vec3f pos;
        hashIdToPos(hash_id, pos);
        inf_map_->updateGridCounter(pos, KNOWN_FREE, UNKNOWN);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(pos, KNOWN_FREE, UNKNOWN);
        }
        // 更新前沿计数器（如果启用）
        if (cfg_.frontier_extraction_en) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            fcnt_map_->updateFrontierCounter(id_g, false);
        }
    }
    // 未知状态 → 不需要更新计数器
    ret = 0;  // 对数几率清零（即未知状态）
}

/// 从缓存中批量应用概率更新
/// 遍历待更新队列，区分命中点和未命中点，
/// 分别调用 hitPointUpdate 和 missPointUpdate
void ProbMap::probabilisticMapFromCache() {
    while (!raycast_data_.update_cache_id_g.empty()) {
        Vec3f pos;
        Vec3i id_g = raycast_data_.update_cache_id_g.front();
        raycast_data_.update_cache_id_g.pop();
        Vec3i id_l;
        globalIndexToLocalIndex(id_g, id_l);
        int hash_id = getLocalIndexHash(id_l);
        globalIndexToPos(id_g, pos);

        if (raycast_data_.hit_cnt[hash_id] > 0) {
            // 有命中 → 增加占用概率
            hitPointUpdate(pos, hash_id, raycast_data_.hit_cnt[hash_id]);
        } else {
            // 只有未命中 → 减少占用概率（趋向已知空闲）
            missPointUpdate(pos, hash_id,
                            raycast_data_.operation_cnt[hash_id] - raycast_data_.hit_cnt[hash_id]);
        }

        // 重置该栅格的计数
        raycast_data_.hit_cnt[hash_id] = 0;
        raycast_data_.operation_cnt[hash_id] = 0;
    }
}

/// 命中点更新：对数几率 += l_hit * hit_num（趋向占用）
/// 如果状态发生变化，通知膨胀地图和ESDF地图
void ProbMap::hitPointUpdate(const Vec3f& pos, const int& hash_id, const int& hit_num) {
    float& ret = occupancy_buffer_[hash_id];

    // 记录更新前的状态
    GridType from_type = UNDEFINED;
    if (isOccupied(ret)) from_type = GridType::OCCUPIED;
    else if (isKnownFree(ret)) from_type = GridType::KNOWN_FREE;
    else from_type = GridType::UNKNOWN;

    // 应用对数几率增量
    ret += cfg_.l_hit * hit_num;
    if (ret > cfg_.l_max) ret = cfg_.l_max;  // 限制上限

    // 检查状态是否改变
    GridType to_type;
    if (isOccupied(ret)) to_type = GridType::OCCUPIED;
    else if (isKnownFree(ret)) to_type = GridType::KNOWN_FREE;
    else to_type = GridType::UNKNOWN;

    // 如果状态改变，更新子地图
    if (from_type != to_type) {
        Vec3f center_pos;
        Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        globalIndexToPos(id_g, center_pos);
        inf_map_->updateGridCounter(center_pos, from_type, to_type);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(center_pos, from_type, to_type);
        }
        // 如果原来是已知空闲，变为占用后需要减少前沿计数
        if (cfg_.frontier_extraction_en && from_type == KNOWN_FREE) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            fcnt_map_->updateFrontierCounter(id_g, false);
        }
    }
}

/// 未命中点更新：对数几率 += l_miss * hit_num（趋向已知空闲）
/// 与 hitPointUpdate 对称，但方向相反
void ProbMap::missPointUpdate(const Vec3f& pos, const int& hash_id, const int& hit_num) {
    float& ret = occupancy_buffer_[hash_id];

    GridType from_type;
    if (isOccupied(ret)) from_type = GridType::OCCUPIED;
    else if (isKnownFree(ret)) from_type = GridType::KNOWN_FREE;
    else from_type = GridType::UNKNOWN;

    // 应用对数几率减量
    ret += cfg_.l_miss * hit_num;
    if (ret < cfg_.l_min) ret = cfg_.l_min;  // 限制下限

    GridType to_type;
    if (isOccupied(ret)) to_type = GridType::OCCUPIED;
    else if (isKnownFree(ret)) to_type = GridType::KNOWN_FREE;
    else to_type = GridType::UNKNOWN;

    // 捕捉跳跃边（状态转换）
    if (from_type != to_type) {
        Vec3f center_pos;
        Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        globalIndexToPos(id_g, center_pos);
        inf_map_->updateGridCounter(center_pos, from_type, to_type);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(center_pos, from_type, to_type);
        }

        // 如果变为已知空闲，增加前沿计数（使相邻的未知栅格可能成为前沿）
        if (cfg_.frontier_extraction_en && to_type == KNOWN_FREE) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            fcnt_map_->updateFrontierCounter(id_g, true);
        }
    }
}

/// 射线投射处理
/// 核心步骤如下：
/// 1) 对每个点云点进行降采样和强度过滤
/// 2) 处理虚拟天花板/地板裁剪
/// 3) 处理射线范围裁剪
/// 4) 处理局部更新盒子裁剪
/// 5) 有效命中点插入缓存
/// 6) 沿射线方向从 raycast_range_min 到命中点遍历，标记未命中
void ProbMap::raycastProcess(const PointCloud& input_cloud, const Vec3f& cur_odom) {
    // 初始化缓存盒子边界
    raycast_data_.cache_box_min = cur_odom;
    raycast_data_.cache_box_max = cur_odom;
    Vec3f raycast_box_min, raycast_box_max;

    {
        std::lock_guard<std::mutex> lck{raycast_data_.raycast_range_mtx};
        raycast_box_max = raycast_data_.local_update_box_max;
        raycast_box_min = raycast_data_.local_update_box_min;
    }

    const int& cloud_in_size = input_cloud.size();
    auto raycasting_cloud = vec_Vec3f{};
    raycasting_cloud.reserve(cloud_in_size);

    /// 步骤1: 遍历所有点云点
    int temperol_cnt{0};
    for (const auto& pcl_p : input_cloud) {
        // 1.1) 强度过滤：跳过强度低于阈值的点（可能是噪声）
        if (cfg_.intensity_thresh > 0 &&
            pcl_p.intensity < cfg_.intensity_thresh) {
            continue;
        }

        // 1.2) 时间降采样：每隔 point_filt_num 个点取一个
        if (temperol_cnt++ % cfg_.point_filt_num) {
            continue;
        }

        Vec3f p(pcl_p.x, pcl_p.y, pcl_p.z);
        Vec3i pt_id_g;

        // 如果不启用射线投射，直接将点云点作为占用命中插入
        if (!cfg_.raycasting_en) {
            if(insideLocalMap(p)) {
                posToGlobalIndex(p, pt_id_g);
                insertUpdateCandidate(pt_id_g, true);
                raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
                raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);
            }
            continue;
        }

        bool update_hit{true};

        // 1.3) 虚拟天花板和地板过滤
        // 将天花板/地板之上的点截断到边界平面
        if (p.z() > cfg_.virtual_ceil_height) {
            update_hit = false;  // 交点不是真实障碍物
            const double dz = p.z() - cur_odom.z();
            const double pc = cfg_.virtual_ceil_height - cur_odom.z();
            p = cur_odom + (p - cur_odom).normalized() * pc / dz;
        } else if (p.z() < cfg_.virtual_ground_height) {
            update_hit = false;
            const double dz = p.z() - cur_odom.z();
            const double pc = cfg_.virtual_ground_height - cur_odom.z();
            p = cur_odom + (p - cur_odom).normalized() * pc / dz;
        }

        // 1.4) 射线最大范围过滤
        const double sqr_dis = (p - cur_odom).squaredNorm();
        if (sqr_dis > cfg_.sqr_raycast_range_max) {
            double k = cfg_.raycast_range_max / sqrt(sqr_dis);
            p = k * (p - cur_odom) + cur_odom;  // 截断到最大范围
            update_hit = false;  // 截断点不是真实障碍物
        }

        // 局部更新盒子过滤
        if (((p - raycast_box_min).minCoeff() < 0) ||
            ((p - raycast_box_max).maxCoeff() > 0)) {
            p = lineBoxIntersectPoint(p, cur_odom, raycast_box_min, raycast_box_max);
            update_hit = false;
        }

        // 记录缓存盒子边界
        raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
        raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);

        // 保存处理后的点用于后续射线遍历
        raycasting_cloud.push_back(p);

        // 如果是有效命中点，插入更新缓存
        if (update_hit) {
            posToGlobalIndex(p, pt_id_g);
            insertUpdateCandidate(pt_id_g, true);
        }
    }

    /// 步骤2: 遍历所有处理后点，执行射线投射（标记射线穿透区域为空闲）
    if(cfg_.raycasting_en) {
        for (const auto& p : raycasting_cloud) {
            // 从 raycast_range_min 开始遍历（避免机器人自身的遮挡）
            Vec3f raycast_start = (p - cur_odom).normalized() * cfg_.raycast_range_min + cur_odom;
            raycast_data_.raycaster.setInput(raycast_start, p);
            Vec3f ray_pt;
            while (raycast_data_.raycaster.step(ray_pt)) {
                Vec3i cur_ray_id_g;
                posToGlobalIndex(ray_pt, cur_ray_id_g);
                if (!insideLocalMap(cur_ray_id_g)) {
                    break;  // 出地图，停止遍历
                }
                insertUpdateCandidate(cur_ray_id_g, false);  // 标记为未命中（穿透 = 空闲）
            }
        }
    }

}

/// 将栅格插入更新候选队列
/// 使用操作计数进行去重：第一次命中时入队，后续只更新计数
/// @param id_g 栅格全局索引
/// @param is_hit true=命中（障碍物），false=未命中（穿透/空闲）
void ProbMap::insertUpdateCandidate(const Vec3i& id_g, bool is_hit) {
    const auto& hash_id = getHashIndexFromGlobalIndex(id_g);
    raycast_data_.operation_cnt[hash_id]++;

    // 第一次操作时入队（后续同一帧内对同一栅格的操作只更新计数）
    if (raycast_data_.operation_cnt[hash_id] == 1) {
        raycast_data_.update_cache_id_g.push(id_g);
    }

    if (is_hit) {
        raycast_data_.hit_cnt[hash_id]++;
    }
}

/// 更新局部更新盒子的位置
/// 局部更新盒子跟随机器人移动，但受限于局部地图边界
/// 用于限制射线投射的处理范围
void ProbMap::updateLocalBox(const Vec3f& cur_odom) {
    std::lock_guard<std::mutex> lck(raycast_data_.raycast_range_mtx);

    Vec3i cur_odom_i;
    posToGlobalIndex(cur_odom, cur_odom_i);
    Vec3i local_updatebox_min_i, local_updatebox_max_i;

    // 以机器人位置为中心，计算更新盒子的索引范围
    if (cfg_.raycasting_en) {
        local_updatebox_max_i = cur_odom_i + cfg_.half_local_update_box_i;
        local_updatebox_min_i = cur_odom_i - cfg_.half_local_update_box_i;
    }

    // 将索引转换为世界坐标
    globalIndexToPos(local_updatebox_min_i, raycast_data_.local_update_box_min);
    globalIndexToPos(local_updatebox_max_i, raycast_data_.local_update_box_max);

    // 限制在局部地图边界内
    raycast_data_.local_update_box_max = raycast_data_.local_update_box_max.cwiseMin(local_map_bound_max_d_);
    raycast_data_.local_update_box_min = raycast_data_.local_update_box_min.cwiseMax(local_map_bound_min_d_);
}

/// 重置整个概率地图
void ProbMap::resetLocalMap() {
    std::cout << RED << " -- [Prob-Map] Clear all local map." << RESET << std::endl;
    std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0);  // 清零占用概率
    // 清空更新缓存队列
    while (!raycast_data_.update_cache_id_g.empty()) {
        raycast_data_.update_cache_id_g.pop();
    }
    raycast_data_.batch_update_counter = 0;
    std::fill(raycast_data_.operation_cnt.begin(), raycast_data_.operation_cnt.end(), 0);
    std::fill(raycast_data_.hit_cnt.begin(), raycast_data_.hit_cnt.end(), 0);
}