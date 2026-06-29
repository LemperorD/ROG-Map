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

#include "rog_map/rog_map.h"

using namespace rog_map;

/// ROGMap 构造函数
/// 初始化所有子系统：
/// 1) 从ROS参数服务器加载配置
/// 2) 初始化概率地图和子地图（膨胀、ESDF、前沿）
/// 3) 设置可视化发布器
/// 4) 如果启用ROS回调，订阅里程计和点云话题
/// 5) 如果启用了PCD加载，从文件加载预生成地图
ROGMap::ROGMap(const ros::NodeHandle& nh) :nh_(nh) {

    // 加载配置并初始化概率地图
    cfg_ = rog_map::Config(nh);
    initProbMap();

    // 打开日志文件
    map_info_log_file_.open(DEBUG_FILE_DIR("rm_info_log.csv"), std::ios::out | std::ios::trunc);
    time_log_file_.open(DEBUG_FILE_DIR("rm_performance_log.csv"), std::ios::out | std::ios::trunc);

    // 设置可视化默认配置
    vm_.vizcfg.use_body_center = true;
    vm_.vizcfg.box_min = -cfg_.visualization_range / 2;
    vm_.vizcfg.box_max = cfg_.visualization_range / 2;

    // 绑定动态重配置回调
    vm_.vizcfg.callback_func = boost::bind(&ROGMap::VizCfgCallback, this, _1, _2);
    vm_.vizcfg.vizcfgserver.setCallback(vm_.vizcfg.callback_func);

    // 设置机器人初始位置为固定地图原点
    robot_state_.p = cfg_.fix_map_origin;

    // 地图初始化：滑动或固定模式
    if (cfg_.map_sliding_en) {
        // 滑动模式：将地图滑动到原点(0,0,0)
        mapSliding(Vec3f(0, 0, 0));
        inf_map_->mapSliding(Vec3f(0, 0, 0));
    }
    else {
        // 固定模式：地图固定在 fix_map_origin
        local_map_bound_min_d_ = -cfg_.half_map_size_d + cfg_.fix_map_origin;
        local_map_bound_max_d_ = cfg_.half_map_size_d + cfg_.fix_map_origin;
        mapSliding(cfg_.fix_map_origin);
        inf_map_->mapSliding(cfg_.fix_map_origin);
        vm_.vizcfg.box_min += cfg_.fix_map_origin;
        vm_.vizcfg.box_max += cfg_.fix_map_origin;
    }

    /// 初始化可视化发布器
    if (cfg_.visualization_en) {
        // 基础地图可视化
        vm_.occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/occ", 1);
        vm_.unknown_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/unk", 1);
        vm_.occ_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/inf_occ", 1);
        vm_.unknown_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/inf_unk", 1);

        // 前沿可视化
        if (cfg_.frontier_extraction_en) {
            vm_.frontier_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/frontier", 1);
        }

        // ESDF可视化
        if (cfg_.esdf_en) {
            vm_.esdf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf", 1);
            vm_.esdf_neg_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf/neg", 1);
            vm_.esdf_occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf/occ", 1);
        }

        // 设置可视化定时器（按指定频率发布）
        if (cfg_.viz_time_rate > 0) {
            vm_.viz_timer = nh_.createTimer(ros::Duration(1.0 / cfg_.viz_time_rate), &ROGMap::vizCallback, this);
        }
    }
    // 地图边界MarkerArray发布器（始终创建）
    vm_.mkr_arr_pub = nh_.advertise<visualization_msgs::MarkerArray>("rog_map/map_bound", 1);

    // 如果启用ROS回调模式，订阅话题
    if (cfg_.ros_callback_en) {
        rc_.odom_sub = nh_.subscribe(cfg_.odom_topic, 1, &ROGMap::odomCallback, this);
        rc_.cloud_sub = nh_.subscribe(cfg_.cloud_topic, 1, &ROGMap::cloudCallback, this);
        // 高频更新定时器（1kHz），用于尽快处理缓存的数据
        rc_.update_timer = nh_.createTimer(ros::Duration(0.001), &ROGMap::updateCallback, this);
    }

    // 输出地图信息到日志
    writeMapInfoToLog(map_info_log_file_);
    map_info_log_file_.close();

    // 输出性能日志的列标题
    for (int i = 0; i < time_consuming_name_.size(); i++) {
        time_log_file_ << time_consuming_name_[i];
        if (i != time_consuming_name_.size() - 1) {
            time_log_file_ << ", ";
        }
    }
    time_log_file_ << endl;

    // 如果启用了PCD加载，从文件加载预生成地图
    if (cfg_.load_pcd_en) {
        string pcd_path = cfg_.pcd_name;
        PointCloud::Ptr pcd_map(new PointCloud);
        if (pcl::io::loadPCDFile(pcd_path, *pcd_map) == -1) {
            cout << RED << "Load pcd file failed!" << RESET << endl;
            exit(-1);
        }
        Pose cur_pose;
        cur_pose.first = Vec3f(0, 0, 0);
        updateOccPointCloud(*pcd_map);             // 直接标记占用（无射线投射）
        esdf_map_->updateESDF3D(robot_state_.p);   // 计算初始ESDF
        cout << BLUE << " -- [ROGMap]Load pcd file success with " << pcd_map->size() << " pts." << RESET << endl;
        map_empty_ = false;
    }
}

// ===== 碰撞检测 API（三种重载形式） =====

/// 碰撞检测（基本版本）：检查线段是否无障碍
/// 使用DDA射线投射沿线段遍历，检查每个栅格的状态
///
/// @param use_inf_map true=在膨胀地图上检查（更保守），false=在概率地图上检查
/// @param use_unk_as_occ true=未知区域视为占用（保守），false=未知区域允许通过（激进，适用于探索）
bool ROGMap::isLineFree(const rog_map::Vec3f& start_pt, const rog_map::Vec3f& end_pt,
                        const bool& use_inf_map, const bool& use_unk_as_occ) const {
    // NaN检查
    if(start_pt.array().isNaN().any() || end_pt.array().isNaN().any()) {
        cout<<RED<<" -- [ROGMap] Call isLineFree with NaN in start or end pt, return false."<<RESET<<endl;
        return false;
    }

    raycaster::RayCaster raycaster;
    if (use_inf_map) {
        raycaster.setResolution(cfg_.inflation_resolution);  // 使用膨胀地图分辨率
    } else {
        raycaster.setResolution(cfg_.resolution);            // 使用基础分辨率
    }

    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    while (raycaster.step(ray_pt)) {
        if (!use_unk_as_occ) {
            // 策略A: 只排斥占用（未知和已知空闲都允许通过）
            if (use_inf_map) {
                if (isOccupiedInflate(ray_pt)) return false;
            } else {
                if (isOccupied(ray_pt)) return false;
            }
        } else {
            // 策略B: 只允许已知空闲（占用和未知都不允许通过）
            if (use_inf_map) {
                if ((isUnknownInflate(ray_pt) || isOccupiedInflate(ray_pt)))
                    return false;
            } else {
                if (!isKnownFree(ray_pt)) return false;
            }
        }
    }
    return true;  // 全程无障碍
}

/// 碰撞检测（带邻居列表版本）：考虑机器人尺寸
/// @param max_dis 最大检查距离
/// @param neighbor_list 额外的邻居偏移（例如机器人半径对应的偏移列表）
bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    while (raycaster.step(ray_pt)) {
        // 距离限制检查
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
            return false;
        }

        if (neighbor_list.empty()) {
            // 无邻居列表：只检查该点本身
            if (isOccupied(ray_pt)) return false;
        } else {
            // 有邻居列表：检查该点及其所有偏移邻居
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto& nei : neighbor_list) {
                Vec3i shift_tmp = ray_pt_id_g + nei;
                if (isOccupied(shift_tmp)) return false;
            }
        }
    }
    return true;
}

/// 碰撞检测（带返回值版本）：返回路径上最后一个空闲点
/// 用于部分路径规划：如果线段不通，返回最远可通行的位置
bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, Vec3f& free_local_goal, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    free_local_goal = start_pt;  // 初始化为起点
    while (raycaster.step(ray_pt)) {
        free_local_goal = ray_pt;  // 持续更新最后一个空闲点
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
            return false;
        }

        if (neighbor_list.empty()) {
            if (isOccupied(ray_pt)) return false;
        } else {
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto& nei : neighbor_list) {
                Vec3i shift_tmp = ray_pt_id_g + nei;
                if (isOccupied(shift_tmp)) return false;
            }
        }
    }
    free_local_goal = end_pt;  // 全程通畅，终点可达
    return true;
}

/// 手动更新地图 API
/// 仅在 ros_callback_en = false 时可用
void ROGMap::updateMap(const PointCloud& cloud, const Pose& pose) {
    TimeConsuming ssss("sss", false);
    if (cfg_.ros_callback_en) {
        std::cout << RED << "ROS callback is enabled, can not insert map from updateMap API." << RESET << std::endl;
        return;
    }

    if (cloud.empty()) {
        static int local_cnt = 0;
        if (local_cnt++ > 100) {
            cout << YELLOW << "No cloud input, please check the input topic." << RESET << endl;
            local_cnt = 0;
        }
        return;
    }

    updateRobotState(pose);
    updateProbMap(cloud, pose);
    writeTimeConsumingToLog(time_log_file_);
}

RobotState ROGMap::getRobotState() const {
    return robot_state_;
}

/// 更新机器人状态
/// 从里程计/跟踪数据中提取位置、姿态、偏航角，
/// 并更新局部地图边界
void ROGMap::updateRobotState(const Pose& pose) {
    robot_state_.p = pose.first;                      // 位置
    robot_state_.q = pose.second;                     // 姿态四元数
    robot_state_.rcv_time = ros::Time::now().toSec(); // 接收时间戳
    robot_state_.rcv = true;                          // 标记已接收数据
    robot_state_.yaw = get_yaw_from_quaternion<double>(pose.second);  // 提取偏航角
    updateLocalBox(pose.first);                       // 更新局部地图边界
}

/// 里程计回调
/// 1) 更新机器人状态
/// 2) 广播 world → drone 的TF变换
void ROGMap::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
    // 从ROS消息提取位置和姿态
    updateRobotState(std::make_pair(
        Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
              odom_msg->pose.pose.position.z),
        Quatf(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
              odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)));

    // 广播 world → drone TF变换（用于RViz中可视化机器人模型）
    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w;
    br_map_ego.sendTransform(transformStamped);
}

/// 点云回调
/// 缓存最新的点云数据，加上对应的机器人位姿
/// 如果里程计超时，跳过此帧（避免使用过时位姿）
void ROGMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (!robot_state_.rcv) {
        return;  // 尚未收到里程计，跳过
    }

    // 检查里程计超时：点云时间不能比里程计时间晚太久
    double cbk_t = ros::Time::now().toSec();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
        return;
    }

    // 将ROS PointCloud2消息转换为PCL点云
    PointCloud temp_pc;
    pcl::fromROSMsg(*cloud_msg, temp_pc);

    // 在锁保护下缓存数据
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;  // 增加未处理帧计数
    map_empty_ = false;
    rc_.updete_lock.unlock();
}

/// 更新回调（高频触发，1kHz）
/// 检查是否有新的点云数据需要处理，
/// 如果有，调用 updateProbMap 更新地图
void ROGMap::updateCallback(const ros::TimerEvent& event) {
    // 如果地图为空，输出警告
    if (map_empty_) {
        static double last_print_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
            std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET << std::endl;
            last_print_t = cur_t;
        }
        return;
    }

    // 没有未处理的帧
    if (rc_.unfinished_frame_cnt == 0) {
        return;
    } else if (rc_.unfinished_frame_cnt > 1) {
        // 多帧积压 → 处理跟不上输入，输出警告
        std::cout << RED <<
            " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET << std::endl;
    }

    // 在锁保护下获取缓存数据并重置计数器
    static PointCloud temp_pc;
    static Pose temp_pose;
    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;  // 清零未处理计数
    rc_.updete_lock.unlock();

    // 更新概率地图
    updateProbMap(temp_pc, temp_pose);

    // 输出耗时日志
    writeTimeConsumingToLog(time_log_file_);
}

/// 向量列表 → ROS PointCloud2 消息
/// 用于将从地图查询得到的点列表转换为可发布的ROS消息格式
void ROGMap::vecEVec3fToPC2(const vec_E<Vec3f>& points, sensor_msgs::PointCloud2& cloud) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
        pcl_cloud[i].x = static_cast<float>(points[i][0]);
        pcl_cloud[i].y = static_cast<float>(points[i][1]);
        pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "world";
}

/// 可视化回调
/// 发布所有可视化数据：
/// - 占用和未知栅格点云
/// - 膨胀占用和膨胀未知栅格点云
/// - 前沿点云
/// - ESDF（正距离 / 负距离）
/// - 可视化范围、局部地图范围、更新范围等边界框
void ROGMap::vizCallback(const ros::TimerEvent& event) {
    TimeConsuming ssss("vizCallback", false);

    if (!cfg_.visualization_en || map_empty_) {
        return;
    }

    Vec3f box_min, box_max;

    // 确定可视化范围
    if (cfg_.use_dynamic_reconfigure) {
        // 使用动态重配置设定的范围
        box_min = vm_.vizcfg.box_min;
        box_max = vm_.vizcfg.box_max;
        if (vm_.vizcfg.use_body_center) {
            box_min += robot_state_.p;
            box_max += robot_state_.p;
        }
    } else {
        // 使用配置文件设定的范围（以机器人为中心）
        box_max = robot_state_.p + cfg_.visualization_range / 2;
        box_min = robot_state_.p - cfg_.visualization_range / 2;
    }

    boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0) {
        cout << RED << " -- [ROGMap] Visualization range is too small." << RESET << endl;
        return;
    }

    // ===== 发布未知地图 =====
    if (cfg_.pub_unknown_map_en && vm_.unknown_pub.getNumSubscribers() >= 1) {
        vec_E<Vec3f> unknown_map, inf_unknown_map;
        boxSearch(box_min, box_max, UNKNOWN, unknown_map);
        sensor_msgs::PointCloud2 cloud_msg;
        vecEVec3fToPC2(unknown_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.unknown_pub.publish(cloud_msg);

        // 膨胀未知地图（如果启用）
        if (cfg_.unk_inflation_en && vm_.unknown_inf_pub.getNumSubscribers() >= 1) {
            boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
            vecEVec3fToPC2(inf_unknown_map, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            vm_.unknown_inf_pub.publish(cloud_msg);
        }
    }

    // ===== 发布前沿地图 =====
    if (cfg_.frontier_extraction_en && vm_.frontier_pub.getNumSubscribers() >= 1) {
        vec_E<Vec3f> frontier_map;
        boxSearch(box_min, box_max, FRONTIER, frontier_map);
        sensor_msgs::PointCloud2 cloud_msg;
        vecEVec3fToPC2(frontier_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.frontier_pub.publish(cloud_msg);
    }

    // ===== 发布占用地图 =====
    vec_E<Vec3f> occ_map, inf_occ_map;
    sensor_msgs::PointCloud2 cloud_msg;
    if (vm_.occ_pub.getNumSubscribers() >= 1) {
        boxSearch(box_min, box_max, OCCUPIED, occ_map);
        vecEVec3fToPC2(occ_map, cloud_msg);
        vm_.occ_pub.publish(cloud_msg);
    }

    if (vm_.occ_inf_pub.getNumSubscribers() >= 1) {
        boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
        vecEVec3fToPC2(inf_occ_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.occ_inf_pub.publish(cloud_msg);
    }

    // ===== 发布ESDF地图 =====
    if (cfg_.esdf_en) {
        // 正距离开销（障碍物外部）
        if (vm_.esdf_pub.getNumSubscribers() >= 1) {
            esdf_map_->getPositiveESDFPC2(box_min, box_max, robot_state_.p.z() - 0.5, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            vm_.esdf_pub.publish(cloud_msg);
        }

        // 负距离开销（障碍物内部）
        if (vm_.esdf_neg_pub.getNumSubscribers() >= 1) {
            esdf_map_->getNegativeESDFPC2(box_min, box_max, robot_state_.p.z() - 0.5, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            vm_.esdf_neg_pub.publish(cloud_msg);
        }

#ifdef ESDF_MAP_DEBUG
        // 调试模式：发布ESDF中的占用栅格
        esdf_map_->getESDFOccPC2(box_min, box_max, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.esdf_occ_pub.publish(cloud_msg);
#endif
    }

    // ===== 发布可视化边界框（MarkerArray） =====
    vm_.mkr_arr.markers.clear();

    // 可视化范围（紫色）
    visualizeBoundingBox(vm_.mkr_arr, box_min, box_max, "Visualization Range", Color::Purple());
    visualizeText(vm_.mkr_arr, "Visualization Range Text", "Visualization Range",
                  box_max + Vec3f(0, 0, 0.5), Color::Purple(), 0.6, 0);

    // 局部地图范围（橙色）
    Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
    boundBoxByLocalMap(local_map_min, local_map_max);
    visualizeBoundingBox(vm_.mkr_arr, local_map_min, local_map_max, "Local Map Range", Color::Orange());
    visualizeText(vm_.mkr_arr, "Local Map Range Text", "Local Map Range",
                  local_map_max + Vec3f(0, 0, 1.0), Color::Orange(), 0.6, 0);

    // 射线投射更新范围（绿色）
    visualizeBoundingBox(vm_.mkr_arr, raycast_data_.cache_box_min, raycast_data_.cache_box_max,
                         "Updating Range", Color::Green());
    visualizeText(vm_.mkr_arr, "Updating Range Text", "Updating Range",
                  raycast_data_.cache_box_max + Vec3f(0, 0, 0.5), Color::Green(), 0.6, 0);

    // 局部地图原点（红色点）
    visualizePoint(vm_.mkr_arr, local_map_origin_d_, Color::Red(), "Local Map Origin", 0.2, 0);

    // ESDF更新范围（蓝色）
    if (cfg_.esdf_en) {
        Vec3f esdf_box_max, esdf_box_min;
        esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
        visualizeText(vm_.mkr_arr, "ESDF Map Text", "ESDF Map", esdf_box_max + Vec3f(0, 0, 1.0),
                      Color::Blue(), 0.6, 0);
        visualizeBoundingBox(vm_.mkr_arr, esdf_box_min, esdf_box_max, "ESDF Updating Range", Color::Blue());
    }

    vm_.mkr_arr_pub.publish(vm_.mkr_arr);
}

/// 动态重配置回调
/// 从 dynamic_reconfigure GUI 接收参数更新
void ROGMap::VizCfgCallback(rog_map::VizConfig& config, uint32_t level) {
    vm_.vizcfg.use_body_center = config.use_body_center;
    vm_.vizcfg.box_min.x() = config.x_lower_bound;
    vm_.vizcfg.box_min.y() = config.y_lower_bound;
    vm_.vizcfg.box_min.z() = config.z_lower_bound;
    vm_.vizcfg.box_max.x() = config.x_upper_bound;
    vm_.vizcfg.box_max.y() = config.y_upper_bound;
    vm_.vizcfg.box_max.z() = config.z_upper_bound;
}