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

#include <utils/common_lib.hpp>

namespace rog_map {
    using std::string;
    using std::vector;
/// 未知标志常量，用于初始化和重置
#define RM_UNKNOWN_FLAG (-99999)
    typedef pcl::PointXYZINormal PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;

    /// ROG-Map 配置管理类
    /// 负责从 ROS 参数服务器加载所有配置参数
    /// 并对参数进行合法性检查和预处理（如计算各类地图尺寸、初始化邻域查找表等）
    class Config {
    private:
        /// 从 ROS 参数服务器加载标量参数
        /// @tparam T 参数类型
        /// @param param_name 参数名（相对于命名空间）
        /// @param param_value 输出参数值
        /// @param default_value 加载失败时的默认值
        /// @param required 是否为必需参数（必需参数加载失败会抛出异常）
        /// @return 成功加载返回true，使用默认值返回false
        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value = T{}, bool required = false) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
                std::cout << param_value << std::endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                       (nh_.getNamespace() + "/" + param_name).c_str());
                param_value = default_value;
                std::cout << param_value << std::endl;
                if (required) {
                    throw std::invalid_argument(
                            string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
                }
                return false;
            }
        }

        /// 从 ROS 参数服务器加载数组参数（std::vector 类型）
        /// @tparam T 数组元素类型
        template<class T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value = vector<T>{},
                       bool required = false) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
                for (size_t i = 0; i < param_value.size(); i++) {
                    std::cout << param_value[i] << " ";
                }
                std::cout << std::endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                       (nh_.getNamespace() + "/" + param_name).c_str());
                param_value = default_value;
                for (size_t i = 0; i < param_value.size(); i++) {
                    std::cout << param_value[i] << " ";
                }
                std::cout << std::endl;
                if (required) {
                    throw std::invalid_argument(
                            string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
                }
                return false;
            }
        }

    public:
        Config(){};

        /// 构造函数：从 ROS 参数服务器加载所有配置
        /// @param nh ROS 节点句柄
        /// @param name_space 参数命名空间（默认为 "rog_map"）
        Config(const ros::NodeHandle &nh,
               const string &name_space = "rog_map") : nh_(nh) {
            std::cout<<" -- [ROG Config] Current namespace: "<< nh_.getNamespace()<<std::endl;

            // ===== ESDF（欧几里得符号距离场）相关参数 =====
            LoadParam(name_space + "/esdf/resolution", esdf_resolution, 0.2);
            LoadParam(name_space + "/esdf/enable", esdf_en, false);
            vector<double> temp_esdf_update_box;
            LoadParam(name_space + "/esdf/local_update_box", temp_esdf_update_box, temp_esdf_update_box);

            if (esdf_en) {
                if (temp_esdf_update_box.size() != 3) {
                    ROS_ERROR("Fix map origin size is not 3!");
                    exit(-1);
                } else {
                    esdf_local_update_box = Vec3f(temp_esdf_update_box[0], temp_esdf_update_box[1],
                                                  temp_esdf_update_box[2]);
                }
            }

            // ===== PCD 点云文件加载参数 =====
            LoadParam(name_space + "/load_pcd_en", load_pcd_en, false);
            if (load_pcd_en) {
                LoadParam(name_space + "/pcd_name", pcd_name, string("map.pcd"));
            }

            // ===== 地图滑动参数 =====
            LoadParam(name_space + "/map_sliding/enable", map_sliding_en, true);
            LoadParam(name_space + "/map_sliding/threshold", map_sliding_thresh, -1.0);

            // ===== 固定地图原点 =====
            vector<double> temp_fix_origin;
            LoadParam(name_space + "/fix_map_origin", temp_fix_origin, vector<double>{0, 0, 0});
            if (temp_fix_origin.size() != 3) {
                ROS_ERROR("Fix map origin size is not 3!");
                exit(-1);
            } else {
                fix_map_origin = Vec3f(temp_fix_origin[0], temp_fix_origin[1], temp_fix_origin[2]);
            }

            // ===== 前沿提取 =====
            LoadParam(name_space + "/frontier_extraction_en", frontier_extraction_en, false);

            // ===== ROS 回调参数 =====
            LoadParam(name_space + "/ros_callback/enable", ros_callback_en, false);
            LoadParam(name_space + "/ros_callback/cloud_topic", cloud_topic, string("/cloud_registered"));
            LoadParam(name_space + "/ros_callback/odom_topic", odom_topic, string("/lidar_slam/odom"));
            LoadParam(name_space + "/ros_callback/odom_timeout", odom_timeout, 0.05);

            // ===== 可视化参数 =====
            LoadParam(name_space + "/visualization/enable", visualization_en, false);
            LoadParam(name_space + "/visualization/use_dynamic_reconfigure", use_dynamic_reconfigure, false);
            LoadParam(name_space + "/visualization/pub_unknown_map_en", pub_unknown_map_en, false);
            LoadParam(name_space + "/visualization/frame_id", frame_id, string("world"));
            LoadParam(name_space + "/visualization/time_rate", viz_time_rate, 0.0);
            LoadParam(name_space + "/visualization/frame_rate", viz_frame_rate, 0);
            vector<double> temp_vis_range;
            LoadParam(name_space + "/visualization/range", temp_vis_range, vector<double>{0, 0, 0});
            if (temp_vis_range.size() != 3) {
                ROS_ERROR("Visualization range size is not 3!");
                exit(-1);
            } else {
                visualization_range = Vec3f(temp_vis_range[0], temp_vis_range[1], temp_vis_range[2]);
                // 如果可视化范围未设置（全为0），则禁用可视化
                if (visualization_range.minCoeff() <= 0) {
                    std::cout << YELLOW << " -- [ROG] Visualization range is not set, visualization is disabled"
                              << RESET << std::endl;
                    visualization_en = false;
                }
            }

            // ===== 分辨率参数 =====
            LoadParam(name_space + "/resolution", resolution, 0.1);
            LoadParam(name_space + "/inflation_resolution", inflation_resolution, 0.1);
            // 膨胀分辨率必须大于等于基础分辨率
            if (resolution > inflation_resolution) {
                ROS_ERROR("The inflation resolution should be equal or larger than the resolution!");
                exit(-1);
            }

            // ===== 未知区域膨胀参数 =====
            LoadParam(name_space + "/unk_inflation_en", unk_inflation_en, false);
            LoadParam(name_space + "/unk_inflation_step", unk_inflation_step, 1);

            // ===== 膨胀步长和强度阈值 =====
            LoadParam(name_space + "/inflation_step", inflation_step, 1);
            LoadParam(name_space + "/intensity_thresh", intensity_thresh, -1);

            // ===== 地图尺寸 =====
            vector<double> temp_map_size;
            LoadParam(name_space + "/map_size", temp_map_size, vector<double>{10, 10, 0});
            if (temp_map_size.size() != 3) {
                ROS_ERROR("Map size dimension is not 3!");
                exit(-1);
            }
            map_size_d = Vec3f(temp_map_size[0], temp_map_size[1], temp_map_size[2]);

            // ===== 点云降采样参数 =====
            LoadParam(name_space + "/point_filt_num", point_filt_num, 2);
            if (point_filt_num <= 0) {
                std::cout << RED << " -- [ROG] point_filt_num should be larger or equal than 1, it is set to 1 now."
                          << RESET << std::endl;
                point_filt_num = 1;
            }

            // ===== 射线投射参数 =====
            LoadParam(name_space + "/raycasting/enable", raycasting_en, true);
            LoadParam(name_space + "/raycasting/batch_update_size", batch_update_size, 1);
            if (batch_update_size <= 0) {
                std::cout << RED << " -- [ROG] batch_update_size should be larger or equal than 1, it is set to 1 now."
                          << RESET << std::endl;
                batch_update_size = 1;
            }

            // ===== 概率更新参数 =====
            LoadParam(name_space + "/raycasting/unk_thresh", unk_thresh, 0.70);
            LoadParam(name_space + "/raycasting/p_hit", p_hit, 0.70f);
            LoadParam(name_space + "/raycasting/p_miss", p_miss, 0.70f);
            LoadParam(name_space + "/raycasting/p_min", p_min, 0.12f);
            LoadParam(name_space + "/raycasting/p_max", p_max, 0.97f);
            LoadParam(name_space + "/raycasting/p_occ", p_occ, 0.80f);
            LoadParam(name_space + "/raycasting/p_free", p_free, 0.30f);

            // ===== 射线范围 =====
            vector<double> temp_ray_range;
            LoadParam(name_space + "/raycasting/ray_range", temp_ray_range, vector<double>{0.3, 10});
            if (temp_ray_range.size() != 2) {
                ROS_ERROR("Ray range size is not 2!");
                exit(-1);
            }
            raycast_range_min = temp_ray_range[0];
            raycast_range_max = temp_ray_range[1];
            sqr_raycast_range_max = raycast_range_max * raycast_range_max;  // 预计算平方值用于距离比较
            sqr_raycast_range_min = raycast_range_min * raycast_range_min;

            // ===== 局部更新盒子 =====
            vector<double> update_box;
            LoadParam(name_space + "/raycasting/local_update_box", update_box, vector<double>{999, 999, 999});
            if (update_box.size() != 3) {
                ROS_ERROR("Update box size is not 3!");
                exit(-1);
            }
            local_update_box_d = Vec3f(update_box[0], update_box[1], update_box[2]);

            // ===== 虚拟天花板和地板高度 =====
            // （高于天花板或低于地板的区域被视为占用）
            LoadParam(name_space + "/virtual_ground_height", virtual_ground_height, -0.1);
            LoadParam(name_space + "/virtual_ceil_height", virtual_ceil_height, -0.1);

            // 根据参数重新计算地图尺寸
            resetMapSize();

            // ===== 概率更新：将对数几率(log-odds)预计算为查找表 =====
            // logit(p) = log(p / (1-p))，将概率转换为对数几率
#define logit(x) (log((x) / (1 - (x))))
            l_hit = logit(p_hit);    // 命中时的对数几率增量
            l_miss = logit(p_miss);   // 未命中时的对数几率减量
            l_min = logit(p_min);     // 对数几率下限
            l_max = logit(p_max);     // 对数几率上限
            l_occ = logit(p_occ);     // 判定为占用的对数几率阈值
            l_free = logit(p_free);   // 判定为空闲的对数几率阈值

            // 计算从空闲/未知变为占用或空闲所需的最小命中/未命中次数
            int n_free = ceil(l_free / l_miss);
            int n_occ = ceil(l_occ / l_hit);

            // 输出概率更新参数
            std::cout << BLUE << "\t[ROG] n_free: " << n_free << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] n_occ: " << n_occ << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_hit: " << l_hit << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_miss: " << l_miss << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_min: " << l_min << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_max: " << l_max << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_occ: " << l_occ << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_free: " << l_free << RESET << std::endl;

            // ===== 初始化球面邻域查找表 =====
            // 用于膨胀操作：查找膨胀步长范围内的所有邻居栅格偏移
            for (int dx = -inflation_step; dx <= inflation_step; dx++) {
                for (int dy = -inflation_step; dy <= inflation_step; dy++) {
                    for (int dz = -inflation_step; dz <= inflation_step; dz++) {
                        // 如果步长为1则全部包含；否则只包含球面（欧氏距离≤步长）范围内的
                        if (inflation_step == 1 ||
                            dx * dx + dy * dy + dz * dz <= inflation_step * inflation_step) {
                            spherical_neighbor.emplace_back(dx, dy, dz);
                        }
                    }
                }
            }
            // 按距离排序（近到远），便于优化
            std::sort(spherical_neighbor.begin(), spherical_neighbor.end(), [](const Vec3i &a, const Vec3i &b) {
                return a.norm() < b.norm();
            });

            // ===== 初始化未知区域膨胀邻域查找表 =====
            if (unk_inflation_en) {
                unk_spherical_neighbor.clear();
                for (int dx = -unk_inflation_step; dx <= unk_inflation_step; dx++) {
                    for (int dy = -unk_inflation_step; dy <= unk_inflation_step; dy++) {
                        for (int dz = -unk_inflation_step; dz <= unk_inflation_step; dz++) {
                            if (unk_inflation_step == 1 ||
                                dx * dx + dy * dy + dz * dz <= unk_inflation_step * unk_inflation_step) {
                                unk_spherical_neighbor.emplace_back(dx, dy, dz);
                            }
                        }
                    }
                }
                std::sort(unk_spherical_neighbor.begin(), unk_spherical_neighbor.end(),
                          [](const Vec3i &a, const Vec3i &b) {
                              return a.norm() < b.norm();
                          });
            }
        }

        // ===== ESDF 配置 =====
        bool esdf_en{false};
        Vec3f esdf_local_update_box;          ///< ESDF的局部更新范围
        double esdf_resolution;               ///< ESDF地图的分辨率

        // ===== PCD 加载配置 =====
        bool load_pcd_en{false};
        bool use_dynamic_reconfigure{false};
        string pcd_name{"map.pcd"};

        // ===== 基础和膨胀地图的核心尺寸参数 =====
        double resolution, inflation_resolution;
        int inflation_step;                   ///< 膨胀球体半径（以膨胀地图栅格为单位）
        Vec3f local_update_box_d, half_local_update_box_d;
        Vec3i local_update_box_i, half_local_update_box_i;
        Vec3f map_size_d, half_map_size_d;    ///< 地图的物理尺寸和半尺寸（米）
        Vec3i inf_half_map_size_i, half_map_size_i, fro_half_map_size_i;
        double virtual_ceil_height, virtual_ground_height;
        int virtual_ceil_height_id_g, virtual_ground_height_id_g;

        // ===== 功能开关 =====
        bool visualization_en{false}, frontier_extraction_en{false},
                raycasting_en{true}, ros_callback_en{false}, pub_unknown_map_en{false};

        // ===== 邻域查找表 =====
        std::vector<Vec3i> spherical_neighbor;       ///< 占用膨胀球面邻域偏移列表
        std::vector<Vec3i> unk_spherical_neighbor;   ///< 未知膨胀球面邻域偏移列表

        // ===== 其他配置 =====
        int intensity_thresh;                 ///< 点云强度过滤阈值（-1表示不过滤）
        string frame_id;                      ///< TF 坐标系ID
        bool map_sliding_en{true};            ///< 是否启用地图滑动
        Vec3f fix_map_origin;                 ///< 固定地图原点（map_sliding_en=false时使用）
        string odom_topic, cloud_topic;       ///< ROS话题名称

        // ===== 概率更新参数 =====
        double raycast_range_min, raycast_range_max;        ///< 射线投射的最小/最大距离
        double sqr_raycast_range_min, sqr_raycast_range_max; ///< 距离平方（预计算用于快速比较）
        int point_filt_num, batch_update_size;              ///< 点云降采样率和批处理大小
        float p_hit, p_miss, p_min, p_max, p_occ, p_free;  ///< 概率值
        float l_hit, l_miss, l_min, l_max, l_occ, l_free;  ///< 对数几率（预计算）

        // ===== 未知膨胀配置 =====
        bool unk_inflation_en{false};
        int unk_inflation_step{0};

        // ===== 其他 =====
        double odom_timeout;                  ///< 里程计超时时间（秒）
        Vec3f visualization_range;            ///< 可视化范围
        double viz_time_rate;                 ///< 可视化发布时间频率（Hz）
        int viz_frame_rate;                   ///< 可视化发布帧率
        double unk_thresh;                    ///< 未知判定阈值
        double map_sliding_thresh;            ///< 地图滑动触发阈值

        /// 重新计算地图尺寸
        /// 根据分辨率和膨胀参数计算各级地图的尺寸（double 和 int 表示）
        /// 确保:
        /// 1) 膨胀图尺寸 ≥ 基础概率图尺寸（包含膨胀边界）
        /// 2) 前沿计数图尺寸 ≥ 概率图尺寸
        /// 3) 所有尺寸为奇数（ORIGIN_AT_CENTER模式下）
        void resetMapSize() {

            // 计算膨胀分辨率与基础分辨率的比率
            int inflation_ratio = ceil(inflation_resolution / resolution);

#ifdef ORIGIN_AT_CENTER
            // 当原点在栅格中心时，膨胀比率必须为奇数
            // 以确保栅格可以对齐
    if (inflation_ratio % 2 == 0) {
        inflation_ratio += 1;
    }
    std::cout << RED << " -- [RM-Config] inflation_ratio: " << inflation_ratio << std::endl;
#endif

            // 基于取整后的比率重新计算膨胀分辨率
            inflation_resolution = resolution * inflation_ratio;
            std::cout << GREEN << " -- [RM-Config] inflation_resolution: " << inflation_resolution << std::endl;

            half_map_size_d = map_size_d / 2;

            // Size_d 仅用于计算索引数量
            // 1) 计算膨胀地图的索引数量
            //    膨胀地图大小 = 基础地图大小 + 膨胀步长边界
            int max_step = 0;
            if (!unk_inflation_en) {
                max_step = inflation_step;
            } else {
                max_step = std::max(inflation_step, unk_inflation_step);
            }
            inf_half_map_size_i = (half_map_size_d / inflation_resolution).cast<int>()
                                  + (max_step + 1) * Vec3i::Ones();

            // 2) 计算概率地图的索引数量（应小于膨胀地图）
            half_map_size_i = (inf_half_map_size_i - (max_step + 1) * Vec3i::Ones()) * inflation_ratio;

            // 3) 计算前沿计数地图的尺寸
            if (frontier_extraction_en) {
                fro_half_map_size_i = half_map_size_i + Vec3i::Constant(1);
            }

            // 4) 根据索引数量重新计算地图物理尺寸
            map_size_d = (half_map_size_i * 2 + Vec3i::Constant(1)).cast<double>() * resolution;
            half_map_size_d = map_size_d / 2;

            // 5) 计算射线投射更新盒子的索引尺寸
            half_local_update_box_d = local_update_box_d / 2;
            half_local_update_box_i = (half_local_update_box_d / resolution).cast<int>();
            local_update_box_i = half_local_update_box_i * 2 + Vec3i::Constant(1);
            local_update_box_d = local_update_box_i.cast<double>() * resolution;
        }


        ros::NodeHandle nh_;

    };

}
