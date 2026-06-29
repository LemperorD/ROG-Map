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

#include <rog_map/prob_map.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <utils/common_lib.hpp>
#include <dynamic_reconfigure/server.h>
#include <rog_map/VizConfig.h>
#include <utils/visual_utils.hpp>

namespace rog_map {
    using namespace std;

    /// 点云类型别名：带强度和法向量的XYZ点
    typedef pcl::PointXYZINormal PointType;
    typedef pcl::PointCloud<PointType> PointCloudXYZIN;

    /// ROG-Map 顶层公开API类
    /// 继承自 ProbMap，整合了ROS回调接口、可视化功能和公开查询API
    ///
    /// 主要功能：
    /// - 自动ROS回调模式：订阅里程计和点云话题，自动更新地图
    /// - 手动更新模式：通过 updateMap() 直接插入点云和位姿
    /// - 碰撞检测：isLineFree() 提供基于射线的无障碍检测
    /// - RViz可视化：发布占用图、ESDF、前沿等可视化数据
    /// - 动态重配置：通过 dynamic_reconfigure 在运行时调整可视化参数
    class ROGMap : public ProbMap {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef shared_ptr<ROGMap> Ptr;

        /// 构造函数：从ROS NodeHandle初始化
        /// @param nh ROS节点句柄（用于参数加载和话题订阅/发布）
        ROGMap(const ros::NodeHandle &nh);

        ~ROGMap() = default;

        /// 碰撞检测：检查线段是否无碰撞（世界坐标）
        /// @param start_pt 线段起点
        /// @param end_pt 线段终点
        /// @param max_dis 最大检查距离（超过则返回false），默认无限制
        /// @param neighbor_list 额外的邻居偏移列表（用于考虑机器人半径）
        /// @return 线段全程无障碍返回true
        bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                        const double &max_dis = 999999,
                        const vec_Vec3i &neighbor_list = vec_Vec3i{}) const;

        /// 碰撞检测：检查线段是否无碰撞（同时返回最后的空闲点）
        /// @param[out] free_local_goal 输出：线段上最后一个空闲点的坐标
        bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                        Vec3f &free_local_goal, const double &max_dis = 999999,
                        const vec_Vec3i &neighbor_list = vec_Vec3i{}) const;

        /// 碰撞检测：在膨胀地图或基础地图中检查（世界坐标）
        /// @param use_inf_map true=使用膨胀地图，false=使用基础概率地图
        /// @param use_unk_as_occ true=将未知区域视为占用（保守），false=未知区域允许通过（激进）
        bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                        const bool & use_inf_map = false,
                        const bool & use_unk_as_occ = false) const;

        /// 手动更新地图（不使用ROS回调时调用）
        /// @param cloud 输入点云
        /// @param pose 机器人当前位姿（位置 + 四元数）
        void updateMap(const PointCloud &cloud, const Pose &pose);

        /// 获取机器人当前状态
        RobotState getRobotState() const;

    private:
        ros::NodeHandle nh_;

        RobotState robot_state_;                      ///< 机器人当前状态

        /// ROS回调相关数据结构
        struct ROSCallback {
            ros::Subscriber odom_sub, cloud_sub;       ///< 里程计和点云话题订阅器
            int unfinished_frame_cnt{0};               ///< 未完成处理的帧计数（>1说明处理跟不上输入）
            Pose pc_pose;                              ///< 缓存的最新点云对应的位姿
            PointCloud pc;                             ///< 缓存的最新点云
            ros::Timer update_timer;                   ///< 更新定时器（高频触发）
            mutex updete_lock;                         ///< 数据交换互斥锁（注意：原代码拼写为updete）
        } rc_;

        /// 可视化相关数据结构
        struct VisualizeMap {
            ros::Publisher occ_pub, unknown_pub,       ///< 占用和未知栅格发布器
                    occ_inf_pub, unknown_inf_pub,      ///< 膨胀占用和膨胀未知栅格发布器
                    mkr_arr_pub, frontier_pub,         ///< MarkerArray和前沿发布器
                    esdf_pub, esdf_neg_pub, esdf_occ_pub;  ///< ESDF（正距离、负距离、占用）发布器
            visualization_msgs::MarkerArray mkr_arr;    ///< 累积的MarkerArray
            ros::Timer viz_timer;                       ///< 可视化定时器
            /// 可视化配置（通过dynamic_reconfigure动态调整）
            struct VizCfg {
                dynamic_reconfigure::Server<rog_map::VizConfig> vizcfgserver;
                dynamic_reconfigure::Server<rog_map::VizConfig>::CallbackType callback_func;
                bool use_body_center{false};            ///< 是否以机器人身体为中心
                Vec3f box_min, box_max;                 ///< 可视化包围盒的边界
            } vizcfg;
        } vm_;

        std::ofstream time_log_file_, map_info_log_file_;  ///< 性能和地图信息日志文件

        /// 更新机器人状态（位置、姿态、时间戳）
        void updateRobotState(const Pose &pose);

        /// 里程计回调：接收并缓存里程计数据，广播TF变换
        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

        /// 点云回调：接收并缓存点云数据
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

        /// 更新回调：检测并处理缓存的点云+位姿数据
        void updateCallback(const ros::TimerEvent &event);

        /// 将 Vec3f 向量列表转换为 sensor_msgs::PointCloud2
        static void vecEVec3fToPC2(const vec_E<Vec3f> &points, sensor_msgs::PointCloud2 &cloud);

        /// 可视化回调：发布所有可视化数据到RViz
        void vizCallback(const ros::TimerEvent &event);

        /// 动态重配置回调：更新可视化参数
        void VizCfgCallback(rog_map::VizConfig &config, uint32_t level);

    };
}
