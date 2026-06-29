/**
 * ROG-Map 示例：A* 路径搜索节点
 *
 * 功能：使用 ROG-Map 提供的地图进行 A* 路径搜索，
 * 通过 RViz 的 "3D Nav Goal" 工具交互式设置起点和终点。
 *
 * 使用方式：
 * 1. roslaunch rog_map_example astar_example.launch
 * 2. 在 RViz 中按 G 键启用 3D Nav Goal
 * 3. 点击两个点：第一个为起点（橙色），第二个为终点（绿色）
 * 4. 每次点击两个点后自动执行路径搜索
 */

#include "rog_astar/rog_astar.hpp"


void rvizClickCallback(const geometry_msgs::PoseStampedConstPtr& msg);

rog_astar::AStar::Ptr rog_astar_ptr;
ros::Publisher mkr_pub;


/// 发布带文本标签的点标记到 RViz
/// @param p 点坐标
/// @param text 标签文本
/// @param c 颜色
void publishPointWithText(const rog_map::Vec3f& p,
                          const std::string& text,
                          const rog_map::Color c = rog_map::Color::Green());

int main(int argc, char** argv) {
    /// 初始化ROS节点
    ros::init(argc, argv, "rm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    /// 【1】创建 ROGMap 指针（地图模块）
    rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(nh);

    /// 【2】创建 A* 路径搜索模块，传入地图指针
    rog_astar_ptr = std::make_shared<rog_astar::AStar>(nh, rog_map_ptr);

    /// 【3】创建交互节点：订阅RViz点击 + 发布MarkerArray
    ros::Subscriber rviz_click_sub = nh.subscribe("/goal", 1, &rvizClickCallback);
    mkr_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    /// 【4】启动异步Spinner
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();

    /// 【5】运行预置示例路径搜索
    rog_astar::Vec3f start, goal;
    rog_astar_ptr->getExampleStartGoal(start, goal);

    publishPointWithText(start, "start", rog_map::Color::Orange());
    publishPointWithText(goal, "goal", rog_map::Color::Green());

    const auto ret = rog_astar_ptr->runExample();
    if (ret) {
        ROS_INFO("Path found");
    }
    else {
        ROS_ERROR("Path not found");
    }

    ros::waitForShutdown();
    return 0;
}


/// 发布带文本标签的点标记
void publishPointWithText(const rog_map::Vec3f& p, const std::string& text, const rog_map::Color c) {
    // ===== 球体标记（位置指示器） =====
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "world";
    point_marker.header.stamp = ros::Time::now();
    point_marker.ns = text + "_pos";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = p(0);
    point_marker.pose.position.y = p(1);
    point_marker.pose.position.z = p(2);
    point_marker.pose.orientation.w = 1.0;
    point_marker.scale.x = 0.2;
    point_marker.scale.y = 0.2;
    point_marker.scale.z = 0.2;
    point_marker.color = c;
    point_marker.color.a = 1.0;

    // ===== 文本标记（标签） =====
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = text;
    text_marker.id = 1;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;  // 始终面向相机
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = p(0);
    text_marker.pose.position.y = p(1);
    text_marker.pose.position.z = p(2) + 0.3;  // 文字偏移在球体上方
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.5;
    text_marker.color = c;
    text_marker.color.a = 1.0;
    text_marker.text = text;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(point_marker);
    marker_array.markers.push_back(text_marker);
    mkr_pub.publish(marker_array);
}


/// RViz 点击回调
/// 使用 "3D Nav Goal" 工具点击RViz时触发
/// 交替接收起点和终点，每两个点触发一次路径搜索
void rvizClickCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    ROS_INFO("Note, the click height is set to 1.0 to ease the user interaction.");
    ROS_INFO("x: %f, y: %f, z: %f", msg->pose.position.x, msg->pose.position.y, 1.0);
    static rog_map::Vec3f start_pos, goal_pos;
    static bool is_start = true;

    /// 注意：为方便RViz交互，点击高度设置为1.0米
    if (is_start) {
        /// 第一个点 = 起点
        start_pos = rog_map::Vec3f(msg->pose.position.x, msg->pose.position.y, 1.0);
        is_start = false;
        publishPointWithText(start_pos, "start", rog_map::Color::Orange());
    }
    else {
        /// 第二个点 = 终点，触发路径搜索
        goal_pos = rog_map::Vec3f(msg->pose.position.x, msg->pose.position.y, 1.0);
        publishPointWithText(goal_pos, "goal", rog_map::Color::Green());

        /// 设置搜索标志位：
        /// UNKNOWN_AS_FREE: 未知区域视为可通过（激进策略，适合探索场景）
        /// ON_INF_MAP: 在膨胀地图上搜索（考虑机器人安全半径）
        int flag = rog_astar::UNKNOWN_AS_FREE | rog_astar::ON_INF_MAP;
        const auto ret = rog_astar_ptr->pathSearch(start_pos, goal_pos, 0.1, flag);
        if (ret) {
            ROS_INFO("Path found");
        }
        else {
            ROS_ERROR("Path not found");
        }

        is_start = true;  // 重置为下一次搜索准备
    }
}
