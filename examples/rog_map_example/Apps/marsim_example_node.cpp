/**
 * ROG-Map 示例：MARSIM 集成节点
 *
 * 功能：创建 ROGMap 实例并自动订阅里程计和点云话题，
 * 在 MARSIM 无人机仿真环境中实时更新局部滑动栅格地图。
 *
 * 使用方式：
 * 1. roslaunch test_interface single_drone_os128.launch  # 启动仿真
 * 2. roslaunch rog_map_example marsim_example.launch     # 启动此节点 + 键盘控制
 */

#include "rog_map/rog_map.h"


int main(int argc, char** argv) {
    /// 初始化ROS节点
    ros::init(argc, argv, "rm_node");
    ros::NodeHandle nh("~");

    /// 设置PCL控制台输出级别（减少无关日志）
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    /// 【1】创建 ROGMap 智能指针
    /// 构造函数会自动：
    /// - 从ROS参数服务器加载配置
    /// - 初始化概率地图和子地图（膨胀地图、ESDF地图、前沿地图）
    /// - 订阅里程计和点云话题（如果 ros_callback.enable = true）
    /// - 创建可视化发布器
    rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(nh);

    /// 【2】启动异步Spinner以处理ROS回调
    ros::AsyncSpinner spinner(0);  // 参数0 = 使用所有可用CPU核心
    spinner.start();
    ros::Duration(1.0).sleep();    // 等待1秒让系统初始化完成

    /// 保持节点运行直到收到关闭信号
    ros::waitForShutdown();
    return 0;
}
