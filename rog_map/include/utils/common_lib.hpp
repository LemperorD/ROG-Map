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

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Eigen>


/// 调试文件和PCD文件的路径宏定义
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#define PCD_FILE_DIR(name) (string(string(ROOT_DIR) + "pcd/"+name))

namespace rog_map {
    /// 数值类型定义，使用双精度浮点数
    typedef double decimal_t;
    /// 三维整数向量（用于网格索引），例如网格坐标(1,2,3)
    using Vec3i = Eigen::Matrix<int, 3, 1>;
    /// 三维浮点向量（用于世界坐标），例如位置(1.0, 2.0, 3.0)
    using Vec3f = Eigen::Matrix<decimal_t, 3, 1>;
    /// 四元数类型，用于表示旋转
    using Quatf = Eigen::Quaterniond;
    /// 位姿类型：位置(Vec3f) + 姿态四元数的 pair
    using Pose = std::pair<Vec3f, Eigen::Quaterniond>;
    /// 使用Eigen内存对齐分配器的向量模板
    template<typename T>
    using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
    /// 三维网格索引向量
    using vec_Vec3i = vec_E<Vec3i>;
    /// 三维位置向量
    using vec_Vec3f = vec_E<Vec3f>;

    using std::vector;
    using std::string;
    using std::cout;
    using std::endl;

    /// 坐标轴常量定义
    constexpr int AXIS_X = 0;
    constexpr int AXIS_Y = 1;
    constexpr int AXIS_Z = 2;
    /// 常量 (0.5, 0.5, 0.5)，用于索引到位置的偏移计算
    const Vec3f ZeroPointFive3d(0.5, 0.5, 0.5);

    /// ANSI终端颜色控制码：红色，用于错误输出
    const std::string RED = "\033[1;31m";
    /// ANSI终端颜色控制码：绿色，用于成功输出
    const std::string GREEN = "\033[1;32m";
    /// ANSI终端颜色控制码：黄色，用于警告输出
    const std::string YELLOW = "\033[1;33m";
    /// ANSI终端颜色控制码：蓝色，用于信息输出
    const std::string BLUE = "\033[1;34m";
    /// ANSI终端颜色重置码
    const std::string RESET = "\033[0m";

    typedef pcl::PointXYZINormal PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;


    /// 栅格类型枚举
    /// 用于表示地图中每个栅格的状态
    enum GridType {
        UNDEFINED = 0,   ///< 未定义状态
        UNKNOWN = 1,     ///< 未知状态：既不是已知空闲也不是占用的区域
        OUT_OF_MAP,      ///< 超出地图范围
        OCCUPIED,        ///< 占用状态：有障碍物的区域
        KNOWN_FREE,      ///< 已知空闲状态：确认无障碍物的区域
        FRONTIER,        ///< 前沿：与已知空闲栅格相邻的未知栅格，表示探索边界
    };

    /// 栅格类型对应的字符串名称数组，用于调试输出
    const static std::vector<std::string> GridTypeStr{"UNDEFINED",
                                                      "UNKNOWN",
                                                      "OUT_OF_MAP",
                                                      "OCCUPIED",
                                                      "KNOWN_FREE",
                                                      "FRONTIER"};

/// 符号函数：x>0返回1，x<0返回-1，x=0返回0
#define SIGN(x) ((x > 0) - (x < 0))
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#define PCD_FILE_DIR(name) (string(string(ROOT_DIR) + "pcd/"+name))

    /// 重载 << 运算符，用于打印 vector 类型
    template<typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
        out << "[";
        for (typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it) {
            out << *it;
            if (it != v.end() - 1) {
                out << ", ";
            }
        }
        out << "]";
        return out;
    }

    /// 机器人状态结构体
    /// 存储机器人的位置、速度、加速度、姿态和接收时间
    struct RobotState {
        Vec3f p, v, a, j;        ///< 位置(p)、速度(v)、加速度(a)、加加速度(j)
        double yaw;               ///< 偏航角
        double rcv_time;          ///< 接收到消息的时间戳
        bool rcv{false};          ///< 是否已接收到里程计数据
        Quatf q;                  ///< 当前姿态四元数
    };


    /// 将四元数转换为 YPR（偏航-俯仰-横滚）欧拉角
    /// @tparam Scalar_t 标量类型（float 或 double）
    /// @param q_ 输入四元数（会自动归一化）
    /// @return 3x1 向量，依次为 [yaw, pitch, roll]
    template<typename Scalar_t>
    static Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t> &q_) {
        Eigen::Quaternion<Scalar_t> q = q_.normalized();

        Eigen::Matrix<Scalar_t, 3, 1> ypr;
        ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

        return ypr;
    }

    /// 从四元数中提取偏航角（绕Z轴旋转的角度）
    /// @tparam Scalar_t 标量类型
    /// @param q 输入四元数
    /// @return 偏航角（弧度）
    template<typename Scalar_t>
    static Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t> &q) {
        return quaternion_to_ypr(q)(0);
    }




    /// 计算路径的总长度
    /// @param path 由Vec3f点组成的路径
    /// @return 路径的总欧氏距离
    static double computePathLength(const vec_E<Vec3f> &path) {
        if (path.size() < 2) {
            return 0.0;
        }
        double len = 0.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            len += (path[i] - path[i + 1]).norm();
        }
        return len;
    }

    /// 计算从pos出发到pt的射线与轴对齐包围盒(box_min, box_max)的交点
    /// 用于将射线截断到局部更新范围内
    /// @param pt 射线终点
    /// @param pos 射线起点（通常是机器人当前位置）
    /// @param box_min 包围盒最小坐标
    /// @param box_max 包围盒最大坐标
    /// @return 射线与包围盒的交点（如果相交）；否则返回最近边界点
    static Vec3f lineBoxIntersectPoint(const Vec3f &pt, const Vec3f &pos,
                                       const Vec3f &box_min, const Vec3f &box_max) {
        Eigen::Vector3d diff = pt - pos;                          // 射线方向向量
        Eigen::Vector3d max_tc = box_max - pos;                   // 到最大边界的距离
        Eigen::Vector3d min_tc = box_min - pos;                   // 到最小边界的距离

        double min_t = 1000000;                                   // 记录最近交点参数

        for (int i = 0; i < 3; ++i) {
            if (fabs(diff[i]) > 0) {
                // 计算射线与每个轴对齐平面的交点参数 t
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t)
                    min_t = t1;

                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t)
                    min_t = t2;
            }
        }

        // 返回交点位置（略微偏移1e-3避免数值精度问题）
        return pos + (min_t - 1e-3) * diff;
    }

    /// 获取线段与平面的交点（辅助函数）
    /// @param fDst1 线段端点1到平面的有符号距离
    /// @param fDst2 线段端点2到平面的有符号距离
    /// @param P1 线段端点1
    /// @param P2 线段端点2
    /// @param Hit 输出交点
    /// @return 如果线段与平面相交返回true，否则false
    static bool GetIntersection(float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f &Hit) {
        if ((fDst1 * fDst2) >= 0.0f) return false;               // 两端点在平面同侧，不相交
        if (fDst1 == fDst2) return false;                         // 平行，不相交
        Hit = P1 + (P2 - P1) * (-fDst1 / (fDst2 - fDst1));        // 线性插值求交点
        return true;
    }

    /// 检查交点是否在包围盒的指定轴对面内
    /// @param Hit 交点
    /// @param B1 包围盒最小角
    /// @param B2 包围盒最大角
    /// @param Axis 轴编号：1=YZ平面，2=XZ平面，3=XY平面
    /// @return 如果交点在对应平面范围内返回true
    static bool InBox(Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis) {
        if (Axis == 1 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.y() > B1.y() && Hit.y() < B2.y()) return true;
        if (Axis == 2 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.x() > B1.x() && Hit.x() < B2.x()) return true;
        if (Axis == 3 && Hit.x() > B1.x() && Hit.x() < B2.x() && Hit.y() > B1.y() && Hit.y() < B2.y()) return true;
        return false;
    }

/// 线段与轴对齐包围盒的相交检测
/// 包围盒由B1(最小坐标)和B2(最大坐标)定义
/// @param L1 线段起点
/// @param L2 线段终点
/// @param B1 包围盒最小角
/// @param B2 包围盒最大角
/// @param Hit 输出交点
/// @return 如果相交返回true，否则false
    static bool lineIntersectBox(Vec3f L1, Vec3f L2, Vec3f B1, Vec3f B2, Vec3f &Hit) {
        // 快速排斥测试：线段完全在包围盒某侧之外
        if (L2.x() < B1.x() && L1.x() < B1.x()) return false;
        if (L2.x() > B2.x() && L1.x() > B2.x()) return false;
        if (L2.y() < B1.y() && L1.y() < B1.y()) return false;
        if (L2.y() > B2.y() && L1.y() > B2.y()) return false;
        if (L2.z() < B1.z() && L1.z() < B1.z()) return false;
        if (L2.z() > B2.z() && L1.z() > B2.z()) return false;

        // 检测线段与包围盒六个面的交点
        if ((GetIntersection(L1.x() - B1.x(), L2.x() - B1.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
            || (GetIntersection(L1.y() - B1.y(), L2.y() - B1.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
            || (GetIntersection(L1.z() - B1.z(), L2.z() - B1.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3))
            || (GetIntersection(L1.x() - B2.x(), L2.x() - B2.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
            || (GetIntersection(L1.y() - B2.y(), L2.y() - B2.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
            || (GetIntersection(L1.z() - B2.z(), L2.z() - B2.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3)))
            return true;

        return false;
    }
}
