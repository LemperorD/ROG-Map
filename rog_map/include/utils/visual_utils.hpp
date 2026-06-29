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

#include "utils/common_lib.hpp"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/MarkerArray.h"

namespace rog_map {
    /// RGB 颜色类，继承自 ROS 的 std_msgs::ColorRGBA
    /// 提供多种便捷的构造函数和预定义静态颜色
    class Color : public std_msgs::ColorRGBA {
    public:
        Color() : std_msgs::ColorRGBA() {}

        /// 从十六进制颜色值构造（如 0xFF0000 表示红色）
        /// @param hex_color 24位RGB颜色值，每通道8位
        Color(int hex_color) {
            int _r = (hex_color >> 16) & 0xFF;
            int _g = (hex_color >> 8) & 0xFF;
            int _b = hex_color & 0xFF;
            r = static_cast<double>(_r) / 255.0;
            g = static_cast<double>(_g) / 255.0;
            b = static_cast<double>(_b) / 255.0;
        }

        /// 从已有颜色和透明度创建新颜色
        Color(Color c, double alpha) {
            r = c.r;
            g = c.g;
            b = c.b;
            a = alpha;
        }

        /// 从RGB值构造（自动将>1的值视为0-255范围）
        Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {
            r = red > 1.0 ? red / 255.0 : red;
            g = green > 1.0 ? green / 255.0 : green;
            b = blue > 1.0 ? blue / 255.0 : blue;
        }

        /// 从RGBA值构造
        Color(double red, double green, double blue, double alpha) : Color() {
            r = red > 1.0 ? red / 255.0 : red;
            g = green > 1.0 ? green / 255.0 : green;
            b = blue > 1.0 ? blue / 255.0 : blue;
            a = alpha;
        }

        /// ===== 预定义静态颜色 =====
        static const Color White() { return Color(1.0, 1.0, 1.0); }
        static const Color Black() { return Color(0.0, 0.0, 0.0); }
        static const Color Gray() { return Color(0.5, 0.5, 0.5); }
        static const Color Red() { return Color(1.0, 0.0, 0.0); }
        static const Color Green() { return Color(0.0, 0.96, 0.0); }
        static const Color Blue() { return Color(0.0, 0.0, 1.0); }
        static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }
        static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
        static Color Orange() { return Color(1.0, 0.5, 0.0); }
        static const Color Purple() { return Color(0.5, 0.0, 1.0); }
        static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
        static const Color Teal() { return Color(0.0, 1.0, 1.0); }
        static const Color Pink() { return Color(1.0, 0.0, 0.5); }
    };


/* ===== 类型A: 直接通过 Publisher 发布 Marker ===== */

    /// 发布一个文本标签到 RViz
    /// @param pub ROS 发布器
    /// @param ns 命名空间（用于区分不同的Marker组）
    /// @param text 要显示的文本
    /// @param position 文本显示位置（世界坐标系）
    /// @param c 文本颜色
    /// @param size 文本大小（z方向缩放）
    /// @param id Marker的ID（-1表示自动分配）
    static void visualizeText(const ros::Publisher &pub,
                              const std::string &ns,
                              const std::string &text,
                              const Vec3f &position,
                              const Color &c,
                              const double &size,
                              const int &id = -1) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.ns = ns.c_str();
        // ID 分配：指定ID则使用指定值，否则自动递增
        if (id >= 0) {
            marker.id = id;
        } else {
            static int id = 0;
            marker.id = id++;
        }
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;  // 始终面向相机的文本
        marker.scale.z = size;
        marker.color = c;
        marker.text = text;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(marker);
        pub.publish(arr);
    };

    /// 发布一个球体点标记到 RViz
    /// @param pub_ ROS 发布器
    /// @param pt 点位置
    /// @param color 球体颜色（默认粉色）
    /// @param ns 命名空间
    /// @param size 球体大小
    /// @param id Marker ID
    /// @param print_ns 是否同时显示文本标签
    static void visualizePoint(const ros::Publisher &pub_,
                               const Vec3f &pt,
                               Color color = Color::Pink(),
                               std::string ns = "pt",
                               double size = 0.1, int id = -1,
                               const bool &print_ns = true) {
        visualization_msgs::MarkerArray mkr_arr;
        visualization_msgs::Marker marker_ball;
        static int cnt = 0;
        Vec3f cur_pos = pt;
        // 跳过NaN点
        if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
            return;
        }
        // 配置球体Marker
        marker_ball.header.frame_id = "world";
        marker_ball.header.stamp = ros::Time::now();
        marker_ball.ns = ns.c_str();
        marker_ball.id = id >= 0 ? id : cnt++;
        marker_ball.action = visualization_msgs::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::Marker::SPHERE;
        marker_ball.scale.x = size;
        marker_ball.scale.y = size;
        marker_ball.scale.z = size;
        marker_ball.color = color;

        geometry_msgs::Point p;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();

        marker_ball.pose.position = p;
        mkr_arr.markers.push_back(marker_ball);

        // 添加文本标签（如果需要）
        if (print_ns) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns + "_text";
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.6;
            marker.color = color;
            marker.text = ns;
            marker.pose.position.x = cur_pos.x();
            marker.pose.position.y = cur_pos.y();
            marker.pose.position.z = cur_pos.z() + 0.5;  // 文字偏移在球体上方
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        }

        pub_.publish(mkr_arr);
    }

    /// 直接发布轴对齐包围盒到 RViz
    /// @param pub ROS 发布器
    /// @param box_min 包围盒最小角坐标
    /// @param box_max 包围盒最大角坐标
    /// @param ns 命名空间
    /// @param color 线条颜色
    /// @param size_x 线条宽度
    /// @param alpha 不透明度（0=全透明，1=不透明）
    /// @param print_ns 是否显示文本标签
    static void visualizeBoundingBox(const ros::Publisher &pub,
                                     const Vec3f &box_min,
                                     const Vec3f &box_max,
                                     const string &ns,
                                     const Color &color,
                                     const double &size_x = 0.1,
                                     const double &alpha = 1.0,
                                     const bool &print_ns = true) {
        Vec3f size = (box_max - box_min) / 2;         // 半边长
        Vec3f vis_pos_world = (box_min + box_max) / 2; // 中心位置
        double width = size.x();
        double length = size.y();
        double hight = size.z();
        visualization_msgs::MarkerArray mkrarr;

        int id = 0;
        visualization_msgs::Marker line_strip;
        line_strip.header.stamp = ros::Time::now();
        line_strip.header.frame_id = "world";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.ns = ns;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = id++;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;  // 折线类型
        line_strip.scale.x = size_x;

        line_strip.color = color;
        line_strip.color.a = alpha;
        geometry_msgs::Point p[8];

        // 计算包围盒的8个顶点
        p[0].x = vis_pos_world(0) - width;
        p[0].y = vis_pos_world(1) + length;
        p[0].z = vis_pos_world(2) + hight;
        p[1].x = vis_pos_world(0) - width;
        p[1].y = vis_pos_world(1) - length;
        p[1].z = vis_pos_world(2) + hight;
        p[2].x = vis_pos_world(0) - width;
        p[2].y = vis_pos_world(1) - length;
        p[2].z = vis_pos_world(2) - hight;
        p[3].x = vis_pos_world(0) - width;
        p[3].y = vis_pos_world(1) + length;
        p[3].z = vis_pos_world(2) - hight;
        p[4].x = vis_pos_world(0) + width;
        p[4].y = vis_pos_world(1) + length;
        p[4].z = vis_pos_world(2) - hight;
        p[5].x = vis_pos_world(0) + width;
        p[5].y = vis_pos_world(1) - length;
        p[5].z = vis_pos_world(2) - hight;
        p[6].x = vis_pos_world(0) + width;
        p[6].y = vis_pos_world(1) - length;
        p[6].z = vis_pos_world(2) + hight;
        p[7].x = vis_pos_world(0) + width;
        p[7].y = vis_pos_world(1) + length;
        p[7].z = vis_pos_world(2) + hight;

        // LINE_STRIP 连接相邻点：0→1→2→3...
        for (int i = 0; i < 8; i++) {
            line_strip.points.push_back(p[i]);
        }
        // 补充连接保证8条边都画出（立方体12条边 + 额外对角线）
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[3]);
        line_strip.points.push_back(p[2]);
        line_strip.points.push_back(p[5]);
        line_strip.points.push_back(p[6]);
        line_strip.points.push_back(p[1]);
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[7]);
        line_strip.points.push_back(p[4]);
        mkrarr.markers.push_back(line_strip);
        pub.publish(mkrarr);
    }

/* ===== 类型B: 将 Marker 添加到给定的 MarkerArray 中，以便稍后统一发布 ===== */

    /// 向 MarkerArray 中添加文本标签
    static void visualizeText(visualization_msgs::MarkerArray &mkr_arr,
                              const std::string &ns,
                              const std::string &text,
                              const Vec3f &position,
                              const Color &c = Color::White(),
                              const double &size = 0.6,
                              const int &id = -1) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.ns = ns.c_str();
        if (id >= 0) {
            marker.id = id;
        } else {
            static int id = 0;
            marker.id = id++;
        }
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.scale.z = size;
        marker.color = c;
        marker.text = text;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        mkr_arr.markers.push_back(marker);
    };

    /// 向 MarkerArray 中添加球体点标记
    static void visualizePoint(visualization_msgs::MarkerArray &mkr_arr,
                               const Vec3f &pt,
                               Color color = Color::Pink(),
                               std::string ns = "pt",
                               double size = 0.1, int id = -1,
                               const bool &print_ns = true) {
        visualization_msgs::Marker marker_ball;
        static int cnt = 0;
        Vec3f cur_pos = pt;
        if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
            return;
        }
        marker_ball.header.frame_id = "world";
        marker_ball.header.stamp = ros::Time::now();
        marker_ball.ns = ns.c_str();
        marker_ball.id = id >= 0 ? id : cnt++;
        marker_ball.action = visualization_msgs::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::Marker::SPHERE;
        marker_ball.scale.x = size;
        marker_ball.scale.y = size;
        marker_ball.scale.z = size;
        marker_ball.color = color;

        geometry_msgs::Point p;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();

        marker_ball.pose.position = p;
        mkr_arr.markers.push_back(marker_ball);

        // 添加文本标签
        if (print_ns) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns + "_text";
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.6;
            marker.color = color;
            marker.text = ns;
            marker.pose.position.x = cur_pos.x();
            marker.pose.position.y = cur_pos.y();
            marker.pose.position.z = cur_pos.z() + 0.5;
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        }
    }

    /// 向 MarkerArray 中添加包围盒
    /// 使用 LINE_STRIP 连续折线画出立方体的所有边
    static void visualizeBoundingBox(visualization_msgs::MarkerArray &mkrarr,
                                     const Vec3f &box_min,
                                     const Vec3f &box_max,
                                     const string &ns,
                                     const Color &color,
                                     const double &size_x = 0.1,
                                     const double &alpha = 1.0,
                                     const bool &print_ns = true) {
        Vec3f size = (box_max - box_min) / 2;
        Vec3f vis_pos_world = (box_min + box_max) / 2;
        double width = size.x();
        double length = size.y();
        double hight = size.z();

        int id = 0;
        visualization_msgs::Marker line_strip;
        line_strip.header.stamp = ros::Time::now();
        line_strip.header.frame_id = "world";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.ns = ns;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = id++;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = size_x;

        line_strip.color = color;
        line_strip.color.a = alpha;  // 设置不透明度
        geometry_msgs::Point p[8];

        // 计算立方体的8个顶点
        p[0].x = vis_pos_world(0) - width;
        p[0].y = vis_pos_world(1) + length;
        p[0].z = vis_pos_world(2) + hight;
        p[1].x = vis_pos_world(0) - width;
        p[1].y = vis_pos_world(1) - length;
        p[1].z = vis_pos_world(2) + hight;
        p[2].x = vis_pos_world(0) - width;
        p[2].y = vis_pos_world(1) - length;
        p[2].z = vis_pos_world(2) - hight;
        p[3].x = vis_pos_world(0) - width;
        p[3].y = vis_pos_world(1) + length;
        p[3].z = vis_pos_world(2) - hight;
        p[4].x = vis_pos_world(0) + width;
        p[4].y = vis_pos_world(1) + length;
        p[4].z = vis_pos_world(2) - hight;
        p[5].x = vis_pos_world(0) + width;
        p[5].y = vis_pos_world(1) - length;
        p[5].z = vis_pos_world(2) - hight;
        p[6].x = vis_pos_world(0) + width;
        p[6].y = vis_pos_world(1) - length;
        p[6].z = vis_pos_world(2) + hight;
        p[7].x = vis_pos_world(0) + width;
        p[7].y = vis_pos_world(1) + length;
        p[7].z = vis_pos_world(2) + hight;

        // 按顺序连接顶点构成折线
        for (int i = 0; i < 8; i++) {
            line_strip.points.push_back(p[i]);
        }
        // 补充额外的边以画全立方体的12条棱
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[3]);
        line_strip.points.push_back(p[2]);
        line_strip.points.push_back(p[5]);
        line_strip.points.push_back(p[6]);
        line_strip.points.push_back(p[1]);
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[7]);
        line_strip.points.push_back(p[4]);
        mkrarr.markers.push_back(line_strip);
    }

}
