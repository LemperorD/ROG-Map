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

#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include "memory"
#include "utils/common_lib.hpp"

#define RAYCAST_MODE ORIGIN_TYPE
namespace rog_map {


    namespace raycaster {
        /// 符号函数模板：val>0 返回 1，val<0 返回 -1，val=0 返回 0
        template<typename T>
        int signum(T val) {
            return (T(0) < val) - (val < T(0));
        }

        /// 3D DDA（数字微分分析器）光线投射器
        /// 用于在3D体素网格中高效遍历一条射线穿过的所有栅格
        /// 基于 Amanatides & Woo 1987 的网格遍历算法
        class RayCaster {
        public:
            typedef std::shared_ptr<RayCaster> Ptr;

            RayCaster() = default;

            /// 设置栅格分辨率
            /// @param resolution 每个栅格的边长（米）
            void setResolution(const double &resolution);

            /// 构造函数，同时设置分辨率
            RayCaster(const double &resolution);

            ~RayCaster() = default;

            /// 将世界坐标转换为栅格索引
            /// 根据 ORIGIN_AT_CORNER 或 ORIGIN_AT_CENTER 宏采用不同的舍入策略
            /// @param d 世界坐标（米）
            /// @param id 输出栅格索引
            void posToIndex(const double d, int &id) const;

            /// 将栅格索引转换为世界坐标（栅格中心位置）
            /// @param id 栅格索引
            /// @param d 输出世界坐标（米）
            void indexToPos(const int &id, double &d) const;

            /// 设置射线的起点和终点
            /// 计算射线在3D网格中的遍历参数
            /// @param start 射线起点（世界坐标）
            /// @param end 射线终点（世界坐标）
            /// @return 如果起点和终点在同一栅格中返回 false，否则返回 true
            bool setInput(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

            /// 沿射线前进一步，获取下一个穿过的栅格中心坐标
            /// @param ray_pt 输出当前栅格的世界坐标
            /// @return 如果到达终点栅格返回 false，否则返回 true
            bool step(Eigen::Vector3d &ray_pt);

        private:
            double resolution_{-1};              ///< 栅格分辨率（米/栅格）
            bool first_point{true};              ///< 是否为第一个点

            // ===== 射线遍历状态变量 =====
            double start_x_d_, start_y_d_, start_z_d_;   ///< 射线起始点世界坐标
            double end_x_d_, end_y_d_, end_z_d_;         ///< 射线终止点世界坐标
            double t_to_bound_x_, t_to_bound_y_, t_to_bound_z_;  ///< 到达下一个X/Y/Z边界所需的参数t
            int expand_dir_x_, expand_dir_y_, expand_dir_z_;     ///< X/Y/Z方向的扩展方向（-1, 0, 或 +1）
            int end_x_i_, end_y_i_, end_z_i_;                     ///< 射线终点栅格索引
            int start_x_i_, start_y_i_, start_z_i_;               ///< 射线起点栅格索引
            int cur_ray_pt_id_x_, cur_ray_pt_id_y_, cur_ray_pt_id_z_;  ///< 当前射线所在的栅格索引
            double t_when_step_x_, t_when_step_y_, t_when_step_z_; ///< 每步跨越一个栅格所需要的t增量
            int step_num_{0};                                      ///< 已步进次数计数器
        };
    }
}
