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

#include "utils/raycaster.h"

namespace rog_map {
    namespace raycaster {
        /// 构造函数：设置分辨率并检查 ORIGIN 宏的互斥性
        RayCaster::RayCaster(const double &resolution) {
#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
            // 这两个宏不能同时定义
            throw std::runtime_error(" -- [SlidingMap]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both true!");
#endif
#endif

            if (resolution < 0) {
                throw std::runtime_error(" -- [Raycaster]: resolution must be positive!");
            } else {
                resolution_ = resolution;
            }
        }

        void RayCaster::setResolution(const double &resolution) {
            resolution_ = resolution;
        }

        /// 世界坐标 → 栅格索引
        /// ORIGIN_AT_CORNER: id = floor(d / resolution)，原点在栅格角上
        /// ORIGIN_AT_CENTER: id = round(d / resolution)，原点在栅格中心
        void RayCaster::posToIndex(const double d, int &id) const {
#ifdef ORIGIN_AT_CORNER
            id = std::floor((d / resolution_));
#endif

#ifdef ORIGIN_AT_CENTER
            id = static_cast<int>((d / resolution_ + signum(d) * 0.5));
#endif
        }

        /// 栅格索引 → 世界坐标（栅格中心）
        /// ORIGIN_AT_CORNER: pos = (id + 0.5) * resolution
        /// ORIGIN_AT_CENTER: pos = id * resolution
        void RayCaster::indexToPos(const int &id, double &d) const {
#ifdef ORIGIN_AT_CORNER
            d = (id + 0.5) * resolution_;
#endif
#ifdef ORIGIN_AT_CENTER
            d = id * resolution_;
#endif
        }

        /// 设置射线并初始化遍历参数
        /// 实现 3D DDA 算法的初始化阶段：
        /// 1) 计算起始和终止栅格索引
        /// 2) 计算各方向的扩展方向
        /// 3) 计算到达第一个栅格边界的 t 参数
        /// 4) 计算每步跨越一个栅格所需的 t 增量
        bool RayCaster::setInput(const Eigen::Vector3d &start, const Eigen::Vector3d &end) {
            if (resolution_ < 0) {
                throw std::runtime_error(" -- [RayCaster] Resolution is not set!");
            }
            // 保存起点和终点的世界坐标
            start_x_d_ = start.x();
            start_y_d_ = start.y();
            start_z_d_ = start.z();

            end_x_d_ = end.x();
            end_y_d_ = end.y();
            end_z_d_ = end.z();

            // 将起点和终点转换为栅格索引
            posToIndex(start_x_d_, start_x_i_);
            posToIndex(start_y_d_, start_y_i_);
            posToIndex(start_z_d_, start_z_i_);

            posToIndex(end_x_d_, end_x_i_);
            posToIndex(end_y_d_, end_y_i_);
            posToIndex(end_z_d_, end_z_i_);

            // 计算各方向的索引增量
            int delta_X = end_x_i_ - start_x_i_;
            int delta_Y = end_y_i_ - start_y_i_;
            int delta_Z = end_z_i_ - start_z_i_;

            // 设置当前栅格索引为起点索引
            cur_ray_pt_id_x_ = start_x_i_;
            cur_ray_pt_id_y_ = start_y_i_;
            cur_ray_pt_id_z_ = start_z_i_;

            // 确定各方向的扩展方向：+1（正向）、-1（反向）或 0（不变）
            expand_dir_x_ = static_cast<int>(signum(static_cast<int>(delta_X)));
            expand_dir_y_ = static_cast<int>(signum(static_cast<int>(delta_Y)));
            expand_dir_z_ = static_cast<int>(signum(static_cast<int>(delta_Z)));

            // 如果起点和终点在同一栅格，则不需要遍历
            if (expand_dir_x_ == 0 && expand_dir_y_ == 0 && expand_dir_z_ == 0) {
                return false;
            }

            // 计算射线总长度和各方向分量长度
            double dis_x_over_t, dis_y_over_t, dis_z_over_t, t_max;
            dis_x_over_t = std::abs(end_x_d_ - start_x_d_);
            dis_y_over_t = std::abs(end_y_d_ - start_y_d_);
            dis_z_over_t = std::abs(end_z_d_ - start_z_d_);

            t_max = sqrt(dis_x_over_t * dis_x_over_t + dis_y_over_t * dis_y_over_t + dis_z_over_t * dis_z_over_t);

            // 归一化方向分量（即射线的方向余弦）
            dis_x_over_t /= t_max;
            dis_y_over_t /= t_max;
            dis_z_over_t /= t_max;

            // 构建步进查找表
            // t_when_step_x_ 表示沿X方向穿越一个栅格所需的参数t的增量
            // 如果某方向无移动（expand_dir==0），设置为无穷大
            t_when_step_x_ = expand_dir_x_ == 0 ? std::numeric_limits<double>::max() :
                             std::abs(resolution_ / dis_x_over_t);
            t_when_step_y_ = expand_dir_y_ == 0 ? std::numeric_limits<double>::max() :
                             std::abs(resolution_ / dis_y_over_t);
            t_when_step_z_ = expand_dir_z_ == 0 ? std::numeric_limits<double>::max() :
                             std::abs(resolution_ / dis_z_over_t);

            // 计算起始栅格中心位置
            double start_grid_center_x, start_grid_center_y, start_grid_center_z;
            indexToPos(start_x_i_, start_grid_center_x);
            indexToPos(start_y_i_, start_grid_center_y);
            indexToPos(start_z_i_, start_grid_center_z);

            // 计算下一个栅格边界的位置
            double next_bound_x, next_bound_y, next_bound_z;
            next_bound_x = start_grid_center_x + expand_dir_x_ * resolution_ * 0.5;
            next_bound_y = start_grid_center_y + expand_dir_y_ * resolution_ * 0.5;
            next_bound_z = start_grid_center_z + expand_dir_z_ * resolution_ * 0.5;

            // 计算到达第一个栅格边界所需的参数t
            // t_to_bound_x_ 表示射线从起点到第一个X方向栅格边界所需的参数t
            t_to_bound_x_ = expand_dir_x_ == 0 ? std::numeric_limits<double>::max() :
                            std::fabs(next_bound_x - start_x_d_) / dis_x_over_t;
            t_to_bound_y_ = expand_dir_y_ == 0 ? std::numeric_limits<double>::max() :
                            std::fabs(next_bound_y - start_y_d_) / dis_y_over_t;
            t_to_bound_z_ = expand_dir_z_ == 0 ? std::numeric_limits<double>::max() :
                            std::fabs(next_bound_z - start_z_d_) / dis_z_over_t;


            step_num_ = 0;
            first_point = true;
            return true;
        }

        /// 沿射线步进一步
        /// 使用 3D DDA 算法：比较 t_to_bound_x/y/z，选择最小的作为下一步方向
        /// @param ray_pt 输出：当前栅格中心的世界坐标
        /// @return 如果到达终点栅格返回 false
        bool RayCaster::step(Eigen::Vector3d &ray_pt) {
            step_num_++;

            // 输出当前栅格中心的世界坐标
            indexToPos(cur_ray_pt_id_x_, ray_pt.x());
            indexToPos(cur_ray_pt_id_y_, ray_pt.y());
            indexToPos(cur_ray_pt_id_z_, ray_pt.z());

            // 检查是否已到达终点栅格
            if (cur_ray_pt_id_x_ == end_x_i_ && cur_ray_pt_id_y_ == end_y_i_ && cur_ray_pt_id_z_ == end_z_i_) {
                return false;
            }

            // 选择 t_to_bound 最小的方向进行步进（即最先碰到哪个方向的栅格边界）
            if (t_to_bound_x_ < t_to_bound_y_) {
                if (t_to_bound_x_ < t_to_bound_z_) {
                    // X方向最先到达边界
                    cur_ray_pt_id_x_ += expand_dir_x_;       // 移动到下一个栅格
                    t_to_bound_x_ += t_when_step_x_;          // 更新到达下个边界的t
                } else {
                    // Z方向最先到达边界
                    cur_ray_pt_id_z_ += expand_dir_z_;
                    t_to_bound_z_ += t_when_step_z_;
                }
            } else {
                if (t_to_bound_y_ < t_to_bound_z_) {
                    // Y方向最先到达边界
                    cur_ray_pt_id_y_ += expand_dir_y_;
                    t_to_bound_y_ += t_when_step_y_;
                } else {
                    // Z方向最先到达边界
                    cur_ray_pt_id_z_ += expand_dir_z_;
                    t_to_bound_z_ += t_when_step_z_;

                }
            }
            return true;
        }

    }
}