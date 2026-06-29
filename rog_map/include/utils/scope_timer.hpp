/**
* 本文件是 ROG-Map 的一部分
*
* 版权所有 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* 由 Yunfan REN <renyf at connect dot hku dot hk> 开发
* 更多信息请参见 <https://github.com/hku-mars/ROG-Map>.
* 如果您使用此代码，请参阅上述网站上列出的相应出版物。
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

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>

namespace rog_map {
    /// RAII 风格的高精度计时器类
    /// 使用 std::chrono::high_resolution_clock 进行时间测量
    /// 在析构时自动输出耗时信息（ns/µs/ms/s 自适应单位）
    /// 也可以手动调用 stop() 获取耗时
    class TimeConsuming {

    public:
        TimeConsuming();

        /// 构造函数：指定任务名称和重复次数
        /// @param msg 计时器标签名称（用于输出识别）
        /// @param repeat_time 重复次数，用于计算单次平均耗时
        TimeConsuming(std::string msg, int repeat_time) {
            repeat_time_ = repeat_time;
            msg_ = msg;
            tc_start = std::chrono::high_resolution_clock::now();  // 记录开始时间
            has_shown = false;                                      // 尚未输出结果
            print_ = true;                                          // 启用自动打印
        }

        /// 构造函数：指定任务名称和是否输出日志
        /// @param msg 计时器标签
        /// @param print_log 是否在析构时自动输出耗时（false时只能通过stop()获取）
        TimeConsuming(std::string msg, bool print_log) {
            msg_ = msg;
            repeat_time_ = 1;
            print_ = print_log;
            tc_start = std::chrono::high_resolution_clock::now();
            has_shown = false;
        }

        /// 析构函数：如果启用了打印且尚未输出，自动输出耗时
        ~TimeConsuming() {
            if (!has_shown && enable_ && print_) {
                tc_end = std::chrono::high_resolution_clock::now();
                double dt = std::chrono::duration_cast<std::chrono::duration<double >>(tc_end - tc_start).count();
                double t_us = (double) dt * 1e6 / repeat_time_;     // 转换为微秒并计算平均

                // 根据耗时大小选择合适的单位输出
                if (t_us < 1) {
                    // 小于1微秒，使用纳秒
                    t_us *= 1000;
                    printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
                } else if (t_us > 1e6) {
                    // 大于1秒，使用秒
                    t_us /= 1e6;
                    printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
                } else if (t_us > 1e3) {
                    // 大于1毫秒，使用毫秒
                    t_us /= 1e3;
                    printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
                } else
                    // 1微秒至1毫秒，使用微秒
                    printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
            }
        }

        /// 启用/禁用计时器
        void set_enbale(bool enable) {
            enable_ = enable;
        }

        /// 重新开始计时（重置起始时间点）
        void start() {
            tc_start = std::chrono::high_resolution_clock::now();
        }

        /// 停止计时并返回耗时
        /// @return 从 start() 或构造到现在的耗时（秒）
        double stop() {
            if (!enable_) { return 0; }
            has_shown = true;                                        // 标记已输出，防止析构重复输出
            tc_end = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double >>(tc_end - tc_start).count();

            if (!print_) {
                return dt;                                           // 不打印，直接返回耗时
            }

            double t_us = (double) dt * 1e6 / repeat_time_;
            if (t_us < 1) {
                t_us *= 1000;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e6) {
                t_us /= 1e6;
                printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e3) {
                t_us /= 1e3;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
            } else
                printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
            return dt;                                               // 返回原始耗时（秒）
        }

    private:
        std::chrono::high_resolution_clock::time_point tc_start, tc_end;  ///< 开始和结束时间点
        std::string msg_;                                                   ///< 计时器标识名称
        int repeat_time_{1};                                                ///< 重复次数（用于平均计算）
        bool has_shown = false;                                             ///< 是否已输出结果
        bool enable_{true};                                                 ///< 计时器是否启用
        bool print_{true};                                                  ///< 是否打印输出
    };
}
