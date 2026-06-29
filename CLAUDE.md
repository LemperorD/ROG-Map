# CLAUDE.md

## 项目概述

ROG-Map（Robocentric Occupancy Grid Map，以机器人为中心的占用栅格地图）是一个高效、基于激光雷达的局部占用栅格地图框架，适用于大场景、高分辨率的运动规划。论文发表于 IROS 2024，由香港大学 MaRS 实验室开发。基于 ROS 1 Noetic，使用 C++14。

**核心特性：**
- **零拷贝地图滑动** — 通过模运算索引实现地图随机器人移动而滑动，无需重新分配内存
- **增量地图扩展** — 基于概率射线投射的占用更新，支持批处理
- **基于计数器的多分辨率地图** — 分离的概率地图、膨胀地图、ESDF 地图和前沿地图层，分辨率独立

## 构建系统

- **构建工具:** catkin (ROS 1)
- **CMake 项目:** `rog_map`（静态库）
- **依赖:** Eigen3, PCL, ROS (roscpp, std_msgs, pcl_ros, geometry_msgs, nav_msgs, visualization_msgs, message_filters), libdw（用于 backward-cpp 堆栈回溯）
- **编译选项:** `-O3 -Wall -g -fPIC`，C++14 标准
- **离散化模式:** `ORIGIN_AT_CORNER`（栅格位置在栅格中心；原点在原点的栅格角上）

### 构建命令
```bash
mkdir -p rog_ws/src && cd rog_ws/src
git clone https://github.com/hku-mars/ROG-Map.git
cd ..
catkin_make -DBUILD_TYPE=Release
```

## 架构设计

### 类继承层次（自顶向下）

```
SlidingMap（抽象基类 — 零拷贝循环缓冲区地图）
├── ProbMap — 概率占用栅格地图，含射线投射
│   └── ROGMap — 公开 API + ROS 接口（里程计/点云回调、可视化）
├── CounterMap（抽象 — 多分辨率基于计数器的地图）
│   ├── InfMap — 球面膨胀障碍物地图
│   └── ESDFMap — 欧几里得符号距离场，支持增量更新
└── FreeCntMap — 通过空闲邻居计数进行前沿提取
```

### 核心概念

**滑动地图 SlidingMap（sliding_map.h/.cpp）**
基础数据结构。使用 3D 循环缓冲区（模索引）数组，使地图可以"滑动"以跟随机器人，无需内存拷贝或重新分配。关键操作：
- `posToGlobalIndex()` / `globalIndexToPos()` — 世界坐标与全局栅格索引之间的转换。在 `ORIGIN_AT_CORNER` 模式下：`id = floor(pos / resolution)`，`pos = (id + 0.5) * resolution`
- `globalIndexToLocalIndex()` — 使用模运算将全局索引包裹到本地缓冲区（对应论文公式(7)-(8)）
- `mapSliding(odom)` — 平移地图原点，清除移出边界的栅格，对每个清除的栅格调用 `resetCell()`
- `getLocalIndexHash()` — 将 3D 索引展开为 1D 数组偏移量

**概率地图 ProbMap（prob_map.h/.cpp）**
通过射线投射维护对数几率(log-odds)占用概率：
- `occupancy_buffer_` — 每个栅格的对数几率值的一维数组
- `raycastProcess()` — 对每个激光雷达点，从 `raycast_range_min` 向该点投射射线。命中端点获得 `p_hit` 更新；穿越的空闲空间获得 `p_miss` 更新
- `probabilisticMapFromCache()` — 将缓存的命中/未命中计数批量应用到对数几率值
- `insertUpdateCandidate()` — 将栅格加入概率更新队列（通过计数器去重）
- 状态转换会触发 `InfMap::updateGridCounter()`、`ESDFMap::updateGridCounter()` 和前沿计数器更新

**计数器地图 CounterMap（counter_map.h/.cpp）**
多分辨率抽象层。每个计数器地图栅格覆盖基础概率地图的 `膨胀比例³` 个子栅格：
- `md_.occupied_cnt` — 被占用的子栅格数量
- `md_.unknown_cnt` — 未知的子栅格数量
- 计数器地图栅格的判断规则：若 `occupied_cnt > 0` 则为占用(OCCUPIED)；若 `unknown_cnt >= unk_thresh` 则为未知(UNKNOWN)；否则为已知空闲(KNOWN_FREE)
- `updateGridCounter()` — 当子栅格类型变化时递增/递减计数器
- `triggerJumpingEdge()` — 纯虚函数；当计数器地图栅格在类型间转换时调用

**膨胀地图 InfMap（inf_map.h/.cpp）**
继承自 CounterMap。维护球面膨胀占用和可选的未知膨胀：
- `imd_.occ_inflate_cnt` — 球面邻域中被占用的邻居数量
- `imd_.unk_inflate_cnt` — 球面邻域中未知的邻居数量（仅在启用 `unk_inflation_en` 时）
- `updateInflation()` — 当栅格变为/不再为占用时，对球面邻域内所有栅格递增/递减膨胀计数器
- `updateUnkInflation()` — 对未知膨胀同理

**ESDF 地图 ESDFMap（esdf_map.h/.cpp）**
继承自 CounterMap。使用独立扫描抛物线算法进行增量欧几里得符号距离场计算：
- 6 遍 1D DP 算法（`fillESDF()`）— 沿每个轴扫描以计算平方欧几里得距离变换，然后开方得到最终距离
- `updateESDF3D()` — 计算正向距离变换（到最近障碍物的距离）和负向距离变换（从障碍物内部到边界的距离），合并为有符号距离场
- 三线性插值用于距离查询 `evaluateEDT()` 和梯度查询 `evaluateFirstGrad()`

**前沿计数地图 FreeCntMap（free_cnt_map.h）**
统计每个栅格的已知空闲邻居数（3x3x3 立方体）。若栅格为未知(UNKNOWN)但存在已知空闲邻居，则该栅格为前沿(FRONTIER)——即探索边界。

**ROGMap（rog_map.h/.cpp）**
顶层公开 API，组合 ProbMap + ROS 接口：
- `updateMap(cloud, pose)` — 手动更新（当 `ros_callback.enable: false` 时使用）
- `isLineFree()` — 基于射线投射的碰撞检测（3 个重载版本）
- ROS 回调：`odomCallback()` 订阅里程计，`cloudCallback()` 订阅点云，`updateCallback()` 处理数据
- `vizCallback()` — 发布可视化点云和标记数组
- 通过 `VizConfig` 支持动态重配置

### 工具类

- **RayCaster（raycaster.h/.cpp）** — 3D DDA（数字微分分析器），用于沿射线遍历体素
- **TimeConsuming（scope_timer.hpp）** — RAII 计时器，自动以 ns/µs/ms/s 自适应单位输出耗时
- **Color + Visualization（visual_utils.hpp）** — RViz 标记辅助函数（包围盒、文本、点）
- **common_lib.hpp** — 类型别名（`Vec3f`、`Vec3i`、`Quatf`、`Pose`）、栅格类型枚举、数学工具函数

### 配置参数（config.hpp）

`Config` 类从 ROS 参数服务器的 `rog_map` 命名空间加载所有参数。关键参数：

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `resolution` | 0.1 | 基础概率地图分辨率（米） |
| `inflation_resolution` | 0.1 | 膨胀地图分辨率（米） |
| `inflation_step` | 1 | 球面膨胀半径（以栅格数为单位） |
| `map_size` | [10,10,0] | 局部地图尺寸（米） |
| `map_sliding.enable` | true | 启用零拷贝地图滑动 |
| `map_sliding.threshold` | -1.0 | 滑动触发距离（米） |
| `raycasting.enable` | true | 启用概率射线投射 |
| `raycasting.ray_range` | [0.3,10] | 射线投射最小/最大距离（米） |
| `raycasting.batch_update_size` | 1 | 累积 N 帧后批量应用更新 |
| `raycasting.p_hit/p_miss/p_min/p_max/p_occ/p_free` | 0.7/0.7/0.12/0.97/0.8/0.3 | 对数几率更新参数 |
| `esdf.enable` | false | 启用 ESDF 计算 |
| `frontier_extraction_en` | false | 启用前沿检测 |
| `virtual_ceil/ground_height` | -0.1 | 虚拟天花板/地板高度（超出视为占用） |

## 目录结构

```
ROG-Map/
├── rog_map/                        # 核心库
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/
│   │   ├── rog_map/
│   │   │   ├── rog_map.h           # 公开 API + ROS 接口
│   │   │   ├── prob_map.h          # 概率栅格地图
│   │   │   ├── inf_map.h           # 膨胀地图
│   │   │   ├── esdf_map.h          # ESDF 地图
│   │   │   ├── free_cnt_map.h      # 前沿计数器地图
│   │   │   └── rog_map_core/
│   │   │       ├── config.hpp      # 参数加载
│   │   │       ├── sliding_map.h   # 核心滑动地图（抽象基类）
│   │   │       └── counter_map.h   # 基于计数器的地图（抽象基类）
│   │   └── utils/
│   │       ├── common_lib.hpp      # 类型定义、枚举、数学函数
│   │       ├── raycaster.h/.cpp    # 3D DDA 射线遍历
│   │       ├── scope_timer.hpp     # RAII 性能计时器
│   │       ├── visual_utils.hpp    # RViz 可视化辅助函数
│   │       └── backward.hpp        # 堆栈回溯库
│   ├── src/rog_map/               # 实现文件
│   ├── config/visualization.cfg   # 动态重配置
│   └── log/plot_performance_log.py
├── examples/
│   ├── rog_map_example/           # 基础使用示例
│   │   ├── Apps/                  # marsim、astar、rrt 示例节点
│   │   ├── config/                # YAML 参数预设文件
│   │   └── launch/                # ROS launch 文件
│   └── MARSIM/                    # 完整的 MARSIM 仿真器集成
├── scripts/                       # check_version.sh、call_pos_cmd.sh
└── misc/                          # README 中的图片和 Logo
```

## 代码规范

- **命名空间:** `rog_map`
- **命名风格:** 方法/变量使用 snake_case，类名使用 PascalCase
- **类型别名:** `Vec3f` = `Eigen::Matrix<double, 3, 1>`，`Vec3i` = `Eigen::Matrix<int, 3, 1>`，`Pose` = `pair<Vec3f, Quatf>`
- **常量:** 枚举值使用大写（`OCCUPIED`、`KNOWN_FREE`、`UNKNOWN`、`FRONTIER`）
- **互斥锁模式:** `rc_.updete_lock`（注意：原代码中 "update" 拼写为 "updete"）用于保护 ROS 回调数据交换
- **调试宏:** `COUNTER_MAP_DEBUG`、`ESDF_MAP_DEBUG`、`ORIGIN_AT_CORNER`/`ORIGIN_AT_CENTER`
- **日志:** 通过 `RED`、`GREEN`、`YELLOW`、`BLUE` ANSI 转义码进行彩色输出
- `map_sliding_en: false` 表示地图固定在 `fix_map_origin` — 用于 PCD 文件加载示例

---

## 性能分析与加速方案

> **背景**: 将 ROG-Map 移植到导航项目，上位机为 Intel CPU + 集成显卡，需要评估最佳加速策略。

### 1. 计算负载画像

基于 marsim_example.yaml 典型配置（`map_size: [50, 50, 6]`, `resolution: 0.1m`, `inflation_resolution: 0.2m`, `point_filt_num: 15`, `ray_range: [0, 5]`）：

#### 1.1 数据规模

| 层级 | 地图尺寸 (voxel) | 体素总数 | 数据类型 |
|------|------------------|----------|---------|
| ProbMap | [501, 501, 61] | ~15.3M | float (4B) → **~60MB** |
| InfMap | [255, 255, 35] | ~2.3M | int16 × 2 (occ+unk) → **~9MB** |
| CounterMap (ESDF) | [255, 255, 35] | ~2.3M | int16 × 2 + double → **~27MB** |
| FreeCntMap | [503, 503, 63] | ~15.9M | int16 → **~32MB** |
| ESDF distance_buffer | ~2.3M | ~2.3M | double (8B) → **~18MB** |
| **总计** | | | **~146MB** |

#### 1.2 每帧计算量分析

以 Ouster OS1-128 LiDAR（~130k 点/帧）为例，`point_filt_num=15` 降采样后每帧约 **8,700 个有效点**：

| 模块 | 单帧操作数 | 操作类型 | 内存访问模式 | 耗时占比（估算） |
|------|----------|---------|-------------|----------------|
| **RaycastProcess** (射线投射) | 8,700点 × ~50步 = ~435k 体素访问 | DDA 步进，分支密集 | 沿射线随机访问 | **~40%** |
| **ProbMapFromCache** (概率更新) | ~435k 体素 × 对数几率更新 | 浮点加减 + 比较 | 与射线访问相同 | ~5% |
| **InfMap::updateInflation** (膨胀) | ~N个状态变化 × ~7邻域 | 整数加减 (原子性要求) | 球面邻域随机访问 | **~15%** |
| **ESDF::updateESDF3D** (距离场) | ~75k 体素 × 6遍 = 450k 次处理 | fillESDF 1D DP | 沿轴扫描，缓存友好 | **~25%** |
| **FreeCntMap** (前沿计数) | ~N个状态变化 × 27邻域 | 整数加减 | 3×3×3立方体 | ~5% |
| **VizCallback** (可视化) | 扫描 15M 体素→提取子集 | 类型比较 | 连续扫描 | ~10% |
| **mapSliding** (地图滑动) | 偶发（移动 0.3m 触发一次） | 内存清零 | 批量 FILL | 可忽略 |

### 2. 加速方案对比分析

#### 2.1 方案 A: CPU SIMD (SSE4.2/AVX2)

**原理**: 利用 Intel CPU 的单指令多数据指令集，同时对 4/8 个 float/int 做运算。

| 优点 | 缺点 |
|------|------|
| Eigen 库已自动利用 SIMD（`Vec3f` 运算等） | Raycaster DDA 是分支密集型，SIMD 无法加速 |
| `std::fill`/`memset` 已被编译器优化为 SIMD | 随机内存访问使 SIMD 收益有限 |
| 零额外依赖 | 膨胀/ESDF 的索引计算无法向量化 |

**适用模块**:
- ❌ Raycaster: DDA 步进逻辑是数据依赖的串行分支，SIMD 无法加速
- ✅ ESDF fillESDF: 理论上 1D DP 可以 SIMD 化，但抛物线算法（while 循环找交点）破坏了向量化
- ✅ Viz/boxSearch: 类型比较 + 坐标转换可用 SIMD 批量处理
- ❌ InfMap: 球面邻域遍历是标量索引操作

**结论**: 边际收益。Eigen 已自动做了能做的那部分。专门做 SIMD 手写优化的投入产出比极低。

**评分**: ⭐⭐☆☆☆ (2/5)

---

#### 2.2 方案 B: CPU 多线程并行 (OpenMP / TBB)

**原理**: 利用 OpenMP 将独立任务分布到多个 CPU 核心并行执行。

| 优点 | 缺点 |
|------|------|
| Intel CPU 通常有多核心（4-16核） | 需要处理线程安全（锁/原子/归约） |
| 实现简单：`#pragma omp parallel for` | ROS 回调线程 + OMP 线程可能超订 |
| 数据已在主内存，无需搬运 | 膨胀更新可能产生竞争写入 |
| 大部分内核是天然并行的 | 小任务并行化的额外开销可能超过收益 |

**逐模块分析**:

| 模块 | 并行策略 | 并行度 | 竞争风险 | 预期加速 |
|------|---------|--------|---------|---------|
| **RaycastProcess** | `parallel for` over points | 高（~8700 独立任务） | 无。每个点在独立射线上遍历 | **3-6×** |
| **InfMap updateInflation** | `parallel for` over cells | 中（每帧~数百个状态变化） | ⚠️ 膨胀计数器竞争：多个相邻栅格的球面邻域可能重叠 | **1.5-2×**（需原子操作） |
| **ESDF fillESDF 扫描** | `parallel for` over 2D slices | 高（50×33=1650 slices/遍） | 无。每层 slice 独立 | **3-5×** |
| **Viz boxSearch** | `parallel for` over Z轴切片 | 高（30-60 slices） | 无。各 slice 结果最后合并 | **2-4×** |
| **ProbMapFromCache** | 基于队列串行 | 低（队列需要顺序消费） | 低。可改用 per-bucket 队列 | 1.1× |

**OMP 实现要点**:

```cpp
// Raycaster: 最有效的并行点
#pragma omp parallel for schedule(dynamic, 64)
for (const auto& pcl_p : input_cloud) { ... }

// ESDF: 对 2D slice 并行
#pragma omp parallel for collapse(2)
for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
        fillESDF(...);

// InfMap: 使用原子加法避免竞争
#pragma omp parallel for
for (const auto& nei : spherical_neighbor) {
    #pragma omp atomic
    imd_.occ_inflate_cnt[addr]++;
}
```

**结论**: **最推荐的方案**。实现成本低，加速效果显著，不引入新硬件依赖。

**评分**: ⭐⭐⭐⭐⭐ (5/5)

---

#### 2.3 方案 C: Intel 集成显卡 GPU (OpenCL / oneAPI Level Zero / SYCL)

**原理**: 利用 Intel iGPU（UHD/Iris Xe）的并行计算单元进行大规模并行运算。

| 优点 | 缺点 |
|------|------|
| 集成显卡有数十个 EU（执行单元） | 数据搬运开销大（CPU↔GPU，~146MB 地图数据） |
| 天然适合结构化网格运算（如 ESDF） | Intel iGPU 算力有限（一般 <1 TFLOPS FP32） |
| 可卸载部分计算，释放 CPU | 分支密集任务（DDA）在 GPU 上性能极差 |
| | 调试困难，需要额外的 GPU profiling 工具 |
| | ROS 生态对 GPU 支持弱（没有现成的 GPU ROS 消息） |
| | 每个 Compute Unit 只有 64KB 本地内存，地图太大放不下 |

**逐模块分析**:

| 模块 | GPU 适配性 | 搬运开销 | 评估 |
|------|-----------|---------|------|
| **Raycaster** (DDA) | ❌ 极差。DDA 是分支密集 + 串行步进，GPU warp divergence 严重 | 低 | 不可行 |
| **ESDF fillESDF** (1D DP) | ⚠️ 中等。1D DP 的每步依赖前一步结果，不能完全并行 | 高（需搬运整个 update region） | 理论上可做但收益小 |
| **InfMap updateInflation** | ❌ 极差。在球面邻域做原子加法，GPU 原子操作效率低 | 高 | 不可行 |
| **Viz boxSearch** | ✅ 良好。对每个体素独立判断类型 | 极高（15M 体素 × 坐标信息） | 搬运开销可能超过计算收益 |
| **概率更新** | ❌ 极差。队列消费 + 单值更新 | 高 | 不可行 |

**数据搬运分析**（以 ESDF 为例）:
- 假设 iGPU 有 64 个 EU @ 1.1GHz
- 需要搬运 ESDF 更新区域：`tmp_buffer1_`(2.3M×8B) + `tmp_buffer2_`(2.3M×8B) + `distance_buffer`(2.3M×8B) ≈ **55MB**
- PCIe 带宽（CPU→iGPU 通过系统内存，实际共享内存带宽 ~30-50GB/s）：搬运时间 ~1-2ms
- ESDF 6 遍计算在 iGPU 上：2.3M × 6 遍 ≈ 14M 次处理，每 EU ~220k 次 → **估计 ~3-5ms**
- 加上启动/同步延迟，GPU 端 ESDF 总时间 ≈ **5-8ms**
- CPU 多线程版（4 核）ESDF 耗时 ≈ **2-4ms**

**结论**: 对 Intel iGPU，数据搬运 + 启动开销通常超过计算收益。ROG-Map 的计算模式以不规则内存访问 + 串行分支为主，不是 GPU 的理想负载。

**评分**: ⭐☆☆☆☆ (1/5) — 对于此项目不推荐

---

#### 2.4 方案 D: 混合加速（CPU 多线程 + 算法级优化）

这是针对此项目最合理的方案：**以 OpenMP 多线程为主 + 针对性算法优化**。

**推荐实施优先级**:

##### Tier 1: 立即见效（低风险，高收益）

| 优化项 | 改动位置 | 预期收益 | 风险 |
|--------|---------|---------|------|
| **RaycastProcess OMP 并行** | `prob_map.cpp:662-735` | **-30%~-40%** 耗时 | 极低 |
| **ESDF fillESDF OMP 并行** | `esdf_map.cpp:129-218`（6 处循环） | **-20%~-30%** 耗时 | 低 |
| **boxSearch OMP 并行** | `prob_map.cpp:418-486` | **-5%~-10%** 耗时 | 极低 |
| **VizCallback OMP 并行 + 懒发布** | `rog_map.cpp:349-480` | **-5%** 耗时 | 低 |

##### Tier 2: 中等投入（需要小的重构）

| 优化项 | 说明 | 预期收益 |
|--------|------|---------|
| **InfMap 原子计数器替换** | 用 `std::atomic<int16_t>` 替换 `int16_t` 膨胀计数器，允许 OMP 并行 | **-10%~-15%** |
| **射线缓存预热** | 相邻帧的点云有很大重叠度，缓存上帧的射线结果避免重复穿越 | **-10%~-20%**（取决于运动速度） |
| **降采样前置** | 直接用 PCL 的 `VoxelGrid` 做空间降采样，而非按顺序每隔 N 取一 | 均匀覆盖，提高地图质量 |
| **日志 I/O 分离** | 将 `writeTimeConsumingToLog` 放入独立线程，避免阻塞主循环 | 减少主循环 jitter |

##### Tier 3: 如仍有瓶颈（大改动）

| 优化项 | 说明 |
|--------|------|
| **ESDF 增量范围缩减** | 跟踪实际变化的栅格列表，只对这些栅格周围的局部区域重算 ESDF（而非固定大小的 box） |
| **概率地图压缩** | 将 `float` 改为 `int16_t`（对数几率范围有限），内存减半 → Cache Miss 减半 |
| **空间哈希替换队列** | `update_cache_id_g` 使用并发无锁哈希表替代 `std::queue` + `operation_cnt` 数组 |

##### 不需要优化的部分

- ❌ **mapSliding**: 偶发执行（机器人移动 0.3m 触发一次），单次耗时 <1ms
- ❌ **ProbMapFromCache (概率应用)**: `while (!queue.empty())` 没有计算密集的内核
- ❌ **FreeCntMap**: 3×3×3 乘法距离，开销极小

### 3. 最终推荐方案

```
┌─────────────────────────────────────────────────────────┐
│  推荐策略: 纯 CPU 多线程 (OpenMP)                          │
│                                                           │
│  ● 不推荐 GPU / SIMD                                      │
│    - Intel iGPU 算力不足以抵消数据搬运开销                   │
│    - Raycaster DDA 是分支密集型，GPU warp divergence 严重    │
│    - SIMD 对分支+随机访问负载的加速效果有限                   │
│                                                           │
│  ● OpenMP 多线程可以有效加速:                               │
│    ✓ RaycastProcess (并行 per-point，独立性强)              │
│    ✓ ESDF fillESDF (并行 per-slice，2D独立)                │
│    ✓ Viz/boxSearch (并行 per-slice)                       │
│    △ InfMap (可用原子操作，收益中等)                        │
│                                                           │
│  ● 预估总体加速比: 2.5× ~ 4× (4核~8核 Intel CPU)           │
│  ● 实现代价: ~200行 OpenMP pragma + 少量 atomic 类型替换    │
│  ● 额外收益: OpenMP 运行时开销小于 1ms，延迟几乎无影响        │
└─────────────────────────────────────────────────────────┘
```

### 4. 实现注意事项

1. **不要超订线程**: ROS spinner 已占用 1-N 线程，OpenMP 默认使用全部核心。在 ROS launch 中设 `OMP_NUM_THREADS=cores-2` 或代码中 `omp_set_num_threads()`。

2. **InfMap 竞争处理**: `occ_inflate_cnt` 改为 `std::atomic<int16_t>` 即可用 `fetch_add(1)` 保证竞争安全，无锁开销 <10ns/次。

3. **Raycaster 线程安全**: `RayCaster::step()` 不是线程安全的（修改内部状态）。每个线程需要自己的 `RayCaster` 实例，最简单的方式是让 `raycast_data_.raycaster` 变为 `thread_local` 或者在 lambda 内部创建临时实例。

4. **ESDF 回绕处理**: 循环缓冲区的 `mem_end` 计算在 OMP 并行下不受影响，因为只读不写。

5. **测量验证**: 用项目自带的 `TimeConsuming` 计时器测量优化前后各模块耗时。`log/plot_performance_log.py` 可以画出优化前后对比。

6. **Intel CPU 特性利用**:
   - 现代 Intel CPU 有 AVX-512（512bit 寄存器），Eigen 3.4+ 支持自动使用
   - Intel oneTBB 可作为 OpenMP 的替代品（更适合嵌套并行和不规则任务）
   - `-march=native` 编译选项让编译器生成针对本机 CPU 的最优指令
