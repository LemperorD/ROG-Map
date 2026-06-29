# ROG-Map 性能日志可视化脚本
#
# 功能：读取 rm_performance_log.csv 文件（由 ROGMap 自动生成），
# 为每个计时列绘制箱线图，方便观察各步骤的耗时分布。
#
# 使用方式：
#   1. 运行 ROG-Map 节点后，会在 log/ 目录下生成 rm_performance_log.csv
#   2. cd rog_map/log && python plot_performance_log.py
#
# CSV 列顺序（对应 time_consuming_name_）：
#   Total, Raycast, Update_cache, Inflation, PointCloudNumber, CacheNumber, InflationNumber

import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
data = pd.read_csv('rm_performance_log.csv')

# 获取列标签
columns = data.columns.tolist()

# 删除第一行（可能包含标签而非数据）
data = data.iloc[1:]

# 将数据转换为数值类型
data = data.astype(float)

# 创建一个水平排列的子图，每个列一个子图
fig, axes = plt.subplots(nrows=1, ncols=len(columns), figsize=(3 * len(columns), 3))

# 为每一列绘制箱线图，并设置Y轴范围
for i, column in enumerate(columns):
    ax = axes[i]
    ax.boxplot(data[column])  # 箱线图：显示中位数、四分位数、异常值
    ax.set_xlabel(column)     # X轴标签为列名

    # 前4列（Total, Raycast, Update_cache, Inflation）使用统一的Y轴范围
    # 便于直观对比耗时差异
    if i < 4:
        ax.set_ylim([data[columns[:4]].min().min(), data[columns[:4]].max().max()])

# 设置整体标题
fig.suptitle('Box Plot of Columns')
plt.subplots_adjust(wspace=1.5)

# 调整子图之间的间距
plt.tight_layout()

# 显示图表
plt.show()
