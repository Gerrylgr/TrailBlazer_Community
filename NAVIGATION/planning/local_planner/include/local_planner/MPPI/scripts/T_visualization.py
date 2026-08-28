import pandas as pd
import numpy as np

df = pd.read_csv("/tmp/mppi_weights_2.600000.csv", comment="#")

# 只取每个 cycle 的最后一次迭代（它决定最终输出）
df = df[df.iteration == df.groupby("cycle").iteration.transform("max")].copy()

# 1. 丢掉暂态：reset 后前 20 个 cycle 的分布不代表稳态
df = df[df.cycle > df.cycle.min() + 20]

def ess(w):
    s = w.sum()
    return s * s / (w * w).sum() if s > 0 else 0.0

stats = df.groupby("cycle").apply(lambda g: pd.Series({
    "delta_p50": g.normalized_cost.quantile(0.50),
    "delta_p90": g.normalized_cost.quantile(0.90),
    "w_max":     g.normalized_weight.max(),                 # normalized_weight 中的最大值
    "ess":       ess(g.weight.to_numpy()),
}), include_groups=False)

print(stats.describe().round(3))

# 2. 顺手画一条曲线，能直观看到每次新 goal 的暂态段和稳态段
stats.delta_p50.plot()   # stats 是 groupby 后的结果

