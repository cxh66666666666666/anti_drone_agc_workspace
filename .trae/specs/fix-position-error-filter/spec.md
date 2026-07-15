# PPT 修改 Spec：位置误差过滤（排除起飞 + 避障）

## Why
用户反馈：当前 PPT 第 11 页的「位置误差」统计包含了：
1. 起飞到达目标点的过程（应跳过）
2. 避障导致的偏移（应跳过）

需要重新过滤 CSV 数据，让「位置误差」只反映**稳态飞行且非避障**期间的 LQR 跟踪精度。

## What Changes
- **修改** `compute_metrics_from_csv` 函数：增加过滤逻辑
  - 过滤 A：跳过「起飞到达目标点」阶段（首次 position_error < 1.0 m 之前的所有行）
  - 过滤 B：跳过 is_avoiding=1 的所有行
- **修改** PPT 第 11 页「10 本次实验数据」：
  - 「位置误差」卡片：均值/中位/最大 重算（基于过滤后数据）
  - 「位置误差详情」一行：显示新数值 + 过滤后样本数 + 「已排除起飞与避障」说明
- **不修改** 其他卡片（运行时长 / 记录条数 / 避障激活 / 前进次数）
- **不修改** 动作分布柱状图

## Impact
- Affected specs: `expand-progress-section`（上一轮基线）
- Affected code: `scripts/generate_pptx.py`
- Affected output: `docs/指导书/项目答辩PPT.pptx`

## ADDED Requirements

### Requirement: CSV 过滤规则
**过滤 A（起飞阶段）**：
- 找到首次 `position_error` < 1.0 m 且非 NaN 的行索引 `first_reach_idx`
- 保留 `idx >= first_reach_idx` 的所有行
- 若数据中从未出现 error<1.0，则不做过滤 A

**过滤 B（避障阶段）**：
- 保留 `is_avoiding == 0` 的所有行
- 应用过滤 A 之后

**过滤 C（NaN）**：
- 保留 `position_error` 非 NaN 的行

**WHEN** `compute_metrics_from_csv(path)` 被调用，**THEN** 返回的 `pe_mean` / `pe_median` / `pe_max` 必须基于「A + B + C」过滤后的样本集。

### Requirement: 新增字段
`compute_metrics_from_csv` 返回字典新增字段：
- `pe_filtered_n`（int）：过滤后样本数
- `pe_filtered_dropped_takeoff`（int）：被过滤 A 跳过的样本数
- `pe_filtered_dropped_avoiding`（int）：被过滤 B 跳过的样本数
- `pe_filtered_dropped_nan`（int）：被过滤 C 跳过的样本数
- `first_reach_time`（float 或 None）：首次达到目标的时刻（s），用于 PPT 显示

### Requirement: PPT 第 11 页「位置误差」卡片更新
- 卡片数值（来自新过滤）：
  - 均值 ≈ 0.679 m（原 0.794 m）
  - 中位 ≈ 0.674 m（原 0.676 m）
  - 最大 ≈ 0.883 m（原 5.107 m）
  - 样本数 = 447（原 818）
- 卡片说明文字（每张卡片底部）更新为「稳态飞行 · 非避障期」

### Requirement: PPT 第 11 页「位置误差详情」一行更新
- 新内容：`均值 0.679 m  ·  中位 0.674 m  ·  最大 0.883 m  ·  样本 447 条（已排除 371 条：起步 26 + 避障 126 + NaN 219）`
- 数据必须由脚本实时从 CSV 计算，不写死
- 详细过滤条件用脚注或小字呈现

### Requirement: 不修改的项
- 5 卡片中其他 4 张（运行时长 / 记录条数 / 避障激活 / 前进次数）保持原值
- 动作分布柱状图保持原数据
- 其他 13 页内容不变
- 视觉风格（黑/白/灰阶）不变

### Requirement: 落地阈值可解释
「首次达到目标」的阈值 1.0 m 是合理的工程近似：
- LQR 跟踪稳态误差量级在 0.5 ~ 0.9 m
- 1.0 m 大于稳态抖动且小于起飞阶段典型误差
- 阈值在脚本顶部以常量定义，便于未来调整

## MODIFIED Requirements

### Requirement: 位置误差统计
原 spec：`pe_mean` / `pe_median` / `pe_max` 基于全量 CSV（含起飞 / 避障 / NaN）
**修改为**：基于「稳态 + 非避障 + 非 NaN」过滤后样本

## REMOVED Requirements

（无）
