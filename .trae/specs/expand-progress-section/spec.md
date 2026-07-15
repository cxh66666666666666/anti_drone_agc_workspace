# PPT 修改 Spec：把 09「项目当前进展与在研工作」拆为多页

## Why
用户反馈 09 那一页塞了 4 个区块太挤，需要拆成多页让每页讲清一件事：
- 09 当前系统介绍
- 10 本次实验数据
- 11 当前在研工作
- 12 演示材料（图片/视频占位）
- 13 总结与 Q&A

## What Changes
- 删除：原 `slide_10_progress`（4 象限挤在一页的版本）
- 新增 4 个 slide 函数：
  - `slide_10_current_system` — 09 当前系统介绍（展开为整页）
  - `slide_11_csv_data` — 10 本次实验数据
  - `slide_12_in_progress` — 11 当前在研工作
  - `slide_13_demo_placeholders` — 12 演示材料（图片/视频占位）
- 收尾页 `slide_11_summary` 重命名为 `slide_14_summary`，编号改为「13 总结与 Q&A」
- `TOTAL_PAGES` 由 11 改为 14
- `compute_metrics_from_csv` 增加：从 CSV 提取首末位姿、估算路径长度，扩展数据展示维度

## Impact
- Affected specs: `update-ppt-progress-page`（上一轮基线）
- Affected code: `scripts/generate_pptx.py`
- Affected output: `docs/指导书/项目答辩PPT.pptx`

## ADDED Requirements

### Requirement: 新页面顺序（14 页）
1. 封面
2. 01 项目背景
3. 02 研究意义
4. 03 国内外研究现状
5. 04 项目总体目标
6. 05 系统架构
7. 06 强化学习建模
8. 07 场景设计
9. 08 训练与验证流程
10. **09 项目当前进展与在研工作 · 当前系统介绍**（新）
11. **10 项目当前进展与在研工作 · 本次实验数据**（新）
12. **11 项目当前进展与在研工作 · 当前在研工作**（新）
13. **12 项目当前进展与在研工作 · 演示材料占位**（新）
14. 13 总结与 Q&A（重编号）

**WHEN** 生成完成，**THEN** PPT 恰好为 14 页。

### Requirement: 第 10 页「09 当前系统介绍」内容
- 标题：「09  项目当前进展与在研工作 · 当前系统介绍」
- 副标题一行：「Gazebo + PX4 SITL 上的 4 节点仿真系统」
- 主体分两栏：

**左栏 — 系统组件**（垂直列表，每项 1 行加大字号）：
- 仿真平台：Gazebo + PX4 SITL + MAVROS
- 跟踪层：LQR（已实现 8 字形轨迹可跟踪）
- 感知层：perception_node（12 维扇区特征 + 目标向量 + 速度）
- 规划层：rl_planner_node（启发式 6 动作避障）
- 日志层：experiment_logger_node（10 Hz 写 CSV）
- 规则式避障已能稳定工作

**右栏 — ROS 通信链**（从下到上画箭头链 + 文字说明）：
- /velodyne_points （激光雷达点云）
- /mavros/local_position/pose （位姿）
- ↓
- perception_node → /obstacle_features
- ↓
- rl_planner_node → /rl_planner/local_goal + /rl_planner/action
- ↓
- lqr_tracker_node → /mavros/setpoint_position/local
- ↓
- PX4 Offboard 控制 → Gazebo

**WHEN** 生成完成，**THEN** 第 10 页文字总字数不超过 200 字。

### Requirement: 第 11 页「10 本次实验数据」内容
- 标题：「10  项目当前进展与在研工作 · 本次实验数据」
- 副标题：「数据源：experiment_data/experiment_20260708_185400.csv · 实时统计」
- 5 个统计卡片（每卡片 1 数值 + 1 单位 + 1 说明）：
  1. 运行时长 81.7 s
  2. 记录条数 818 条
  3. 避障激活 35.7 %
  4. 位置误差均值 0.794 m
  5. 动作次数（前进 526 / 左移 154 / 上升 138）
- 动作分布柱状图（纯黑线段，不用彩色）：
  - 前进 64.3% —— 黑色长条
  - 左移 18.8% —— 黑色中条
  - 上升 16.9% —— 黑色短条
  - 右移 / 下降 / 悬停 —— 0（不画条）
- 位置误差说明一行：「均值 0.794 m，中位 0.676 m，最大 5.107 m」

**WHEN** 生成完成，**THEN** 第 11 页所有数值来自 CSV 实时统计。

### Requirement: 第 12 页「11 当前在研工作」内容
- 标题：「11  项目当前进展与在研工作 · 当前在研工作」
- 3 个独立大卡片，垂直堆叠：

**卡片 1：多难度场景指标记录**
- 目标：覆盖 S0~S3 全场景
- 计划：每场景 5 颗随机种子 × 多回合
- 输出：成功率 / 碰撞率 / 路径长度 / 规划时间 / 平滑度

**卡片 2：DQN 实现（补全 dqn_agent.py）**
- 目标：从空文件 → 完整 Q 学习 agent
- 组件：Q 网络 + 目标网络 + replay buffer + epsilon-greedy
- 状态 20 维 / 动作 6 维 / 奖励塑形

**卡片 3：训练与对比**
- 目标：DQN vs 当前规则式基线对比
- 下一步：加入 A* 2D 栅格基线
- 输出：训练曲线 + 对比表

**WHEN** 生成完成，**THEN** 第 12 页包含 3 个独立大卡片，标题与目标明确。

### Requirement: 第 13 页「12 演示材料」内容
- 标题：「12  项目当前进展与在研工作 · 演示材料占位」
- 副标题：「待实验录制 / 截图后插入」
- 4 个占位框 2×2 布局：
  - [图] Gazebo 仿真场景  3.0" × 2.0"
  - [图] RViz 轨迹与障碍  3.0" × 2.0"
  - [视频] 规则式避障飞行回放  3.0" × 2.0"
  - [图] 实验数据可视化  3.0" × 2.0"
- 每个占位框：黑线 1.5pt 矩形 + 中央灰文字 + 下方 11pt 灰说明
- 页底：「提示：占位框可在 PowerPoint 中右键替换为图片/视频」

**WHEN** 生成完成，**THEN** 第 13 页有 4 个 2×2 排列的占位框。

### Requirement: 第 14 页「13 总结与 Q&A」内容
- 标题：「13  总结与 Q & A」
- 主内容：「敬请老师批评指正」36pt 加粗居中
- 小组信息：第 12 组 / 智科 1 班 / 4 位组员 / 指导教师叶景贞
- 保留页脚

**WHEN** 生成完成，**THEN** 第 14 页即原收尾页内容。

### Requirement: 视觉风格不变
- 配色：黑/白/灰阶
- 动作分布柱状图：用矩形条（黑底白字）表示百分比宽度
- 占位框：黑线矩形 + 灰文字
- 每页均带页脚（左下组别小字 + 右下「N / 14」）

### Requirement: 编号一致性
- 页眉标题编号 01~13 连续
- 页脚页码 N / 14（N = 1..14）
- 4 个「项目当前进展与在研工作」子页使用「09·」「10·」「11·」「12·」前缀以保留「09 父主题」关联

## MODIFIED Requirements

### Requirement: 总页数
11 → **14**

### Requirement: 标题编号系统
原 09（项目当前进展）→ 拆为 09 / 10 / 11 / 12 四个子页

## REMOVED Requirements

### Requirement: 原 `slide_10_progress`（4 象限挤一页版本）
**Reason**: 用户要求拆开
**Migration**: 拆为 4 个独立 slide 函数（slide_10_current_system / slide_11_csv_data / slide_12_in_progress / slide_13_demo_placeholders）
