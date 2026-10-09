# N5 单体性能边界深化 Spec（change-id: deepen-n5-single-agent-boundaries）

## Why

M7（`docs/项目任务日志/阶段1/M7/M7_单体性能与三维避障边界验收_v1.md`）已完成单体评估（`verify_m7` PASS=32/FAIL=0），但留下四点未决，N5 逐条量化：

1. **无低矮屏障上越证据**：M7 的 51 个竖直动作回合仅证明「未发散」，且 v4 障碍全部贯穿高度（`center.z=2.0,size.z=2.8`），不存在「顶面低于飞行高度、可上越」的屏障。
2. **实时 LQR 竖直偏移无端到端专项**：M7 仅以「LQR 代码支持竖直叠加 + RL 回合统计」佐证三维动作可执行。
3. **地面 RL 样本与布局单一**：仅 200 episode，唯一 `ground.world` 三杆固定，val 4 / test 6，且只对初始位姿划分 train/val/test，障碍场不变。
4. **LQR 误差未分稳态**：`linear` P95=8.256 m 含起步/换段瞬态，M7 未做瞬态剔除口径。

## What Changes

- **A 低矮屏障上越 vs 水平绕行对照（无人机 RL 单体）**：新增两组**固定、可哈希**的屏障场景（仅屏障高度不同，其余逐字段一致）：`low_barrier`（顶面低于飞行高度 → 上越可行）与 `tall_barrier`（贯穿 → 只能水平绕行）。评估**既有** DQN（不重训），记录 3D 轨迹、净空、竖直动作率、成功率、碰撞与失败分母，并给出「上越 / 绕行」判据对照。
- **B 实时 LQR 竖直参考专项（AGC）**：在 `lqr_tracker_node` 唯一控制权下，向 `/rl_planner/local_offset` 注入**竖直**偏移，核验竖直分量真实进入位置设定点、无人机跟随到新高度、注入清零后恢复。
- **C 地面 RL 障碍布局扩展与训练/验证/测试隔离**：使地面环境的障碍布局（`poles`）**可按场景注入**；扩展为多布局，按**布局**隔离 train/val/test（test 用**未见布局**）；短训 DQN 后评估，报告失败分母、碰撞、净空与轨迹。
- **D LQR 全程与明确窗口稳态误差（AGC）**：在 M7 全段统计之外，报告**剔除起步与本段换段**的明确稳态窗口误差，明确窗口边界依据。
- 不新增/改动空地协同契约话题、判据与坐标/时间基准；全部 run 未出现 `INTERCEPT_CONFIRMED`，`SUCCESS`= 抵近/通过，不宣称拦截完成。**BREAKING**：无。

## Impact

- Affected specs：M7 验收的未决 1–4；M0 接口契约 v1.2（不涉话题/判据变更）。
- Affected code（**实时链路不改**；新增/改动均为离线实验工具 Python 与 RL 环境）：
  - `src/rl_obstacle_avoidance/scripts/n5_barrier_scenarios.py`（新）；复用 `m7_drone_eval.py`、`run_m7_drone_evidence.py`
  - `scripts/n5_lqr_vertical_inject.py`、`scripts/n5_lqr_vertical_eval.py`（新）
  - `src/rl_ground_avoidance/scripts/n5_ground_scenarios.py`、`n5_ground_world_gen.py`（新）；`ground_env.py` / `m7_ground_train_eval.py`（小幅：布局注入）
  - `scripts/n5_lqr_window_eval.py`（新，复用 `m7_lqr_tracking_eval.py` 的轨迹矩阵）
  - `scripts/verify_n5.py`（新，A–D 四段核验）

## ADDED Requirements

### Requirement: 低矮屏障上越与水平绕行对照

系统 SHALL 在**仅屏障高度不同**的两组固定场景上评估同一无人机策略，并 SHALL 从 3D 轨迹给出「上越」与「绕行」的可核验判据，同时保留失败分母。

#### Scenario: 低矮屏障组
- **WHEN** 屏障顶面 < 飞行高度且横向跨航迹
- **THEN** 记录 3D 轨迹、净空、竖直动作率、成功率/碰撞/超时/失败分母；判定是否存在「越过屏障顶面（`pose_z > top + margin` 且 x 越过屏障位）」的上越样本
- **AND** 轨迹与逐回合结局落盘（`trajectories/*.jsonl`、`episodes.jsonl`、`metrics.json`）

#### Scenario: 高屏障组（对照）
- **WHEN** 同布局但屏障贯穿高度（顶面 ≥ 飞行高度）
- **THEN** 上越不可行，仅水平绕行；记录同口径指标与轨迹
- **AND** 与低矮屏障组按同一口径并列对照

### Requirement: 实时 LQR 竖直参考专项

系统 SHALL 在 AGC 实时链路（`lqr_tracker_node` 唯一 setpoint 发布者）下，向 `/rl_planner/local_offset` 注入竖直偏移，并核验竖直参考真实生效。

#### Scenario: 竖直偏移注入
- **WHEN** 在某固定轨迹 run 运行期，以固定频率发布含 z 分量的 `/rl_planner/local_offset`
- **THEN** 记录参考 z、无人机实际 z 与 `/lqr_tracker/position_error`；核验参考 z 随注入变化、无人机跟随至新高度、注入撤销后回到原高度
- **AND** 偏移超时（>0.5 s 无消息）自动清零的行为被如实记录

### Requirement: 地面障碍布局扩展与隔离

系统 SHALL 允许地面 RL 场景注入**障碍布局**，并 SHALL 按布局隔离 train/val/test（test 使用未见布局）。

#### Scenario: 布局注入与隔离
- **WHEN** 按新场景集训练并在 val（部分布局）/test（未见布局）上评估
- **THEN** 记录每回合结局、失败分母、碰撞、净空与轨迹；报告**布局**粒度结果，注明 test 布局未参与训练
- **AND** 匹配障碍布局的 Gazebo 世界可由生成器产出，保证 GUI 可视化与感知几何一致（或如实标注虚拟几何口径）

### Requirement: LQR 全程与稳态窗口误差

系统 SHALL 在 M7 全段统计之外，报告**明确稳态窗口**的跟踪误差，并 SHALL 写明窗口边界依据。

#### Scenario: 分窗口统计
- **WHEN** 复跑固定轨迹矩阵（linear/circular/fixed_horizontal/figure_eight）
- **THEN** 对每条轨迹同时给出全段与稳态窗口（剔除起步与本段换段）的均值/RMS/最大/P95 与样本数
- **AND** 窗口边界依据（时间或轨迹相位）在报告中明确

## MODIFIED Requirements

### Requirement: 地面 RL 场景集（M7 §2.2）

由「仅变初始位姿/参考速度、障碍场固定三杆」扩展为「多障碍布局 + 按布局隔离」。原 M7 固定集与其哈希（`869500acad8f…`）**不改**，作为历史证据保留；N5 另立新场景集与哈希。

## REMOVED Requirements

无。
