# Tasks（change-id: deepen-n5-single-agent-boundaries）

- [x] Task 1: 低矮屏障上越 vs 水平绕行对照（无人机 RL 单体）
  - [x] SubTask 1.1: 新增 `src/rl_obstacle_avoidance/scripts/n5_barrier_scenarios.py`：两组固定场景（`low_barrier` 顶面<飞行高度 / `tall_barrier` 贯穿），除屏障高度外逐字段一致，采用 v4 场景 schema，可哈希
  - [x] SubTask 1.2: 生成 manifest（含 split 与 sha256，`.frozen`）；复用 `m7_drone_eval.py` + 既有模型 `training_runs/dqn_v5_raw_s101/best_model.pt` 评估两组
  - [x] SubTask 1.3: 新增上越/绕行判据与对照报告（轨迹越过屏障顶且 x 越过屏障位=上越；否则绕行），保留失败分母
- [x] Task 2: 实时 LQR 竖直参考专项（AGC C++ 实时链路）
  - [x] SubTask 2.1: 新增 `scripts/n5_lqr_vertical_inject.py`：运行期以固定频率发布含 z 分量的 `/rl_planner/local_offset`（含注入窗与清零），并记录注入时序
  - [x] SubTask 2.2: 新增 `scripts/n5_lqr_vertical_eval.py`：启 `start_sim.sh lqr`（fixed_horizontal），记录参考 z / 实际 z / `/lqr_tracker/position_error`，核验跟随与清零恢复，落盘 JSON
- [x] Task 3: 地面 RL 障碍布局扩展与训练/验证/测试隔离
  - [x] SubTask 3.1: 使地面障碍布局可注入：`ground_env` 读取场景 `poles`（或等价机制），`apply_scenario` 设置 `env.poles`
  - [x] SubTask 3.2: 新增 `src/rl_ground_avoidance/scripts/n5_ground_scenarios.py`：多障碍布局 + 按布局隔离 train/val/test（test 用未见布局），提供 `split_sha256`
  - [x] SubTask 3.3: 新增 `src/rl_ground_avoidance/scripts/n5_ground_world_gen.py`：按布局产出匹配 `ground.world`（GUI 可视化与感知几何一致）
  - [x] SubTask 3.4: 短训 DQN（约 300 episode）后评估 val（部分布局）/test（未见布局），落 `episodes.jsonl`/`metrics.json`/`trajectories`
- [x] Task 4: LQR 全程与稳态窗口误差（AGC）
  - [x] SubTask 4.1: 新增 `scripts/n5_lqr_window_eval.py`：复跑轨迹矩阵（linear/circular/fixed_horizontal/figure_eight），同时输出全段与稳态窗口（剔除起步/换段）统计，并写明窗口边界依据
- [x] Task 5: 核验、批跑取证与验收文档
  - [x] SubTask 5.1: 新增 `scripts/verify_n5.py`：A（屏障对照）/B（竖直参考）/C（地面隔离）/D（窗口误差）四段 PASS/FAIL
  - [x] SubTask 5.2: 运行四部分（`--gui` 非无头），锁定模型/场景/参数哈希并索引批次
  - [x] SubTask 5.3: 撰写 `docs/项目任务日志/阶段2/N5.md`（口径与诚实说明、实现、批次哈希、逐项 PASS/FAIL、交付物核对、未决）

# Task Dependencies

- Task 5 depends on Task 1、Task 2、Task 3、Task 4
- Task 1、Task 2、Task 3、Task 4 相互独立，可并行
