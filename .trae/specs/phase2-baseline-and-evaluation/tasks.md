# Tasks

- [ ] Task 1: 实现 A* 路径规划基线
  - [ ] 创建 `src/rl_obstacle_avoidance/scripts/astar_planner.py`，实现 2D 占据栅格 A* 算法
  - [ ] 支持障碍物膨胀安全半径（默认 0.8m）
  - [ ] 支持路径平滑（shortcut 后处理）
  - [ ] 使用欧氏距离作为启发函数
  - [ ] 输出航点列表，供 LQR 跟踪

- [ ] Task 2: 实现 RRT 路径规划基线
  - [ ] 创建 `src/rl_obstacle_avoidance/scripts/rrt_planner.py`，实现 RRT 算法
  - [ ] 在连续空间中随机采样，向采样点扩展固定步长
  - [ ] 碰撞检测使用线段与障碍物膨胀体距离
  - [ ] 支持 shortcut 路径平滑
  - [ ] 可配置最大迭代次数和步长

- [ ] Task 3: 创建场景生成脚本
  - [ ] 创建 `src/rl_obstacle_avoidance/scripts/generate_world.py`
  - [ ] 从 YAML 配置读取障碍物参数（类型、位置、尺寸）
  - [ ] 生成符合 Gazebo SDF 格式的 world 文件
  - [ ] 支持 box 和 cylinder 两种障碍物类型

- [ ] Task 4: 创建场景配置文件
  - [ ] 创建 `src/rl_obstacle_avoidance/config/scenarios_sparse.yaml`（3-5 个障碍物）
  - [ ] 创建 `src/rl_obstacle_avoidance/config/scenarios_dense.yaml`（8-20 个障碍物）

- [ ] Task 5: 创建独立评估脚本
  - [ ] 创建 `src/rl_obstacle_avoidance/scripts/evaluate_policy.py`
  - [ ] 支持 DQN 模型加载和推理（与训练脚本复用评估逻辑）
  - [ ] 支持 A* 规划器评估
  - [ ] 支持 RRT 规划器评估
  - [ ] 支持规则基线（rl_planner_node.py）评估
  - [ ] 输出汇总指标：成功率、碰撞率、平均路径长度、平均规划时间、轨迹平滑度
  - [ ] 支持多场景多随机种子批量评估
  - [ ] 评估结果保存为 CSV 格式

- [ ] Task 6: 在稀疏障碍物场景下训练 DQN 并评估
  - [ ] 使用 `scenarios_sparse.yaml` 生成的 world 文件训练 DQN
  - [ ] 运行评估脚本对比 DQN、A*、RRT 在稀疏场景下的表现
  - [ ] 生成对比表格和图表

# Task Dependencies
- Task 3 依赖 Task 1 和 Task 2（无直接依赖，可并行）
- Task 4 依赖 Task 3（场景配置文件需要生成脚本验证）
- Task 5 依赖 Task 1、Task 2（评估脚本需要调用 A* 和 RRT 规划器）
- Task 6 依赖 Task 3、Task 4、Task 5（需要场景和评估脚本就绪）