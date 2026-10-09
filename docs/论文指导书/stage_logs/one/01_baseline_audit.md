# days 1–3 基线审计（R0）

> 范围：对旧系统做静态读码审计，产出可解释起点。**未运行仿真、未改写任何旧结果文件**
> （不碰 `cross_validation.json`、`audit_index.*` 与既有分析产物），只新增本文件与冻结清单。
> 代码引用格式为 `文件路径:行号`，行号取自当前工作区 HEAD `8d794788156644325c54a9739509c8fcc960bca7`。

---

## 1. 33 维与 38 维的差异

### 1.1 定义式与来源行号

- 观测维度定义（属性）：
  `src/rl_obstacle_avoidance/scripts/gazebo_env.py:3118-3120`
  ```python
  @property
  def observation_space_dim(self):
      return self.num_sectors + 4 + 11 + N_ACTIONS + PATH_FEATURES
  ```
- `N_ACTIONS = 6`：`gazebo_env.py:75`
- `PATH_FEATURES = 5`（前瞻可达性特征维度）：`gazebo_env.py:76`
- `num_sectors` 默认值来源：`gazebo_env.py:120-121`
  ```python
  self.num_sectors = rospy.get_param("/perception_node/num_sector", 12)
  ```
  即从参数服务器 `/perception_node/num_sector` 读取，缺省 **12**；该缺省与
  `src/rl_obstacle_avoidance/scripts/perception_node.py:31`
  (`self.num_sector = rospy.get_param('~num_sector', 12)`) 一致。

### 1.2 观测拼装（各段语义）

观测向量在 `gazebo_env.py:1823-1824` 拼装：
```python
return sectors + [px, py, pz, pe] + dynamic + last_action_one_hot + \
    self._path_lookahead_features()
```
逐段对应：

| 段 | 维度 | 来源 |
|---|---|---|
| `sectors`（LiDAR 扇区归一化距离） | `num_sectors`=12 | `gazebo_env.py:1776-1782` |
| `[px,py,pz,pe]`（位姿 + 跟踪误差） | 4 | `gazebo_env.py:1784-1792` |
| `dynamic`（速度/偏移/净空变化率/路径误差） | 11 | `gazebo_env.py:1812-1822` |
| `last_action_one_hot` | `N_ACTIONS`=6 | `gazebo_env.py:1810-1811` |
| `_path_lookahead_features()` | `PATH_FEATURES`=5 | `gazebo_env.py:2685-2698` |

`dynamic` 的 11 维为：`velocity_forward, velocity_left, vz/3, reference_speed,
last_offset_x, last_offset_y, last_offset_z`（7）+ `clearance_rates`（3，前/左/右）
+ `path_error`（1）= 11。

### 1.3 验证 12+4+11+6+5=38，及 33 维历史版本

`12+4+11+6+5 = 38`，与 `observation_space_dim` 返回一致。

`33 = 12+4+11+6`，即**不含 5 个前瞻特征**（`PATH_FEATURES`）的历史版本：早期训练
环境在 `_path_lookahead_features()` 加入观测前，维度为 33。`_path_lookahead_features()`
是后续为“沿原航迹前瞻是否需要避障”门控而新增（`gazebo_env.py:2685-2702`），
其[4] 位直接被 `_avoidance_needed()`（`gazebo_env.py:2700-2702`）使用。

### 1.4 各 checkpoint 实际 `observation_dim`

来自 `training_runs/*/config.json`（完整字段见冻结清单 `checkpoint_summary`）：

| 目录 | `observation_dim` | `num_sectors` | `environment.action_interface` | `safety_filter_mode` | `seed` |
|---|---|---|---|---|---|
| `dqn_20260915_075856_887737` | **33** | 12 | (无) | a1 | 0 |
| `dqn_20260915_102257_751824` | **33** | 12 | (无) | a1 | 0 |
| `dqn_20260915_130426_595452` | **33** | 12 | (无) | a1 | 0 |
| `dqn_20260917_075433_512345` | **33** | 12 | (无) | a1 | 0 |
| `dqn_v3_pilot100` | **33** | 12 | (无) | a1 | 42 |
| `dqn_v4_pilot100` | **33** | 12 | (无) | a1 | 42 |
| `dqn_v4_gate_pilot100` | 38 | 12 | (无) | a1 | 42 |
| `dqn_v4_formal_b2_s101` | 38 | 12 | (无) | disabled | 101 |
| `dqn_v4_formal_b2_s202/303/404/505` | 38 | 12 | (无) | disabled | 202/303/404/505 |
| `dqn_v5_raw_s101` | 38 | 12 | raw | disabled | 101 |
| `dqn_v5b_raw_s101` | 38 | 12 | raw | disabled | 101 |

**结论**：`dqn_20260915_*`、`dqn_v3_pilot100`、`dqn_v4_pilot100` 为旧 33 维 checkpoint。
它们的首层网络输入宽度为 33，而当前环境 `observation_space_dim=38`；直接加载会出现
形状不匹配（缺 5 维前瞻特征），**旧 33 维 checkpoint 不能默认加载到当前 38 维环境**，
必须显式改造观测或裁剪网络。`config.json` 未记录 `action_interface` 顶层的为无该字段，
即当时用环境缺省 `masked`（见 §2）。

---

## 2. masked / raw 动作接口差异

### 2.1 参数读取与取值校验

`gazebo_env.py:130-135`：
```python
# 动作接口模式：masked 保持距离掩码（legacy 逐字节不变），raw 让掩码退出裁决。
self.action_interface = str(rospy.get_param(
    "/gazebo_env/action_interface",
    rospy.get_param("~action_interface", "masked"))).lower()
if self.action_interface not in ("masked", "raw"):
    raise ValueError("action_interface must be masked or raw")
```
- 参数名 `/gazebo_env/action_interface`（`gazebo_env.py:132`），私有回退
  `~action_interface`（`:133`），缺省 **masked**（`:133`）。
- 取值校验：`:134-135` 只允许 `masked`/`raw`。
- 运行时注入：`scripts/start_sim.sh:342-346`（`--action-interface` 时
  `rosparam set /gazebo_env/action_interface`）。

### 2.2 raw 档在 `_constrain_action` / `allowed_actions` 中的分支

- `_constrain_action`（`gazebo_env.py:2297-2317`）：
  - 先做**侧向迟滞改写**（`constrain_avoidance_action`，`:2302-2304`）；
  - `raw` 档仅保留该迟滞，随后直接 `return action`（`:2305-2307`），**不再把动作重映射
    进距离掩码**；
  - `masked` 档继续查 `allowed_actions()`，越界时收敛到最近似合法动作（`:2308-2316`）。
- `allowed_actions`（`gazebo_env.py:2346-2384`）：
  - 仍维护侧向锁定 `locked_avoid_side`（`:2358-2374`）；
  - `raw` 档在 `:2372-2374` 直接 `return tuple(range(N_ACTIONS))`——
    **距离掩码退出安全裁决**，仅保留侧向锁定维护；
  - `masked` 档才用 `allowed_avoidance_actions(...)` 做距离阈值掩码（`:2375-2384`）。

### 2.3 stage3 模式 → 接口 / 过滤器 / kappa 映射

`experiments/stage3/run_stage3_batch.py`：
- `KAPPA_A1 = 0.0`（`:53`）、`KAPPA_P = 0.2`（`:54`）
- `FILTER_MODE_OF = {"b2": "disabled", "b2m": "disabled", "a1": "a1", "p": "p"}`（`:71`）
- `KAPPA_OF = {"b2": None, "b2m": None, "a1": KAPPA_A1, "p": KAPPA_P}`（`:72`）
- `ACTION_INTERFACE_OF = {"b2": "raw", "b2m": "masked", "a1": "raw", "p": "raw"}`（`:75`）

| 模式 | 动作接口 | 过滤器模式 | kappa |
|---|---|---|---|
| b2 | raw | disabled | None |
| b2m | **masked** | disabled | None |
| a1 | raw | a1 | 0.0 |
| p | raw | p | 0.2 |

即 **b2 与 b2m 只差“距离掩码”一项**（b2=raw 无掩码，b2m=masked 有掩码）；A1/P 与 B2
同接口（raw），使「过滤器效应」可与「掩码效应」干净分离（注释见 `:73-74`）。
批次内 kappa 通过 `:234-237` 取用，可被 `--action-interface` 临时覆盖（`:348-351`）。

---

## 3. 唯一真实执行链路（逐层）

物理链路：**任务路径 → 候选参考/偏移 → 控制器（外环）→ 位置设定点 → PX4 内环 →
机体运动 → 传感/日志**。以下频率与话题均来自代码。

### 3.1 任务路径（参考轨迹生成）

- 物理量：时间参数化参考轨迹 `trajectory_`（位置/速度/加速度）。
- 生成：`LQRTrackerNode::generateTrajectory()`，`src/lqr_trajectory_tracker/src/lqr_tracker_node.cpp:202-235`
  （按 `trajectory_type_` 分派 `generateLinear/Circular/FixedHorizontal/...`）。
- 发布：参考轨迹 `nav_msgs/Path` → `/lqr_tracker/ref_trajectory`
  （发布者声明 `lqr_tracker_node.cpp:71`）。
- 频率：由跟踪循环决定，`ros::Rate rate(control_rate_)`，`control_rate_` 默认
  **50 Hz**（`:40`、`:144`、`:581`）。

### 3.2 候选参考 / 偏移（RL 避障输出）

- 物理量：三维局部偏移 `(dx,dy,dz)`（机体/地图帧）与速度缩放/暂停控制。
- 生产者（训练链路）：`gazebo_env`（`src/rl_obstacle_avoidance/scripts/gazebo_env.py`）：
  - 偏移 `geometry_msgs/Vector3Stamped` → `/rl_planner/local_offset`
    （发布者 `gazebo_env.py:483-484`，发送 `:2459-2466`）；
  - 局部控制 `geometry_msgs/Vector3Stamped` → `/rl_planner/local_control`
    （`gazebo_env.py:485-486`，发送 `:2468-2476`，x=速度缩放、y=是否暂停、z=相位方向）；
  - 动作号 `std_msgs/Int32` → `/rl_planner/action`（`:482`，`:2230`）。
  - 频率：由 `step()` 决定，`rospy.sleep(self.action_duration)`（`:1544`），
    `action_duration` 默认 **0.1 s → 10 Hz**（`:141`）。
- 另一生产者（非训练交互式链路）：`src/rl_obstacle_avoidance/scripts/rl_planner_node.py`
  以 `rospy.Timer(rospy.Duration(0.2), self.plan_once)` **5 Hz** 运行（`:81`），
  发布 `/rl_planner/action`、`/rl_planner/local_goal`、`/rl_planner/local_offset`
  （`:77-79`）。注意：`--training` 模式下**不启动**该节点（`scripts/start_sim.sh:402`），
  由 DQN 经 `gazebo_env` 独占 offset。

### 3.3 控制器（外环 LQR）

- 消费偏移：`LQRTrackerNode::localOffsetCallback`，`lqr_tracker_node.cpp:306-318`
  （零偏移即清除、恢复原轨迹；否则记 `rl_offset_`）。
- 消费局部控制：`localControlCallback`，`lqr_tracker_node.cpp:320-326`。
- 参考速度发布：`/lqr_tracker/reference_velocity`、`/lqr_tracker/reference_mid_lookahead_velocity`、
  `/lqr_tracker/reference_lookahead_velocity`（均为 `geometry_msgs/Vector3Stamped`）——
  发布者声明 `:73-78`，发送 `:668-676`（当前）、`:682-687`（中）`:689-695`（前瞻）。
- 前馈：`position_correction += ref.velocity * position_lead_time_`（`:568-569`）。
- 频率：同跟踪循环 **50 Hz**（`:581`）。

### 3.4 位置设定点

- 物理量：`desired_position`（LQR 输出，`lqr_tracker_node.cpp:735`）。
- 发布：`publishPositionSetpoint()`（`:458-469`）→ `geometry_msgs/PoseStamped` →
  `/mavros/setpoint_position/local`（发布者 `:69`），频率 **50 Hz**（跟踪循环）。
- 提前量：节点参数 `position_lead_time` 默认 **`dt_ = 0.02 s`**（`:37`、`:148`）；
  实验协议冻结值 **0.80 s**（见 §3.6）。

### 3.5 PX4 内环与起飞阶段

- 起飞/占位：`drone_launch_controller` 用 `sendSetpointsBeforeOffboard(2.0, 20.0)`
  以 **20 Hz** 预发设定点到 `/mavros/setpoint_position/local`
  （`src/drone_launch_controller/src/drone_launch_controller_node.cpp:87-93`、`:335`、`:465`；
  发布者声明 `:36`）。
- 交出控制权：LQR 启动后调用 `/drone_launch_controller/release_control`
  （`lqr_tracker_node.cpp:801`），此后由 LQR 独占设定点。
- PX4 SITL：`scripts/start_sim.sh:203-206`
  `roslaunch px4 mavros_posix_sitl.launch vehicle:=iris vehicle_sdf:=iris_3d_lidar`，
  MAVROS 连接 `/mavros/*`；置 OFFBOARD 与解锁见 `drone_launch_controller_node.cpp:209-253`、`:265-291`。

### 3.6 机体运动 → 传感 / 日志

- 机体：Gazebo iris_3d_lidar 模型在 PX4 Offboard 位置控制下运动（`start_sim.sh:206`）。
- LiDAR 点云：`sensor_msgs/PointCloud2` → `/velodyne_points`，
  由 `perception_node.py:49-50` 订阅，回调 `:65-74` 分扇区。
- 感知输出：`ObstacleFeatures` → `/obstacle_features`（`perception_node.py:57`、`:183`）；
  频率由点云帧率驱动（回调即发布）。
- 状态反馈：`/mavros/local_position/pose`（`geometry_msgs/PoseStamped`）与
  `/mavros/local_position/velocity_local`（`geometry_msgs/TwistStamped`），
  被 `perception_node.py:51-54`、`gazebo_env.py:465-468`、`lqr_tracker_node.cpp:57-60` 订阅。
- 日志：gazebo_env/`run_eval`/`train_dqn` 写 step_log CSV 与 episode 指标
  （`gazebo_env.py:1538-1542` 输出 `geometry_clearance` 等诊断；训练侧 `train_dqn.py:551`
  记录 `observation_dim`）。

### 3.7 延迟数值（标注实测冻结 vs 代码默认）

来源：已冻结的 `experiments/stage1/task2_setpoint_response/control_validation_protocol_v1.md`。

| 数值 | 值 | 性质 |
|---|---|---|
| `position_lead_time` | **0.80 s** | **实测冻结协议值**（`:10`）；节点**代码默认仍为 0.02 s**（`:18`、`lqr_tracker_node.cpp:37,148`） |
| 阶跃纯延迟 | **0.24 / 0.24 / 0.28 s** | **实测冻结值**（`:31`） |
| 阶跃时间常数 | **0.85 / 0.85 / 0.80 s** | **实测冻结值**（`:31`） |
| `tau_react` | **0.84 s** | **实测冻结值**，仅作当前仿真接口的保守表观延迟（`:15`） |
| `a_brake` | 0.044 m/s² | 实测冻结 p10，仅适用当前低速窗口（`:16`、`:34`） |

说明：`position_lead_time=0.80 s` 是**实验协议参数**，不改节点默认配置；阶跃纯延迟/时间常数
与 `tau_react` 均为**实测冻结值**。上述“默认值”仅用于区分「协议冻结」与「代码缺省」。

---

## 4. `_safety_obstacles()` 真值权限复查

### 4.1 该函数读什么、怎么筛选

`gazebo_env.py:2267-2295`：

- 前置门：无位姿、特征不新鲜、或无扇区时返回 `None`（`:2275-2278`）；
  全清帧（最小扇区 ≥0.999）返回 `[]`（`:2280-2281`）。
- **真值几何来源**：`source = self.scenario_obstacles_typed`（`:2284`）；
  若为空则回退用障碍中心 + 尺寸拼装（`:2286-2289`）：
  ```python
  source = [{"center": [cx, cy, cz], "size": self.obstacle_geometry_size, ...}
            for cx, cy, cz in self.obstacle_geometry_centers]
  ```
  其中 `scenario_obstacles_typed` 在 `:625-638` 赋值，`obstacle_geometry_centers`
  在 `:636-644` 由真值中心构成，`obstacle_geometry_size` 在 `:336-337` 设为
  缺省 `DEFAULT_OBSTACLE_SIZE`。
- **范围筛选**：用当前 LiDAR 扇区的**最小归一化距离**换算出半径，仅保留
  「中心到无人机欧氏距离 ≤ 该半径」的障碍（`:2279-2295`）：
  ```python
  observed_range = min(float(value) for value in self.features.lidar_sectors)
  ...
  if math.hypot(cx-px, cy-py) <= min(lidar_max_range,
        max(lidar_min_range, observed_range * lidar_max_range + 1.0))
  ```
- 调用点：`gazebo_env.py:1991`（`obstacles = self._safety_obstacles() if sensor_valid else None`）。

### 4.2 结论：属“真值辅助几何诊断配置”

该函数把**场景真值的障碍中心与尺寸**（`scenario_obstacles_typed` /
`obstacle_geometry_centers` / `obstacle_geometry_size`）作为安全过滤器的障碍“形状”输入，
只用当前帧 LiDAR 的标量最小距离做**范围开关**。因此它属于**真值辅助几何诊断配置**：
安全裁决所依赖的障碍几何并非全部来自传感器。

### 4.3 为何不能支持“纯 LiDAR、未知环境”主张

- 障碍的**位置与尺寸来自场景真值**，而非从点云重建；在未知环境中这些量本不可得。
- 距离筛选只判断“障碍中心与无人机的**欧氏距离**是否在阈值内”，**并不判断该方向是否真的被
  观测到**。因此**无法排除遮挡后方或不可见方向上的障碍**：一个位于无人机背后/被遮挡、
  但欧氏距离落在阈值内的真值障碍仍会被纳入过滤器，反之一个真实存在却未被扫到的方向也不会
  被正确建模。距离门限不能替代可见性/遮挡建模。
- 由此，用该配置得到的“安全”结论不能据以声称主实验满足「纯 LiDAR、未知环境」前提；
  若作为主实验，必须隔离真值几何，改为仅用感知可达的几何（可见性/遮挡一致）作为过滤器输入。

---

## 5. `cross_validation_ok` 拆分

读 `experiments/stage3_task5_cross_validate.py`。其退出码（`:161-163`）要求同时满足
`sound` 与 `within_tolerance`；把判据拆分为两类：

### 5.1 (a) 实现一致性检查

- **声学性（安全关键）**：`predicted_h` 对 `geometry_clearance`（全场景几何真值）比较，
  `error = predicted - observed`（`:48-59`）；`false_safe` 为「预测判定安全但真值不安全」的计数
  （`:62-63`），`sound = (false_safe == 0 and bool(compared))`（`:103`）。
  即要求 `predicted_h` 对 `geometry_clearance` **恒为保守下界**。
- **同量残差双侧界**：比较 `predicted_h + filter_margin` 与 `filter_clearance_now`
  （`:70-80`），残差 `residual = predicted + margin - current`（`:77`）；
  - `optimistic`：`residual > optimism_tolerance` 的计数（`:84`）——预测比实测乐观，禁止；
  - `conservative_exceeded`：`residual < conservative_bound` 的计数（`:85-86`），
    下界由 `-(max_speed*horizon + optimism_tolerance)` 给出（`:40-42`）；
  - `within_tolerance = (bool(residuals) and optimistic==0 and conservative_exceeded==0)`（`:122-123`）。

**为何属“几何/实现一致性”**：预测轨迹的**首个采样点即当前位置**
（`src/rl_obstacle_avoidance/scripts/safety_filter.py:84`，`trajectory=[(x,y,z)]` 起于当前位姿；
`predict_trajectory` 的时域最小净空因此在**同一障碍集**上必然 ≤ 当前瞬时净空
（`safety_policy` 的 `predicted_h` 也扣同一 margin）。所以“预测最小净空 ≤ 当前净空”
在同障碍集上**本来就应该成立**，把 `predicted_h` 与**同一时刻**的 `geometry_clearance`
相比、把 `predicted_h+margin` 与**同一时刻**的 `filter_clearance_now` 相比，
检验的是“预测函数与几何/代码是否自洽”，**不涉及未来**。

### 5.2 (b) 尚缺的未来预测验证

现有脚本的 `compared` / `same_quantity` 均按**同一行（同一时刻）**取
`predicted_h`、`geometry_clearance`、`filter_clearance_now`（`:48-80`），
**没有**用未来真实观测去检验预测。真正需要的验证是：

1. 对齐**预测发出时刻 k 的实际初态**（真实位置/速度）；
2. 对齐**实际执行的输入序列**（k…k+N 真实动作/控制量）；
3. 对齐**未来真实观测**（k+j 的真实几何/净空日志）；
4. 检验**未来 k+j 位置是否被预测轨迹覆盖**，以及**真实最小扫掠净空是否满足预测下界**。

现有脚本**不能替代**该验证，原因：它没有回放「实际执行的输入」（用的是内部按恒速动作外推的
预测），也没有使用 k+j 的**未来观测**（只用了同一时刻的观测）；因此它既未检验“预测位置覆盖
真实未来位置”，也未检验“真实未来扫掠净空 ≥ 预测下界”。现有脚本的四项指标（`sound`、
`false_safe`、`optimistic`、`conservative_exceeded`）都只是**同一时刻**的
实现/几何一致性，无法证明预测在未来时域内成立。

### 5.3 声明

本次为纯读码审计，**未改写任何旧结果文件**：`cross_validation.json`、
`audit_index.json/csv` 与既有分析产物内容保持不变，仅新增本说明文档与冻结清单。

---

## 6. 冻结清单摘要

- 清单路径：`docs/指导书/stage_logs/one/frozen_manifest.json`（生成脚本：
  `experiments/paper_v3/scripts/freeze_baseline.py`）。
- 覆盖文件数：**47 项**（关键源码 12 + checkpoint 28 + 实验索引 3 + 场景清单 4）；
  其中**存在并记哈希 45 项，缺失记 `null` 2 项**（`training_runs/dqn_20260915_075856_887737/best_model.pt`
  与 `training_runs/dqn_20260915_102257_751824/best_model.pt`，这两个旧目录只有 `last_model.pt`，
  无 `best_model.pt`，按硬要求记 `null`，未臆造哈希）。
- checkpoint 摘要：覆盖 14 个 `dqn_*` 目录的 `observation_dim`/`num_sectors`/
  `action_interface`/`environment.action_interface`/`safety_filter_mode`/`seed`。
- 两次运行一致性（幂等）：在**工作区内容不变**时两次运行，所有 `sha256`/`bytes`/`files`/
  `checkpoint_summary` 完全一致；唯一差异是 `generated_utc` 时间戳本身（预期内）。
  运行命令与结论：
  ```
  python3 experiments/paper_v3/scripts/freeze_baseline.py   # 第 1 次
  python3 experiments/paper_v3/scripts/freeze_baseline.py   # 第 2 次
  # diff（忽略 generated_utc）为空 → 幂等；仅 UTC 时间戳按秒变化
  ```
- `git_revision`：`8d794788156644325c54a9739509c8fcc960bca7`；`git_dirty`：`True`
  （工作区存在未提交改动，脚本只读取状态、不修改 git）。