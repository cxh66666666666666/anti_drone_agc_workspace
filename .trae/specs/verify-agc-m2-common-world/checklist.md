# Checklist

> 逐项核查 [spec.md](file:///home/ubuntu/anti_drone_agc_workspace/.trae/specs/verify-agc-m2-common-world/spec.md) 与 [tasks.md](file:///home/ubuntu/anti_drone_agc_workspace/.trae/specs/verify-agc-m2-common-world/tasks.md)，按契约「可核查证据」口径判定。
> 核查方式：现场无头实跑（`scripts/start_agc_collab.sh`）+ `audit_interfaces.py` 现场核查 + run 日志核对 + 代码路径核对。

## 保护距离标定与碰撞口径

- [x] `GROUND_COLLISION` 仅由接触/几何相交触发（`clearance ≤ car_radius_m`），不再用 `collision_m`
- [x] `CLEARANCE_VIOLATION`/`EMERGENCY_TRIGGER` 仍为不改结局的保护带事件，净空度量注释为「车中心到障碍 OBB 表面有符号距离」
- [x] `collision_m` 已从 `agc_params.yaml`、`scenario_flat.yaml`、`agc_common.cpp`、`task_manager.cpp` 全部移除，全仓无残留引用
- [x] `scenario_flat.yaml` 中重复且从未生效的 `thresholds.clearance_*` 已删除，净空阈值以 `agc_params.yaml` 为唯一来源
- [x] 标定值与推导一致：`car_radius_m=0.30`、`emergency_m=0.35`、`protection_m=0.80`，顺序为 `接触(0.30) ≤ emergency(0.35) < protection(0.80)`
- [x] `catkin build air_ground_core` 通过

## 世界、时钟与模型

- [x] 协同 run 只有一个 gzserver / 一个世界 `agc_world`，无 `ground_gazebo` 节点
- [x] 全局 `/use_sim_time=true`，`/clock` 只有一个发布者
- [x] 同一条 `/gazebo/model_states` 内同时含 `iris`/`agc_car`/`intruder_uav`/`pole_1`/`pole_2`
- [x] TF `world`/`air_local`/`drone/base_link`/`car/base_link` 闭合，无裸 `base_link`

## 同拍位姿、坐标变换与时间戳

- [x] 同一 run 内 `/agc/air/odom`、`/agc/ground/odom`、`/agc/targets` 同时收到有效样本，年龄均不超阈值
- [x] `/agc/air/odom` 位置与 Gazebo `iris` 真值位置差 ≤ 0.20 m
- [x] 同 run `air_calibration.json` 的 `residual_m` 非空且 ≤ 0.05 m
- [x] `/agc/ground/odom` 位置与 Gazebo `agc_car` 真值位置差 ≤ 0.05 m
- [x] 采样消息 `header.stamp` 与 `/clock` 差 ≤ 1.0 s（证明取自仿真时钟）

## 地图接口

- [x] `/agc/map` 的 `info.resolution`/`width`/`height`/`origin` 与 `scenario_flat.yaml` 的 `map` 段一致
- [x] `pole_1`/`pole_2` 中心所在栅格为占用，占用格数 > 0
- [x] `/agc/map/version_hash` 非空，且被规划日志消费（`planner_log.jsonl` 含 `map_version_hash`）

## 入口与退出

- [x] 唯一启动入口 `scripts/start_agc_collab.sh` 可无头一键启动并干净退出
- [x] M2 批跑 `batch.json` 显示退出后无残留进程；批跑结束单机进程核查为 0

## 标定结果不破坏安全判据

- [x] 执行轨迹最小净空不低于 `protection_m`，且未出现 `GROUND_COLLISION`/`CLEARANCE_VIOLATION` 结算
- [x] run `meta.json` 的 `params.clearance` 含标定值且不含 `collision_m`；`agc_params_hash`/`scenario_hash` 可复算

## 交付与口径

- [x] `docs/项目任务日志/M2/M2_共同世界验收_v1.md` 给出任务口径、批跑设置、逐项结果、标定推导、交付物核对与未决事项
- [x] 措辞只称 `TRACKING_SUCCESS`（抵近并持续跟踪），未表述为拦截完成，未出现 `INTERCEPT_CONFIRMED`
- [x] 未推进 M3–M9、阶段 B 拦截、真实融合与实物；M1 遗留项未纳入本轮（按用户决定）