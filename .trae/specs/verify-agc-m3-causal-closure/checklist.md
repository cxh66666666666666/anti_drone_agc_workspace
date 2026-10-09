# Checklist

> 依据 [spec.md](file:///home/ubuntu/anti_drone_agc_workspace/.trae/specs/verify-agc-m3-causal-closure/spec.md)。逐项核查「可核查证据」，不以「已有代码」等同「已验收」。

## 受控对比的空中输入（仅改变目标）

- [x] `config/scenario_linear_target.yaml` 与 `scenario_flat.yaml` 除 `target_motion` 外逐字段一致（`car_start`/`target_start`/`obstacles`/`map`/`thresholds`）
- [x] 处理组 `target_motion.mode=linear` 且 `speed_mps ≤ 1.0`（1.0 = `commander.max_linear_speed_mps`），并给出可达性说明
- [x] 处理组 run 任务活动窗口内目标地面位置变化范围 ≥ 1.0 m（来自 `target_trajectory.jsonl` 或 `topics/_agc_targets.jsonl`）

## 空中目标改变驱动地面规划更新

- [x] 处理组 run 任务活动窗口内 `goal` 与目标地面投影中位距离 ≤ 0.5 m、最大 ≤ 1.5 m，且 `goal` 变化范围 ≥ 1.0 m（排除每次 `scenario_reset` 后 2.0 s）
- [x] 对照组 run 任务活动窗口内 `goal` 变化范围 ≤ 0.2 m、目标变化范围 ≤ 0.1 m
- [x] `planner_log.jsonl` 每条规划记录（`trigger≠invalid`）含 `trigger`/`goal`/`car_pose`/`map_version_hash`/`plan_time_ms`/`n_waypoints`/`success`/`failure_reason`
- [x] 处理组 run 含 ≥1 次 `trigger="displacement"`；对照组 run 无 `displacement`，其 `trigger` 仅取 `first`/`period`/`infeasible`/`invalid`

## 规划更新驱动车运动（执行闭环）

- [x] 处理组 run 车实际位移 ≥ 3.0 m（来自 `/agc/ground/odom`）
- [x] `GROUND_APPROACH` 期间存在非零 `/agc/ground/cmd_vel.linear.x`
- [x] 车位移方向与「车→goal」方向的平均同向性为正

## 无信息时安全停车

- [x] `/agc/targets` 转空后 ≤1.0 s 内 `/agc/ground/cmd_vel` 归零
- [x] 自最后有效目标样本起，车实际位移 ≤ 0.10 m
- [x] 无信息 run 终态为 `MISSION_FAILED` 且 `reason_code=TARGET_LOST`（或等价失联终态）

## 离线核验与证据

- [x] `scripts/verify_causal_closure.py` 打印 PASS/FAIL 表并写出 `experiments/agc_runs/m3_causal_closure.json`
- [x] 核验结果全 PASS；任一 FAIL 时脚本退出码为 1
- [x] 三 run 的 `meta.json` 可复算 `agc_params_hash`/`scenario_hash`，且对照组与处理组 `agc_params_hash` 相同
- [x] 证据目录 `_m3_batch_*`（对照/处理/无信息）存在且每 run 干净退出（`any_leftover=false`）
- [x] `docs/项目任务日志/M3/M3_空地最小闭环验收_v1.md` 存在，含受控对比证明、因果链逐项结果与措辞核对（`TRACKING_SUCCESS` ≠ 拦截完成）

## 边界与只读

- [x] 未修改任何实时链路 C++ 节点、M0 已冻结话题/消息、launch、`start_agc_collab.sh`、`scenario_flat.yaml`
- [x] 新增 Python 工具仅为离线日志后处理，未订阅/发布实时话题
