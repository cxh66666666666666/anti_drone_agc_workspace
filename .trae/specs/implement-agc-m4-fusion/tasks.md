# M4 实施任务

## T1 契约与消息（v1.2）
- [x] 新建 `docs/项目任务日志/M0/M0_接口契约_v1.2.md`（v1.1 基础上追加"感知与融合"一节）
- [x] 新增 6 个 msg：TargetObservation / ObstacleObservation / SensorObservation / AirObservationArray / ObstacleTrack / ObstacleTrackArray
- [x] CMakeLists 注册消息与新节点 `air_perception`、`air_fusion`
- [x] `agc_params.yaml`：contract_version=v1.2；新增 perception/fusion/calibration 段

## T2 感知前端 air_perception（C++）
- [x] 订阅 `/gazebo/model_states`，读 `intruder_uav` + 障碍真值
- [x] 多通道（默认 2 个）独立 RNG 噪声 + 独立偏置 + 延迟缓冲 + 丢包 + 离群偏置
- [x] 发 `/agc/air/obs`（AirObservationArray），观测 header.stamp=观测时刻
- [x] 故障在线可调：`/agc/perception/fault/<sensor>/{enabled,drop_prob,latency_s,outlier_bias_m}`
- [x] 落 `perception_log.jsonl`；写 `/agc/perception/obs_seq/<sensor>` 诊断

## T3 融合节点 air_fusion（C++）
- [x] 订阅 `/agc/air/obs`；新鲜度/置信度/空间一致性门控
- [x] 目标：inlier 逆方差加权 + 相关噪声保守协方差；输出 `/agc/targets`(source=air_fusion)
- [x] 障碍：按 obstacle_id 关联 inlier，并集+保守外扩；输出 `/agc/obstacles`
- [x] 无有效观测发空数组，绝不刷新旧 stamp
- [x] 落 `fusion_log.jsonl`（输入/门控/离群/输出/拒收原因）；写 `/agc/fusion/*` 诊断

## T4 接线
- [x] `agc_collab.launch`：`target_source`/`obstacle_source` arg；launch 单一设置 source 参数；条件包含节点
- [x] `start_agc_collab.sh`：`--target-source`/`--obstacle-source`；PROC_NAMES 增两节点
- [x] `gt_adapter`：仅 source==gt_adapter 时 advertise `/agc/targets`
- [x] `agc_run_logger`：target_source 读参数、meta 增 obstacle_source、CONTRACT_DOC→v1.2、procKeywords 增两节点
- [x] `audit_interfaces.py`：`/agc/targets` 期望发布者读参数；按 source 追加 v1.2 话题；nominal/fault 模式

## T5 障碍接入规划
- [x] `map_adapter`：source=air_fusion 时订阅 `/agc/obstacles` 重建障碍格，带内容哈希门限；scenario 不变
- [x] `ground_planner`：source=air_fusion 时用融合障碍做 OBB 精膨胀；scenario 不变
- [x] `task_manager` 不改安全判据（真值裁判安全）

## T6 离线工具
- [x] `scripts/m4_inject.py`：按时间线注入故障并落盘注入事件
- [x] `scripts/verify_fusion.py`：核验字段/可消费/延迟/丢失/冲突/标定

## T7 编译与验证
- [x] `catkin_make` 通过
- [x] 基线回归：`--target-source gt_adapter --obstacle-source scenario` 与 M2/M3 一致
- [x] fusion nominal 批：M3 因果闭环仍成立
- [x] latency/loss/conflict 注入批 + obstacle-fusion 批
- [x] `verify_fusion.py` 全项 PASS

## T8 验收文档
- [x] `docs/项目任务日志/M4/M4_目标识别与多源融合验收_v1.md`
