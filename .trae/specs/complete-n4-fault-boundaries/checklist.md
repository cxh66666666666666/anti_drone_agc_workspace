# Checklist（change-id: complete-n4-fault-boundaries）

## 冷启动冲突规则
- [x] `air_fusion` 无历史参考时使用最大簇（鲁棒）参考，而非最高置信单点
- [x] 候选 ≥2 且最大簇 < `cold_start_min_agree` 时发布空 `/agc/targets` 且 `result=cold_start_ambiguous`（批 `_n4_coldoutlier2_…` 1318 拍歧义、nonempty=0）
- [x] 有历史（`ref_hold_s` 内）路径行为与 M4 一致，未改动（① 名义融合 PASS=7）
- [x] 融合日志含 `cold_start` / `ref_source` / `cluster_sizes`
- [x] `agc_params.yaml` 含 `fusion/cold_start_min_agree`，哈希入 run 日志（`agc_params_hash=fdac26f82135…`）

## 单通道失效
- [x] 单通道 drop 后融合仍由剩余通道输出非空目标（单源 fused tick=71）
- [x] 不出现 `gate_rejected_all` 误报；正常通道不被误拒（③ PASS=9 FAIL=0）

## 控制侧与注入
- [x] `release_control` 后起飞控制器 setpoint advertiser 被关闭；`takeoff`/`arm` 可重新 advertise
- [x] 交接后 `air_control_authority.jsonl` 的 `setpoint_publishers` 仅含 `lqr_tracker_node`（①/④/⑥ B5 PASS；⑤ 注入后为空，均不残留起飞控制器）
- [x] `--timeout-inject` 经 launch 覆盖 `limits/mission_timeout_s` 并写入批记录（④）
- [x] `--handover-inject` 交接后终止 LQR，产生被记录的控制空窗（⑤ control_conflict=1）

## 离线核验与证据
- [x] `verify_fusion.py --mode outlier` 覆盖冷启动歧义并 PASS（PASS=7 FAIL=0）
- [x] `verify_fusion.py --mode single_channel` PASS（PASS=9 FAIL=0）
- [x] `verify_control_authority.py --mode timeout` 校验 `MISSION_TIMEOUT` + 终态后停车/hold + 唯一控制源并 PASS（PASS=14 FAIL=0）
- [x] `verify_control_authority.py --mode handover` 校验 advertiser 不残留 + 控制空窗记录并 PASS（PASS=13 FAIL=0 SKIP=2）
- [x] 批次均 `any_leftover=false`，`--gui` 非无头，场景/参数哈希锁定（`scenario_hash=04f448176f95…`、`contract=agc-m0-v1.2`）
- [x] 回归 M4 nominal（7/0）/ M5 nominal（14/0）+conflict（15/0）/ M6 nominal（baseline 3/0 + 冻结 81/0）无新增误报

## 文档
- [x] `docs/项目任务日志/阶段2/N4.md` 含口径与诚实说明、实现、批次哈希、逐项 PASS/FAIL、交付物核对、未决事项
- [x] 未改契约版本（`agc-m0-v1.2`），未出现 `INTERCEPT_CONFIRMED`，未表述为拦截完成