# Checklist

> 与 `spec.md` 的 Requirement 一一对应；逐项通过后在方框打勾。先完成后由独立代理核对**代码/运行输出/JSON 字段**。结论：**31 项全部通过**（另 3 处验收附注已修正）。

## F1 唯一执行协议与「证书=执行」一致
- [x] `closed_loop_v2.py` 实现**每拍重新观测→重新决策→只发布 1 拍命令**的闭环；转备份则持续发布保持命令（`closed_loop_v2.py:235-306`）
- [x] **禁止**「候选一拍+备份预测」与「持续发布候选」混用：`certificate_input_matches_executed_all_trials=true`，140/140 trial 为真；`max_consecutive_identical_candidate_published` 全域 max=1
- [x] `certificate_v2.py` 证书覆盖**全部允许延迟分支**（`D=[0.18,0.24,0.30]`）+ **方向误差集合**（`σ_E(−∇h)`）+ **全时域扫掠净空** + 终端条件；无点延迟/仅末端判据（`:104-113,127,133-148`）
- [x] 结果**区分五类**且各由**测量**计算：`five_classes_all_present=true`、`all_measured=true`；标签由测量布尔式导出，无硬编码
- [x] 含四类有辨别力样例：`S_feasible_initial`/`S_infeasible_initial`/`S_candidate_to_backup`/`S_repeated_redecision`

## F2 版本化辨识参数与重跑
- [x] 明确区分**速度通道**（`0.2999`）与**位置通道**（`1.1732`）且未互换，给出判断＝**受限**（`params_v2.json:f2_position_channel_verification.judgment`）
- [x] 在**整次 run** 验证数据上报告**多步位置/速度误差**（`aggregate_max`：j=1/5/10/20/50 → 0.0333/0.1048/0.1904/0.3061/0.4505 m）
- [x] 参数版本化（来源/区间/验证结论）；延迟取区间**下/中/上**且真值上界 `0.30 ≥ 0.30`（`delay_branch_definition`）
- [x] **并列**原合成参数作**敏感性对照**（`by_param_set.synthetic_sensitivity`）
- [x] 「已解决」须**不虚报安全 + 可行工况取得进展**；不可行工况只报**不可认证**（`resolved.by_scenario.*` 的 `handled_by`/`uncertifiable_by`）
- [x] 未沿用旧 `no_counterexample_requires_P`：旧字段标 `superseded`，新字段 `no_counterexample_requires_P_v2=false` 独立存在

## F3 时变延迟覆盖
- [x] 显式写出**允许转移规律**（`fixed`/`single_jump`/`per_step_arbitrary`）；逐拍可变用**逐拍所有后继可达集**（区间凸外包，保守）（`coverage_v2.py:34-38,68-96`）
- [x] 复审反例作为**必过样例**且 sound 方案**被覆盖**：`must_pass_sample.json:sound_covers_this_sample=true`，与 `13_review_checks.json` **逐字段一致**
- [x] 旧 `proto_*`/恒定分支在该样例上**失败被复现**（`scheme_results`：`proto_*` 首违例 h=6，恒定分支 h=16）
- [x] 三类测试分别报告：`aggregates`——fixed 全 0；single_jump sound 0 / proto_sliding 291 / proto_commit 365 / 端点 2 支 9 / 全 7 支 8；per_step_arbitrary sound 0 / 462 / 771 / 316 / 314
- [x] 同任务/同时域/同预算计时并说明路径差异（`timing_v2.json` 含 `code_path_diff`；不据一次计时判效率）
- [x] 撤回/限定旧结论（`conclusion.endpoints_cover_general_time_varying=false` 等；`superseded_fields` 3 条）

## 二次 R0 决策
- [x] `16_r0_decision_v2.md` 汇总 F1—F3 证据且链接**真实字段**（抽查 3 条均存在且取值一致）
- [x] 明确「先完成离线修正」顺序；**SITL 不作为第一步**（`16` §2）
- [x] 给出三选一结论并**正式终止当前 `P` 投稿主张**（`16` §3）
- [x] **未生成 R1**；给出先离线的最小实验与 **≤2 周/42—70 h** 预算（`16` §4—§5）
- [x] `11_advisor_memo_v2.md` §6 追加节存在，状态全「未讨论」，**未编造**导师认可

## 文档修正与可追溯
- [x] `14_offline_protocol_and_comparison.md` 交付（协议、全分支证书、版本化参数、重跑结果、五类判定口径）
- [x] `15_time_varying_coverage.md` 交付（转移规律、可达集、必过样例、三类测试、同预算计时）
- [x] `04`/`05`/`06`/`08` **追加**修正声明且**原有小节未删改**（04/05/06 末尾 + 08 §7）
- [x] F4 改述为「**能识别不可认证工况**」，并**撤回**「已解决反例」与 `no_counterexample_requires_P`（`04:156`、`06:95`）
- [x] `05`/`08` 的时变覆盖结论**被限定**至原测试场景并指向 `15`
- [x] 旧 v1 结果**保留为历史材料**（`fair_compare/results`(56) 与 `history_probe/results`(42) 与 `results_v2` 并存；v1 哈希 98/98 OK）
- [x] `docs/指导书/stage_logs/two/publication_tasks.md` 状态句如实（**R0 仍未通过、未生成 R1**）；其它章节未改（§2 P0—P4 勾选仍在）

## 证据纪律与冻结
- [x] `one/` 冻结 meta/trace 与 `src/`+`scripts/` 哈希未变（32/32、241/241 OK）；v1 结果哈希未变（98/98 OK）
- [x] 新脚本确定性可复算（除 `params_v2.json:generated_at`、`timing_v2.json` 墙钟）；无 ROS/Gazebo 残留进程
- [x] 标 `model-level`；零违例只写「本批未观测到」；未引用 `all_survive`/`fair_brake_compare` 作证据

## 验收附注的处置（三项）
- [x] 附注 1（`15` §6 计时表与 `timing_v2.json` 不符）→ 已更新为当前 JSON 数值，并标注为**一次运行的非确定性观测值、不参与任何结论**
- [x] 附注 2（`certificate_input_matches_executed` 断言近乎恒真）→ 已改为比较「**证书实际评估展开序列** vs **实际发布命令**」逐拍，新增 `first_certificate_input_mismatch_index`、`certificate_sequence_source` 与 `--self-test`；自检 `detects_tampering=true`（改写后翻 `false`、恢复后 `true`）；140/140 trial 仍为真，且**数值语义未变**（剥除新增字段后 142 文件哈希全等）
- [x] 附注 3（`docs/指导书/stage_logs/two/publication_tasks.md` 与 `11` §5.1 旧表述未标 superseded）→ 两处均已加**内联 superseded 标注**并指向 `13`/`14`/`15`/`16`