# Checklist

> 与 `spec.md` 的 Requirement 一一对应；逐项通过后在方框打勾。
> 核对方式：独立验收代理逐条对照**代码/运行输出/JSON 字段**（非转述文档自我声明）。结论：**31 项全部通过**。

## N1 小规模辨识与独立验证
- [x] 先完成 2 个烟雾样例，核验 `/clock`+`use_sim_time`、`publish_log` 实际发布与**完整尾段**；确认后才批量（`smoke_long_01/smoke_lat_01` 的 `meta.status=ok`、`use_sim_time=true`；`trace.csv` 含 `presend→…→brake→tail`；mtime 早于 12 trial）
- [x] 12 个 trial＝纵/横 × 0.5/1.0 m/s × 3，逐 run 的 `fit`/`calib`/`valid` 用途**预先**写入 meta（非时间行分割）（`trial_manifest.json`：`n_runs=12`、`r1/r2/r3→fit/calib/valid`；用途由 run-id 约定在采集前固定，`03` §4 已澄清并说明 `meta.json` 保留原始记录未回写）
- [x] 命令过程含**斜坡**（巡航匀速参考）与**制动激励**（设定点回退实测位置）（`brake_hold_test_v2.py` 巡航/制动段；脚本 `sha256` 与全部 run 的 `meta.script_sha256` 一致）
- [x] 报告**命令速度 / 制动实际初速 / 巡航稳态**，并对 1.0 档约 1.10 m/s 给解释，未改标签（`03` §5；`identified_params.json.speed_split`：1.0 档制动初速 1.085、巡航稳态 1.012）
- [x] 模型与误差范围**只在** `fit`/`calib` 拟合；`valid` 只做多步位置/速度与制动峰值/回摆检查（`identify_v3.py` 按 purpose 分流；`fit_sets.valid_runs=*_r3`）
- [x] 不可识别的延迟参数给**区间/不可识别**结论，未硬拟合单值（`delay_interval_s=[0.18,0.30]`）
- [x] 给出 `Legacy` / 正确位置接口低阶标称模型 / `B2` 范围的对比（`identified_params.json.method_compare`）
- [x] `03_identification_validation.md` 交付，含原始消息/trace、元数据、可复算汇总与局限
- [x] 异常 run 标独立状态（`clock_jump`/`clock_stall`/`drop_frame`/超时），未与物理失败混淆（本批 14 run 全 `ok`，无异常）

## N2 可证伪的公平制动机制对照
- [x] 规则法/`B1`/`B2` 使用**同一执行器与同一备份函数**，速度上限/命令约束/传感权限/任务时间一致（`common.py:40-55` SHARED；`:87-120` 同一真值闭环；`:213-219` 同一备份动作；`04` §2；`common.py` 夹注已精确为「同一执行器 + 共同备份语义；规则法内联同一备份目标下的判据（同一能力，非同一函数体）」）
- [x] 规则法使用**速度相关制动距离**，非错误常量阈值（`common.py:134` `d_brake=v·T·(1−exp(−L̂/T))`）
- [x] 覆盖三类控制变量场景：同净空不同速度、同误差范围不同方向/墙面方向、同位置命令不同允许延迟；记录初态/命令/真值/控制侧可见信息（`scenarios.py`；逐 trial JSON 含 `initial_state/command/truth/control_visible`）
- [x] 输出碰撞/裕量违例、通行/停滞、制动最小净空；`passed`/`survives` **由测量计算**（`common.py:239-245`；`run_fair_compare.py:94-108`；无硬编码）
- [x] 含正/负/已解决对照（双方均安全、双方均不可行、标准方法已解决反例）（`summary.json` 的 `controls`/`resolved`，含 `K_open_both_safe`/`K_infeasible_no_backup`/`K_resolved_ce1-3`）
- [x] 未执行候选分叉标 `evidence_level=model-level`，未冒充实飞观测（`run_fair_compare.py:49`）
- [x] `04_fair_comparison.md` + 逐 trial 结果 + 一键重算脚本交付；能回答「标准方法已解决哪几项、还剩什么」（`resolved.no_counterexample_requires_P=true`）

## N3 执行历史细化的独立价值
- [x] 原型与 `B2` 同模型族/噪声界/方向表示/备份/计算上限；**不读真实延迟标签**；未引入 RL（`prototype.py` 仅 import `baselines`；`true_branch_index` 仅在 `probe.py:109` 评分侧；`budget_steps=280`）
- [x] 固定延迟与允许时变延迟**分开**报告；任意跳变保留**所有**允许后继分支（`prototype.py:176-180` `retain_sound`；`probe.py` fixed/tv 场景与评估器分开）
- [x] 构造并运行「过去看似排除、未来仍可能发生」的检查（`summary.conclusion.missed_true_branch_observed_in_tv=true`）
- [x] 记录保留分支数、范围宽度、覆盖违例、可行/通行比例、计算耗时（`summary.json.aggregates` + `timing.json`）
- [x] **仅范围变小却漏真实分支**判为失败（含至少一个此类或边界例子）（`run_probe.py:87-99`；tv `missed_true_branch` + 覆盖违例判负例）
- [x] 更新最近邻表：给出是否等价于标准集合辨识/多模型鲁棒控制的判断（等价则写「无独立方法贡献」）（`two/08` 新增 §6；`conclusion.no_independent_method_contribution=true`）
- [x] `05_history_value_probe.md` 交付，给出继续/否决/证据不足的依据（§7 给出**否决**依据与局限）

## N4 R0 复审与下轮决定
- [x] 汇总完成/失败/未解决项与实际耗时（含仿真占机），结论链接原始证据或原文（`06` §1；抽查 `trial_manifest.json.wall_seconds`、`fair_compare/results/summary.json.resolved.*`、`history_probe/results/timing.json.*`、`baselines/results/summary.json.all_expected_observed` 均指向真实字段）
- [x] 明确三选一结论（继续/改候选方法/停止改题）；证据不足时给最小实验与 ≤2 周/42—70 h 追加预算（`06` §3 选「停止当前机制 P」并交 §9.2 讨论；§4 最小实验＝备份分支改「区间+扫掠净空」后闭环验证，≤2 周/42—70 h）
- [x] `06_r0_decision.md` 与 `00_README.md`（阶段索引/审查入口）交付
- [x] 更新导师备忘录，保留真实讨论状态、未编造认可（`11` §1/§5 全「未讨论」；§2/§5.3 非技术阻塞；无伪造反馈）
- [x] 只有「继续」且 R0 满足时才生成 R1 任务；未因目录齐全自动过关（`06` §5 声明本轮不生成 R1；`docs/指导书/stage_logs/two/publication_tasks.md` §3 无 R1 条目）

## 证据纪律与冻结（贯通）
- [x] `one/` 冻结 meta/trace 与 `src/` 生产源码未被修改（哈希/差异核验）（`sha256sum -c` 对 241 项 `src/scripts` 与 18 项冻结数据**全部 OK**）
- [x] 仿真全程 headless、无残留进程；新数据只写新目录（`pgrep gzserver/px4/mavros` 为 0；新目录 `brake_hold_v3/`、`fair_compare/`、`history_probe/`）
- [x] 结论标注证据级别；零违例只写「本批未观测到」；未引用 `all_survive`/`fair_brake_compare` 作证据（`03`—`06`、`00_README`、`08` §6、`11` §5；该词仅出现在「不得引用」声明处）

## 交付后补正（验收附注的处置）
- [x] 附注 1（用途仅在 manifest、manifest 生成晚于采集）→ 已在 `03` §4 澄清：用途由 run-id 约定**在采集前固定**，manifest 如实转录，`meta.json` 未回写
- [x] 附注 2（采集器脚本未纳入哈希基线）→ 已用 `brake_hold_test_v2.py` 的 SHA-256 与全部 v3 run 的 `meta.script_sha256` **完全一致**补齐冻结证据链，并记入 `03` §4
- [x] 附注 3（`common.py` docstring「同一备份函数」略强）→ 已精确为「同一执行器 + 共同备份语义；规则法内联同一备份目标下的判据」；改注释后 `summary.json` SHA-256 不变（`5c79cb73…`），结果未受影响