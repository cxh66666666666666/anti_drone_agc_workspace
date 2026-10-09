# 研究进度日志

> 本文档是 [publication_plan.md](publication_plan.md) 的配套日志，只记录**做了什么、产物在哪、Gate 结论**。
> v3 算法方案现为历史记录；当前方向见[改题决定](stage_logs/three/07_改题决定_终止后物理结局评测.md)。下方阶段 0—3 与 G0—G3 是 **v2 历史记录**，原定义见 [v2 存档](publication_plan_v2_archived.md)，不套用 v3 的 R0—R6 标准追溯改判。
> 最后更新：2026-09-25

## 改题：终止后物理结局评测（2026-09-25）

- [限额补读裁决](stage_logs/three/06_限额补读与命题裁决.md)已停止失效边界搜索论文主线；前三次算法候选均不再执行。
- [新方向决定](stage_logs/three/07_改题决定_终止后物理结局评测.md)：研究提前终止、几何代理与独立接触事件怎样影响 PX4/Gazebo 闭环避障的安全结局和方法比较。静态复算 2026-09-24 探索批次：A1 5/48 已记录几何碰撞，另 17/48 提前终止而缺后续物理结局；P 为 8/48 与 12/48；B2 为 15/48 与 6/48。旧结果不能确定全程物理碰撞排序。
- [最近邻原文矩阵](stage_logs/three/08_最近邻原文矩阵.md)：原文级 11 篇（含 Gymnasium §4.2、Pardo 2018、SVL、SONG、SoftVTBench、OOPSIEVERSE 等，已归档 PDF 与 SHA-256）＋摘要/文档级 7 项。判定「区分终止/截断」「截断=右删失」「成功/碰撞/超时分列」「几何代理 vs 接触」「任务完成≠安全执行」均已有先例，**不可作新意**；本轮未检索到把「软件截断后机体继续运动的物理尾段」当评测偏差做配对实证的工作（≠研究空白）。
- [尾段/接触探针报告](stage_logs/three/09_尾段接触探针报告.md)：给 `iris_3d_lidar` **派生**带 contact 传感器的机体（原模型未改，双 SHA-256 可复算），实现统一时间基的尾段/接触探针与独立复算脚本；空场景核验四条链路、受控接触核验事件源。实测：在净空 0.25 m 处截断、残余速度约 1.0 m/s 时，尾段外扩 0.30—0.40 m 并发生**真实接触**（2/2 复现，旧「截断即结算」口径会漏掉）；至少 1 个样本里估计位姿口径**虚报**碰撞而独立接触事件判无接触。5 个真实回合链路打通但**终止原因全为 `timeout`**，`shield_infeasible`/`perception_timeout` 本批未产出。
- 状态：**文献矩阵与探针最小验证均已交付**（2026-09-25）；尚无冻结配对批次（[`07`](stage_logs/three/07_改题决定_终止后物理结局评测.md) §4.3 未开始）、无导师反馈。缺口见 [`09`](stage_logs/three/09_尾段接触探针报告.md) §六。

## 阶段 three 独立审查与改题首轮核查（2026-09-25）

- `three/01` 的 T1 通过是作者自检；[独立研究决定](stage_logs/three/04_研究决定.md)发现 C1/C2 未建立所声称的完整包含/终端判定，可靠基线成本口径及 `B=560` 与回退 `980` 冲突。当前 DBDS 方案停止，T3 未开始，无 p95 收益或新安全试验数据。
- [改题可行性首轮核查](stage_logs/three/05_改题可行性检查.md)精读 S-TaLiRo、RL 汽车场景反例搜索、mfEGRA、NADE 四篇原文并保存指纹。泛题“更快找失效边界并估计风险”已有直接先例，**尚不能选为论文方法题目**；窄问题与导师意见仍待明确。
- [导师讨论提要](stage_logs/three/导师讨论提要.md)已准备，尚未发送或收到回复。当时拟议的失效边界窄问题后来也被[限额补读裁决](stage_logs/three/06_限额补读与命题裁决.md)止损；现行方向见上方改题记录。

## v3 原 §9.3 首轮验收与阶段 two 发布（2026-09-24）

- 产物：[stage_logs/one](stage_logs/one/00_README.md)；审查：[09_acceptance_review.md](stage_logs/one/09_acceptance_review.md)。**部分通过，R0 尚未通过**。
- 离线核验：45 个现存冻结文件哈希全部匹配、2 个预先注明的 checkpoint 仍缺失；CE1 六行制动统计重算一致。保留旧基线与探索数据，不重复布置已通过的基础审计。
- 前置补正：统一直接 PX4 与原 LQR 组合链路的接口表述；纠正 B1/B2/P 设计；修订制动指标/时基与未来回放协议；撤回无实际公平比较支撑的结论，校正文献和决策预算。
- 第二波任务已存档于 [阶段 two 任务书](stage_logs/two/publication_tasks.md)，含 P0—P4 补正与 N0—N4 任务；当前执行计划见 [第三波任务](stage_logs/three/第三波任务.md)。原 §9.3 全文保留在第二波任务书附录 A。
- 本次完成审查、离线复算和任务发布；未运行新飞行实验、未完成上述前置修订或新阶段实现，未宣告研究假设通过。

## v3 路线重审（2026-09-24）

- 已完成：核查项目源码、控制与实验报告、部分最新汇总及 5 篇本地论文原文；重写 [publication_plan.md](publication_plan.md)，保留旧版。
- 审查依据与限制：[publication_reassessment_20260924.md](publication_reassessment_20260924.md)。当前默认状态已为 38 维；旧 33 维记录保留历史语义。9 月 23 日新批次和 9 月 24 日选参报告已在重审中补充，不能仅用下方旧 G3 摘要代表最新实验情况。
- 新主线：执行滞后条件下的预测避障、方向相关误差范围和可执行制动备份。DQN 保留为候选生成器/基线，是否纳入主方法按独立收益决定。
- R0：**未验收**。本次完成指导书与证据审查，尚未完成穷尽的新颖性检索、标准预测安全过滤复现或新制动实验；本次未修改控制代码、未启动训练。
- 官网核验：访问获准后服务器返回 HTTP 403，当前投稿要求与收录状态仍待核实；不阻塞技术预实验。
- 二次审查：补充公开学术检索，核查 DDSF 与 Layered Safety 两篇原文；将“未知延迟本身创新”排除，主对照升级为方向相关延迟鲁棒方法。按用户每周 21—35 小时、有导师、无实物的条件补充工作量和六周竞争性预研决策。

## 阶段 0：期刊核验与范围冻结

- **Gate G0：No-Go（2026-09-09）**，任务 1、4 未闭环。
- 完成：
  - 文献矩阵 44 篇（近五年 ≥35 篇，仿真验证/可复现性 ≥5 篇）→ [文献矩阵.md](../Thesis/文献笔记/文献矩阵.md)
  - 状态/动作审计：固化为 33 维状态、6 个动作，禁用旧 21/5 描述 → 计划 §2.1
  - 研究定义与论文骨架 → 计划 §1.1、§4.2、§5.3–§5.5、§6.3
- 未闭环：期刊官网指南/模板/收录认定存档；导师书面确认（主线、作者顺序、投稿状态、硬件在环）。

## 阶段 1：控制接口与测量可信化

- **Gate G1：技术抽查项完成，总 Gate 暂 No-Go（2026-09-09）**，未闭环项为导师确认。
- 完成：
  - 实际速度链路：速度话题/ENU/时间戳核对通过，LQR 已订阅实际速度；三次恒速重复 −2.70±0.09%
  - 设定点响应：控制验证协议冻结 → [control_validation_protocol_v1.md](../../experiments/stage1/task2_setpoint_response/control_validation_protocol_v1.md)（`position_lead_time=0.80 s`；`a_brake=0.044 m/s²` 仅适用于当前低速仿真窗口）
  - 碰撞测量：几何碰撞代理，success/skim/pass 三类抽查通过，最大重算差 4.38e-07 m → [task3_geometry_smoke_report.md](../../experiments/stage1/task3_geometry_smoke/task3_geometry_smoke_report.md)
  - reset/trial：scenario_id / trial_id / trajectory_phase_s 追踪通过 → `test_trial_identity.py`
  - 指标自动计算：10/10 run 逐 trial 独立重算一致 → `metrics_utils.py`、`test_metrics_utils.py`
- 未闭环：仍不含 Gazebo 物理接触真值，论文只能写“几何碰撞代理/几何净空”。

## 阶段 2：场景与复现实验框架

- **Gate G2：技术条件通过（2026-09-12）**。
- 完成：
  - 冻结清单 train 50 / validation 12 / test 30，测试清单哈希 `115336d2…75f`，`manifests/.frozen` 已存在；统一协议 `stage2-baselines-v1` → [stage2_closeout_report.md](../../experiments/stage2/stage2_closeout_report.md)
  - 历史索引收录 11 个旧日志，全部标记 exploratory，不参与正式协议
  - 离线接口 smoke：30 run / 60 episode / 1 个带原因 reset failure（不入论文统计）

## 阶段 3：安全过滤器实现（进行中）

- **Gate G3：No-Go**，五项判据中第 1 项（危险候选被稳定过滤）未达标。
- 完成：
  - 修复 A1/P 绕过环境可行性层、`A_safe ⊄ A_feasible`
  - α 收敛为共享理论参数（A1/P 均 0.5）；空安全集改两级兜底
  - 新增诊断列 `filter_feasible_actions`
- 产物/证据 → [stage3_diagnosis_curve_regression.md](../../experiments/stage3_diagnosis_curve_regression.md)
- 未达标根因：`candidate_filtered` 在 96 episodes 中仅 2 步（0.03%），防护被可行性层吸收。
- 进行中：a1/p 已切 `action_interface=raw`（`run_stage3_batch.py` L73），待验证 `candidate_filtered_in_avoidance_rate ≥ 0.03`（判据见 `select_stage3_params.py`）；动作集无制动档待处置。
