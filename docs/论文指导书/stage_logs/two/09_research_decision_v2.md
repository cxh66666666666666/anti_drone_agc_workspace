# 09 研究决策 v2：决策节奏复位、预算改对与接口更新（阶段 two，P4）

> 日期：2026-09-24　阶段：two　依据：[阶段 two 任务书](publication_tasks.md) §1、§2 **P4**、§4；验收 [F5](../one/09_acceptance_review.md)。
> 修订对象：v1 [`../one/06_research_decision.md`](../one/06_research_decision.md)（**原文保持不动，旧文档冻结**）。
> 配套：接口契约 [`execution_contract_v2.md`](../../../../experiments/paper_v3/protocols/execution_contract_v2.md)、设计补正 [`01_design_corrections.md`](01_design_corrections.md)。

## 1. 决策节奏复位（纠正 v1）

- **R0 决策仍在「六周竞争性预研预算结束时」进行**，**不顺延到第三个月**再看创新是否成立（`publication_plan.md` §9.2；`docs/指导书/stage_logs/two/publication_tasks.md` §1「不以 P 胜出为验收条件」）。
- **本轮（4 周）不做**：确认性测试、**不建确认集**、**不锁创新**、不大规模训练 RL（`docs/指导书/stage_logs/two/publication_tasks.md` §1、§4）。
- v1 把 R0 决策与确认集/主结果安排到第 2—3 个月（`one/06` §2.2），**偏离**「第六周先决定是否值得研究」，**该安排作废**（验收 F5）。

## 2. 预算改对（纠正 v1）

- **本轮口径 = 4 周 × 每周 21—35 h = 84—140 h**，与 `docs/指导书/stage_logs/two/publication_tasks.md` §4 表一致：

| 工作 | 主动投入 |
|---|---:|
| P0—P4 前置补正 | 22—34 h |
| N0—N4 新任务 | 52—82 h |
| 回归修正、导师讨论及不可预见问题余量 | 10—24 h |
| **合计** | **84—140 h** |

- **明确作废**：v1（`one/06` §2.2）的「第 1 月 110—180 h、第 2 月 110—180 h、第 3 月 55—95 h」**不成立**——它超过每周 21—35 h 对应的常规四周上限，且把 R0 决策与确认集前置到第 3 个月（验收 F5）。旧预算**不作为执行依据**。
- 本轮建议节奏（`docs/指导书/stage_logs/two/publication_tasks.md` §4）：第 1 周完成前置项（P0—P4）与 **N0**；第 2 周 **N1**；第 3 周 **N2**；第 4 周 **N3/N4**。达到上限时**先提交真实未完成项**，不省略强基线、不虚构通过。

## 3. 接口决策更新

### 3.1 选定的研究接口

- 本阶段选用的**唯一主接口**为 [`execution_contract_v2.md`](../../../../experiments/paper_v3/protocols/execution_contract_v2.md) 选定的「**直接 PX4 位置设定点台架**」：
  - 通道 `/mavros/setpoint_position/local`（`geometry_msgs/PoseStamped`，`frame_id="map"`），由台架脚本在 OFFBOARD 下 **50 Hz 独占发布**（契约 §1.1）。
- **`Legacy` / `B1` / `B2` / `P` 全部运行在同一接口与同一共同备份上**（契约 §1、§4；`01_design_corrections.md` §1）。

### 3.2 该接口替换原 LQR 外环，必须重新辨识

- 原任务链路（`one/01` §3）为：任务路径 → 候选偏移/局部控制 → **LQR 外环**（`LQRTrackerNode`）→ 位置设定点 → PX4 内环 → 机体；LQR 与台架发布**同一话题**，但**来源不同**（契约 §1.2）。
- 本台架**不启动 LQR/感知节点**，直接把位置设定点写入 PX4（`brake_hold_test.py:9-15`），因此**替换**了原 LQR 外环，**必须重新辨识**。
- **不得**沿用原 LQR＋PX4 组合的延迟/滞后辨识值（`one/01` §3.7 的纯延迟 0.24/0.24/0.28 s、时间常数 0.85/0.85/0.80 s）到本台架；也**不得**宣称本台架已验证原 LQR＋PX4 组合系统（契约 §1.2）。
- **制动语义区别**：原 LQR 链路是 `speed_scale=0 → 冻结参考相位`；本台架是**把参考重置到触发拍实测位置** `p_b`（契约 §1.3）。二者**不可互替**——v1（`one/06` §1.3）把二者混为「组合闭环」是错误，已修订。

### 3.3 第六周前的复核点（保留）

出现以下任一情形，在**第六周前**改接口结论并重新辨识（接口选择本身属"本阶段选用的研究接口"，非既成事实）：
- 台架在更宽速度档/转弯下**延迟/滞后不可重复**（当前仅 0.5/1.0 m/s 直线档、2.0 m 高、无障碍）；
- 位置台架**无法在不切模式的情况下实现必要制动**；
- 误差范围过宽，使**几乎所有通道不可行**（指导书 §11）。

## 4. 与 P4 验收的对应

- 版本差异表/备忘录**无 B2/P 标签混淆**、无过强证据表述：见 [`08_nearest_neighbor_table_v2.md`](08_nearest_neighbor_table_v2.md)、[`11_advisor_memo_v2.md`](11_advisor_memo_v2.md)。
- **当前任务预算与 21—35 h/周一致**（本文件 §2 = 84—140 h）。
- **导师回复、学校认定、未明确费用不作为技术阻塞项**（`docs/指导书/stage_logs/two/publication_tasks.md` §2 P4 验收；详见 `11`）。