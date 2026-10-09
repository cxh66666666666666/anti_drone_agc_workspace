# 12 阶段 two 审查入口：P0—P4 完成情况与交付物索引

> 日期：2026-09-24　阶段：two（统一执行接口、落实标准预测基线并检验剩余问题）
> 依据：[阶段 two 任务书](publication_tasks.md) §2（P0—P4）、§4（预算）；验收 [one/09](../one/09_acceptance_review.md)。
> 用途：**本轮审查入口**。先看本文件与 [`09_research_decision_v2.md`](09_research_decision_v2.md)，再随机复算关键结果；**不以 `all_survive`、退出码 0 或人工勾选单独认定研究通过**。

## 1. 前置补正 P0—P4 完成情况

| 项 | 内容 | 预算 | 状态 | 主要证据 |
|---|---|---:|---|---|
| **P0** | 统一执行接口、备份与方法标签 | 5—8 h | **完成** | [`execution_contract_v2.md`](../../../../experiments/paper_v3/protocols/execution_contract_v2.md)；[`01_design_corrections.md`](01_design_corrections.md) §1（标签）＋ §3（接口结论修订） |
| **P1** | 修正制动采集、时间与指标口径 | 7—10 h | **完成（小样例级）** | [`07_measurement_v2.md`](07_measurement_v2.md)；`protocols/brake_hold_protocol_v2.md`；`scripts/brake_hold_test_v2.py`、`brake_hold_metrics_v2.py`；`identification/brake_hold_v2/{smoke_01,drop_01}` |
| **P2** | 纠正三反例与公平比较的证据等级 | 2—3 h | **完成** | [`01_design_corrections.md`](01_design_corrections.md) §B；[`FAIR_BRAKE_COMPARE_RETRACTION.md`](../../../../experiments/paper_v3/mechanisms/results/FAIR_BRAKE_COMPARE_RETRACTION.md) |
| **P3** | 修订未来预测/回放协议 | 5—8 h | **完成** | `protocols/prediction_replay_format_v2.md`（不覆盖 v1） |
| **P4** | 校正研究决策与来源材料 | 3—5 h | **完成** | 本目录 [`08_nearest_neighbor_table_v2.md`](08_nearest_neighbor_table_v2.md)、[`09_research_decision_v2.md`](09_research_decision_v2.md)、[`10_sources_archival.md`](10_sources_archival.md)、[`11_advisor_memo_v2.md`](11_advisor_memo_v2.md)、本文件 |

> 说明：P1 已达「能由原始日志重算全部关键指标 + 丢帧被独立标记」的**可行性检查**级（`07` §2—§3），**不是统计安全边界**；批量补采属 **N1**。

## 2. 交付物索引（路径 → 内容 → 对应 P 项）

| 路径 | 内容 | 对应 P 项 |
|---|---|---|
| `experiments/paper_v3/protocols/execution_contract_v2.md` | 唯一主接口、量的清单、单位纪律、共同备份、接管/释放规则 | P0 |
| `docs/指导书/stage_logs/two/01_design_corrections.md` | §A 方法标签/旧表述纠正/接口结论修订；§B 三反例与公平比较证据等级更正 | P0、P2 |
| `experiments/paper_v3/protocols/brake_hold_protocol_v2.md` | 制动采集 v2 协议（时基/时间戳/发布序号/状态/指标口径） | P1 |
| `experiments/paper_v3/scripts/brake_hold_test_v2.py` | v2 采集器（会话进程管理、发布日志、独立状态机） | P1 |
| `experiments/paper_v3/scripts/brake_hold_metrics_v2.py` | 指标重算脚本（沿向/水平/终态/尾段/阈值响应时间） | P1 |
| `experiments/paper_v3/identification/brake_hold_v2/{smoke_01,drop_01}` | 新样例原始 trace/发布日志/clock/meta | P1 |
| `docs/指导书/stage_logs/two/07_measurement_v2.md` | P1 报告（口径对照、smoke/drop 指标、时钟核验、未改旧数据声明） | P1 |
| `experiments/paper_v3/mechanisms/results/FAIR_BRAKE_COMPARE_RETRACTION.md` | 撤回 `all_survive=true` 的结论引用（含原文件 SHA-256） | P2 |
| `experiments/paper_v3/protocols/prediction_replay_format_v2.md` | 未来预测/回放 v2：候选复算 / 实际输入回放 / 模型分叉三分；评分侧隔离 | P3 |
| `docs/指导书/stage_logs/two/08_nearest_neighbor_table_v2.md` | 差异表 v2（B2 主对照；WZ-2018/2021 身份纠正；**五篇最近邻正文＋ARMTD 相邻参考均已回填，含 Layered Safety Eq.19/Theorem 1**） | P4 |
| `docs/指导书/stage_logs/two/09_research_decision_v2.md` | 决策节奏复位、预算 84—140 h、接口更新 | P4 |
| `docs/指导书/stage_logs/two/10_sources_archival.md` | 官网来源存档清单 + AIGC 政策原句摘录 | P4 |
| `docs/指导书/stage_logs/two/11_advisor_memo_v2.md` | 导师备忘录 v2（状态列全「未讨论」；非阻塞项声明） | P4 |
| `docs/指导书/stage_logs/two/12_p0_p4_summary.md` | 本文件（审查入口） | P4 |
| `docs/Thesis/文献/预测安全过滤\|可达集/FaSTrack_Chen2021_arXiv2102.07039.pdf` | FaSTrack 原文（Chen 等 2021，arXiv:2102.07039v1，16 页；SHA-256 `ad2b23bf…3718`） | P4 证据 |
| `docs/Thesis/文献/预测安全过滤\|可达集/RTD_Vaskov2019_arXiv1902.01786.pdf` | RTD 原文（Vaskov 等 2019，arXiv:1902.01786v2，ACC 2019，8 页；SHA-256 `a6c3f8f3…ba13`） | P4 证据 |
| `docs/Thesis/文献/预测安全过滤\|可达集/ARMTD_Holmes2020_arXiv2002.01591.pdf` | ARMTD 原文（Holmes 等 2020，arXiv:2002.01591v2，机械臂，14 页；**相邻参考**；SHA-256 `fc74fdda…3313`） | P4 证据 |
| `docs/Thesis/文献/预测安全过滤\|可达集/WZ2018_Wabersich_arXiv1803.08552.pdf` | WZ-2018 原文（arXiv:1803.08552v6，7 页；SHA-256 `0acf555d…be36`） | P4 证据 |
| `docs/Thesis/文献/预测安全过滤\|可达集/WZ2021_Wabersich_arXiv1812.05506.pdf` | WZ-2021 原文（arXiv:1812.05506v4，17 页；SHA-256 `f1604802…8f52`） | P4 证据 |
| `docs/Thesis/文献/预测安全过滤\|可达集/DDSF_Bajelani_arXiv2309.00189.pdf` | DDSF 原文（arXiv:2309.00189v1，8 页；SHA-256 `9958fdca…d126`） | P4 证据 |
| `docs/Thesis/文献/预测安全过滤\|可达集/LayeredSafety_Yamaguchi2026_arXiv2603.00338.pdf` | Layered Safety 原文（arXiv:2603.00338v2，8 页；含 Eq.19/Thm1；SHA-256 `ee68d144…8afa`） | P4 证据 |
| `docs/Thesis/期刊要求/论文模板.pdf` | 官方论文模板（3 页，2026-09-09 下载；SHA-256 `546dc4ff…2c74`） | P4 来源 |
| `docs/Thesis/期刊要求/论文修改细则.md` | 论文修改细则正文（2026-09-09 下载；SHA-256 `cf164b1a…2ddb`） | P4 来源 |
| `docs/Thesis/期刊要求/保密审查证明要求及模板.docx` | 保密审查证明要求及模板（2026-09-09 下载；SHA-256 `de3c7126…49d9`） | P4 来源 |

## 3. 未完成项与阻塞

**未完成项（如实列出）**：
1. **模板与费用（更新）**：模板正文**已由本地留存文件覆盖**（`论文模板.pdf`/`论文修改细则.md`/`保密审查证明要求及模板.docx`，**2026-09-09 下载**，见 [`10`](10_sources_archival.md) §6）；**具体费用金额由用户决定不跟进（2026-09-24）**，不再列为待办。**仍待核验** = ARMTD/Layered Safety 发表处、Kousik 移动机器人 RTD 原始题名、公开代码链接（**用户已检索未找到，2026-09-24**）；**学校认定未核实**。
2. **公平制动机制对照（N2）未做**：三反例仍缺闭环/模型级的**同一执行器 + 同一备份**对照；`all_survive=true` **不可**作证据（`01` §B.4、撤回说明）。
3. **执行历史细化价值（N3）未验**：B2 之上的历史分支筛选原型尚未实现。
4. **最近邻原文状态（更新）**：**五篇最近邻正文＋相邻参考均已读原文** = FaSTrack（Chen 2021）、RTD（Vaskov 2019）、WZ-2018（arXiv:1803.08552v6）、WZ-2021（arXiv:1812.05506v4）、DDSF（arXiv:2309.00189v1）、Layered Safety（arXiv:2603.00338v2，含 **Eq.19/Theorem 1**）、ARMTD（Holmes 2020，**相邻参考**）；均已回填 [`08`](08_nearest_neighbor_table_v2.md) §2/§2.1/§2.2。**发表处**：FaSTrack=*IEEE TAC* 66(12):5861–5876, 2021（用户提供）、WZ-2018=CDC 2018, pp. 7130–7135、WZ-2021=*Automatica* 129:109597 **均确认**；**仍待核验**=ARMTD/Layered Safety 发表处、Kousik 移动机器人 RTD 原始题名、公开代码链接（**用户已检索未找到，2026-09-24**）。
5. **制动数据面窄**：P1 仅 smoke/drop 各 1 次；0.5/1.0 m/s、直线档、2.0 m 高、无障碍（批量与纵/横×速度矩阵属 N1）。

**阻塞**：**未发现技术阻塞项**；导师回复、学校认定、未明确费用均**不作为技术阻塞**（`docs/指导书/stage_logs/two/publication_tasks.md` §2 P4 验收、`11` §2）。

## 4. 明确声明：**未开始 N0—N4**

> 本阶段仅完成前置补正 **P0—P4**。**N0（标准 B1/B2 模型级原型）、N1（选定接口补采辨识与独立验证）、N2（可证伪的公平制动对照）、N3（执行历史细化价值探针）、N4（R0 复审）** 均**尚未开始/未执行**；`docs/指导书/stage_logs/two/publication_tasks.md` §3 已定义，本文件**不将任何 N 任务写成已完成**。

## 5. 证据引用纪律声明

- 本文件及本轮 `two/` 文档**未引用** `experiments/paper_v3/mechanisms/results/fair_brake_compare.json` 的 **`all_survive=true` 作为实验通过**；该结论已按 P2 撤回（`01` §B.4、撤回说明）。
- 相关实验结论只按替代证据等级引用（`01` §B.5）：CE1 = 仿真实测＋离线判定；CE2 = 模型级几何计算；CE3 = 合成玩具模型；公平比较 = **未验证**。