# Tasks

> 范围：根目录 `docs/指导书/stage_logs/two/publication_tasks.md` §3 **N0**（补齐关键原文并实现标准 B1/B2 的模型级原型）。
> 依赖：P0（接口契约 `execution_contract_v2.md`）、P3（回放/评分口径 `prediction_replay_format_v2.md`）——**均已完成**；文献阅读可提前（原文 PDF 齐备，见 `two/08` §1.4）。
> 交付：`docs/指导书/stage_logs/two/02_standard_baselines.md`；代码/结果用 `experiments/paper_v3/baselines/`（新目录，不改旧 one 数据与 `src/` 生产源码）。
> 证据级别：全部 `model-level`；参数为合成/借用值并明确标注，**不得**表述为 PX4 实测。
> 命名避让：N0—N4 占用 `two/00` 与 `02`—`06`，本项即 `02`。

- [x] Task 1: 关键原文精读要点（N0 文献项）
  - [x] SubTask 1.1: 用 `pdftotext` 精读 **DDSF**（`docs/Thesis/文献/预测安全过滤|可达集/DDSF_Bajelani_arXiv2309.00189.pdf`）的**输入-输出安全集定义、有限时域优化问题、假设（确定性/无噪声 LTI、持续激励）与局限**（§II/§III 及后文），逐条标注页码/小节。
  - [x] SubTask 1.2: 精读 **Layered Safety** 的 **Eq.19**（`∂h/∂t ≥ −αh + (1/ε)‖∇h‖²`）与 **Theorem 1**（跟踪 CLF 条件保证）及 Remark 1，核对「鲁棒项为跟踪误差补偿、非方向集合、非延迟项」。
  - [x] SubTask 1.3: 精读 **WZ-2021** 的**模型假设、终端集 `S^t`（Assumption 4.2）、递归可行（§4.2）** 条件；**RTD/FaSTrack** 只核对与本路线直接重合的段落（RTD 的「no delay in braking actuation」注记、FaSTrack 的 TEB/失配界定），不要求复现全部方法。
  - [x] SubTask 1.4: 把要点整理为 `two/02_standard_baselines.md` 的「文献精读要点」一节，与 `two/08` 差异表交叉引用，不重复整表、不修改 `one/03`。

- [x] Task 2: 低阶定高二维闭环模型（含单位表）
  - [x] SubTask 2.1: 新建 `experiments/paper_v3/baselines/model.py`：定高二维、位置命令输入；状态 `p_k,v_k` + 命令历史；模型为「纯延迟 `n_L` 拍 + 一阶滞后 `α=1−exp(−dt/T)`」，速度由位置差分自洽。
  - [x] SubTask 2.2: 参数全部**显式命名并标注来源**（`dt`、`T`、`L`、方向误差半轴等），标注为**合成/借用**；禁止把位置偏移（m）当速度（m/s）。
  - [x] SubTask 2.3: 提供几何净空与**可计算终端集** `Ω = {净空 ≥ margin 且 |v| ≤ v_stop}`；声明短时低速只是经验停稳观察，不变性主张仅限线性模型局部范围。

- [x] Task 3: 共同备份与候选展开（所有方法共用）
  - [x] SubTask 3.1: 新建 `experiments/paper_v3/baselines/backup.py`：共同备份函数——目标置为当前实测水平位置 `p_k`、持续发布保持设定点、含限幅（对齐 `execution_contract_v2.md` §4）。
  - [x] SubTask 3.2: 实现候选展开「**候选一拍 → 共同备份 → 保持**」，输出完整展开序列；保存**上次有效备份**；超时/不可行时执行**剩余备份**并标 `used_last_valid_backup`。
  - [x] SubTask 3.3: 初态不支持备份（时域内到不了 `Ω`）时返回 `certificate_unavailable`，不输出伪造的通过标签。

- [x] Task 4: B1 标称基线与 B2 鲁棒基线（＋P 仅接口）
  - [x] SubTask 4.1: 新建 `experiments/paper_v3/baselines/filters.py` 的 **B1**：标称点预测（声明 `L_hat`，`E={0}`）+ 共同备份，逐点净空检查 + 终端条件。
  - [x] SubTask 4.2: 同文件的 **B2**：延迟增广状态 + 延迟区间 `[L_min,L_max]` + 方向相关误差集合（支持函数 `σ_E(n)`）与**集合传播**；鲁棒判据 `h(p) − σ_E(∇h) ≥ margin`（核对 `−∇h` 方向）；**共用同一备份函数**。
  - [x] SubTask 4.3: **P 仅接口**：函数签名 + 占位（返回明确标注的占位结果/未实现），本阶段不引入新网络或大规模搜索。
  - [x] SubTask 4.4: 差异声明：B1↔B2 为**结构性差异**（中心模型即延迟增广+方向鲁棒），不得写「只差一个 κ 或一个标量集合」。

- [x] Task 5: 可运行入口与四个验收场景（按实际回放度量）
  - [x] SubTask 5.1: 新建 `experiments/paper_v3/baselines/run_baselines.py`（纯 Python、无 ROS）：跑四个场景并写结果 JSON 到 `experiments/paper_v3/baselines/results/`。
  - [x] SubTask 5.2: **场景 1 开阔**：B1/B2 接受正常候选（未被替换）。
  - [x] SubTask 5.3: **场景 2 危险**：危险候选被替换为共同备份，且**回放的已执行命令序列确为备份保持设定点**、轨迹进入 `Ω`。
  - [x] SubTask 5.4: **场景 3 缩短时域**（`N` 过小）：暴露失败——原可接受场景报 `certificate_unavailable`。
  - [x] SubTask 5.5: **场景 4 破坏终端条件**（禁用 `Ω` 检查）：暴露失败——出现实际净空/裕量违例或无法停稳的候选被错误放行。
  - [x] SubTask 5.6: 评分**独立于返回值标签**：由回放轨迹计算实际最小净空、违例计数、终态速度、是否落入 `Ω`；结果 JSON 含初态/命令/模型版本与参数/已执行命令序列/四类度量/`evidence_level=model-level`，重复运行一致。

- [x] Task 6: 交付文档 `two/02_standard_baselines.md`
  - [x] SubTask 6.1: 写模型/单位表（量、单位、来源、「合成/借用」标注）、文献精读要点、Legacy/B1/B2/P 方法差异。
  - [x] SubTask 6.2: 写候选→备份→保持、备份保存/超时降级/证书不可用、可计算终端条件与其「非已证明不变集」声明。
  - [x] SubTask 6.3: 写可运行入口（命令）与基础结果（四场景的可复算数值），标注 `model-level` 证据级别与「非 PX4 实测」。
  - [x] SubTask 6.4: 验证：文档结论链接到结果 JSON 与原文页码；不引用 `all_survive`；不写道具化过强表述。

# Task Dependencies

- Task 2 依赖 Task 1 的无（可并行；文献项独立）。
- Task 3 依赖 Task 2（展开需模型与终端集）。
- Task 4 依赖 Task 3（B1/B2 共用备份）。
- Task 5 依赖 Task 4（场景需 B1/B2 与备份）。
- Task 6 依赖 Task 1—5（文档汇总）。
- Task 1 可与 Task 2 并行；Task 2→3→4→5 为串行链。