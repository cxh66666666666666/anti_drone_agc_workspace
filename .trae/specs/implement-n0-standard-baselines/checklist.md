# Checklist

> 与 `spec.md` 的 Requirement 一一对应；逐项通过后在方框打勾。
> 核对方式：独立验收代理逐条对照**代码/运行输出**（`experiments/paper_v3/baselines/`、`results/*.json`、`02_standard_baselines.md`）核实，非转述文档自我声明。结论：**全部通过**。

## 文献精读（N0 文献项）
- [x] DDSF 的**输入-输出安全集、有限时域优化问题、假设（确定性/无噪声 LTI、持续激励）与局限**逐条列要点并标页码（`02_standard_baselines.md` §2.1；Definition 7 于 PDF 第 4 页经 `pdftotext` 抽查一致）
- [x] Layered Safety **Eq.19** 与 **Theorem 1**（含 Remark 1）要点正确，且写明「鲁棒项为跟踪误差补偿，非方向集合、非延迟项」（`02` §2.2）
- [x] WZ-2021 **模型假设、终端集 `S^t`、递归可行条件**要点正确；RTD/FaSTrack 仅列与本路线重合段落（`02` §2.3/§2.4）
- [x] 精读要点写入 `two/02_standard_baselines.md`，与 `two/08` 交叉引用且未改 `one/03`（§2 顶部交叉引用；`one/03` mtime 早于 N0 产物）

## 低阶闭环模型
- [x] `experiments/paper_v3/baselines/model.py` 提供定高二维、位置命令输入闭环模型，状态含 `p_k,v_k` 与命令历史（`model.py:97-107`）
- [x] 模型为「纯延迟 `n_L` 拍 + 一阶滞后 `α=1−exp(−dt/T)`」，速度由位置差分自洽，无未定义状态转移（`model.py:119-131`）
- [x] 所有量与参数给出单位与来源，且**明确标注合成/借用**；无单位混用（无 m 当 m/s）（`model.py:34-49`；文档 §3 单位纪律）
- [x] 提供可计算终端集 `Ω={净空 ≥ margin 且 |v| ≤ v_stop}`，并声明短时低速为经验观察、非已证明不变集（`model.py:146-152`）

## 共同备份与候选展开
- [x] `backup.py` 的备份目标为**当前实测水平位置** `p_k`，持续发布保持设定点并含限幅（对齐契约 §4）（`backup.py:22-24`；限幅为接口对齐声明，未在原型施加，已在文档 §5 诚实标注）
- [x] 每个候选展开为「**候选一拍 → 共同备份 → 保持**」并输出完整序列（`_eval_b1/_eval_b2` 调用 `bk.expand_candidate`，`filters.py:34,69`；`backup.py:43-47`）
- [x] 保存**上次有效备份**；超时/不可行时执行**剩余备份**并标 `used_last_valid_backup`（`backup.py:57-80`、`filters.py:134-141`；S3 实测 `true`）
- [x] 初态不支持备份时返回 `certificate_unavailable`，不输出伪造通过标签（`filters.py:142-148`）

## B1 / B2 / P
- [x] B1 为标称点预测（声明 `L_hat`，`E={0}`）+ 共同备份 + 逐点净空检查 + 终端条件（`filters.py:28-55`）
- [x] B2 为延迟增广状态 + 延迟区间 + 方向相关误差集合（`σ_E(n)`）与集合传播；鲁棒判据 `h(p) − σ_E(−∇h) ≥ margin`，核对 `−∇h` 方向（`filters.py:58-101`，`model.py:89-93`）
- [x] B2 与 B1 **共用同一备份函数**，差异仅落在延迟建模与不确定性处理（结构性），文档未写「只差一个 κ/标量集合」（文档 §4；`01_design_corrections.md` §2.1 E1/E2）
- [x] P **仅接口**（占位），本阶段无新网络、无大规模搜索（`filters.py:168-172`）

## 可运行入口与验收场景
- [x] `run_baselines.py` 为纯 Python（无 ROS、无第三方依赖）且可复算，结果写入 `baselines/results/`（import 仅标准库；两次运行 JSON 哈希一致）
- [x] **开阔**场景：B1/B2 接受正常候选（未被替换）（`s1_open.json`；`selected_kind=candidate`）
- [x] **危险**场景：危险候选被替换为共同备份，且**回放的已执行命令序列确为备份保持设定点**、轨迹进入 `Ω`（`s2_danger.json`：`executed_is_backup_hold_pk=true`、`executed_samples` 全为 `p_k`）
- [x] **缩短时域**场景：原可接受场景报 `certificate_unavailable`（`s3_short_horizon.json`，N=20）
- [x] **破坏终端条件**场景：出现**实际净空/裕量违例**或无法停稳候选被错误放行（`s4_margin_disabled.json`：B1 放行近墙候选，225 次裕量违例、不入 Ω）
- [x] 结论由**回放轨迹独立度量**（最小净空、违例计数、终态速度、是否落入 `Ω`）给出，**不只检查 `passed`/`safe` 标签**（`run_baselines.py:38-55,118-138`；S4 中 B1 证书 `valid` 但度量暴露违例）
- [x] 结果 JSON 含初态/命令/模型版本与参数/已执行命令序列/度量/`evidence_level=model-level`，重复运行一致（各 `results/*.json`）

## 交付文档
- [x] `two/02_standard_baselines.md` 含模型/单位表、文献要点、Legacy/B1/B2/P 差异、候选展开与备份规则、终端条件、可运行入口与基础结果（§1—§9）
- [x] 文档标注 `model-level` 证据级别与「非 PX4 实测」；结论链接到结果 JSON 与原文页码；未**引用** `all_survive`（仅出现于 §9「不得引用」处）