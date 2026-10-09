# 07 制动采集与指标口径 v2（前置补正 P1）

> 阶段：two　日期：2026-09-24
> 依据：`docs/指导书/stage_logs/two/publication_tasks.md` §2 **P1**；验收 `docs/指导书/stage_logs/one/09_acceptance_review.md` §3 **F2**；
> 接口契约 `experiments/paper_v3/protocols/execution_contract_v2.md`；规格 `.trae/specs/apply-two-prerequisite-corrections/` Task 2。
> 交付：协议 `brake_hold_protocol_v2.md`、采集器 `brake_hold_test_v2.py`、重算脚本 `brake_hold_metrics_v2.py`、
> 样例 `identification/brake_hold_v2/{smoke_01,drop_01}`。

---

## 1. v2 口径（相对 v1 的关键修订）

| 项 | v1 | v2 |
|---|---|---|
| 时基 | 未强制 `use_sim_time`，无 `/clock` 记录（9 个 run 中 4 个 false） | 强制 `/use_sim_time=true` 并订阅记录 `/clock`；另记 `time.monotonic()` 墙钟；动力学一律用仿真钟 |
| 时间戳 | MAVROS 位姿/速度共用"新鲜度"时间、无源时间戳 | 位姿/速度**分别**记源 header 时间戳 + 本机接收时间（仿真/墙钟）；Gazebo 记接收时刻 + 对应仿真时刻（精度 ≤ 一个仿真步长） |
| 发布记录 | 无 `seq` | `publish_log.csv` 每次发布记序号/值/发布时刻；`trace.csv` 带 `pub_seq` |
| 异常状态 | 仅 startup/物理/超时 | 新增**独立** `clock_jump`、`clock_stall`、`drop_frame`，不与物理失败混淆 |
| 指标 | `max_drift_m`（实为净位移，错名）、`tail_drift_m`（仅两点） | 拆为沿向最大外扩 / 水平最大位移 / 终态净偏移 / 尾段最大偏移 / 尾段最大速度，全部由日志重算 |
| 延迟命名 | `brake_latency_s` | `threshold_response_time_s`（阈值响应时间），声明含响应动态、≠纯延迟 |
| 进程管理 | 全局 `pkill -9 -f` | 会话进程组（`start_new_session` + `os.killpg`），只回收本会话进程 |
| 可追溯 | 无 | `meta.json` 含脚本 SHA-256、协议版本、发布者、终止原因 |

口径细节见 `experiments/paper_v3/protocols/brake_hold_protocol_v2.md`。

## 2. smoke_01 关键指标（由原始日志重算）

- run：`experiments/paper_v3/identification/brake_hold_v2/smoke_01`
- 命令：`longitudinal, speed=0.5 m/s, altitude=2.0 m, cruise_distance=3.0 m, settle_timeout=15s, tail_seconds=5s`
- 速度源：Gazebo 真值（评分配置）；位置源：Gazebo 真值；`use_sim_time=true`。
- 采集脚本 SHA-256：`4aa7e8dc34676a3b3f0c7dfb3cd3c7034e036a4eae22dbcdc308c69d6b250021`。

**重算命令**（结果与下表一致）：

```bash
python3 experiments/paper_v3/scripts/brake_hold_metrics_v2.py \
    --run-dir experiments/paper_v3/identification/brake_hold_v2/smoke_01
```

| 指标 | 值 | 说明 |
|---|---:|---|
| 触发时刻 `t_brake_cmd_sim` | 26.844 s | 命令时刻（仿真钟） |
| 停稳确认时刻 `settle_confirm_time_sim` | 30.444 s | `\|v\|<0.05` 连续 2.0 s（评分配置） |
| 沿向最大外扩 `along_max_expansion_m` | 0.2405 m | 沿运动方向(+X)的最大投影 |
| 水平最大位移 `horizontal_max_displacement_m` | 0.2406 m | 径向峰值（与前一行**分开**报告） |
| 终态净偏移 `terminal_net_offset_m` | 0.0567 m | 触发位置 → 停稳确认时刻位置 |
| 尾段最大偏移 `tail_max_offset_m` | 0.0899 m | 停稳确认时刻 → 记录末 |
| 尾段最大速度 `tail_max_speed_mps` | 0.0488 m/s | 停稳确认时刻 → 记录末 |
| 阈值响应时间 `threshold_response_time_s` | 0.18 s | 含响应动态，≠纯延迟 |
| 时基比 `sim_wall_ratio` | 0.999673 | 仿真时长/墙钟时长，≈1，无倍率失真 |

**触发/发布/停稳确认/尾段时刻可区分**：`trace.csv` 的 `phase` 列含
`presend → climb → hold → cruise → brake → tail` 全阶段；命令切换写在 `cmd_*`，制动后恒为保持点
`p_b=(2.49,−0.02,2.05)`；逐次发布在 `publish_log.csv`（本 run 1577 次，序号连续）；
`t_brake_cmd_sim` 与 `settle_confirm_time_sim` 由日志重算并可复现。

## 3. drop_01 丢帧样例的标记结果

- run：`experiments/paper_v3/identification/brake_hold_v2/drop_01`
- 构造方式：`--drop-frame-sim 2.0`——巡航过半后**合成**抑制数据源 2.0 仿真秒
  （记录为空 `sources=""`，共 51 拍）。这是为验证管线而人为构造的数据中断，**非真实网络丢包**。
- 标记结果：
  - `meta.status = "drop_frame"`，`meta.termination_reason = "drop_frame"`；
  - `meta.status_reason = "检测到数据源中断：51 拍无有效源"`；
  - 重算脚本给出 `status_source = "meta_independent"`——**独立状态**，**未**被误判为物理失败
    （物理指标另列：沿向最大外扩 0.2491 m、终态净偏移 0.0521 m、停稳确认 30.452 s）。

## 4. 时钟核验

- `use_sim_time=true`：smoke_01/drop_01 的 `meta.use_sim_time` 均为 `true`（`force_sim_time_ok=true`）。
- `/clock` 有记录：`clock.csv` 分别 9337/9343 条；相邻差 `d_clock ∈ [0.004, 0.028] s`，
  **无负值、无 >1.0 s 跳变**（`neg=0, big=0`），故两 run 均未触发 `clock_jump`/`clock_stall`。

## 5. 旧数据只附新分析、未改写原始 meta/trace（声明与核验）

**声明**：本轮**未**修改 `experiments/paper_v3/identification/brake_hold/*`（v1 旧数据）的任何
`meta.json`/`trace.csv`；也**未**修改 v1 采集器 `brake_hold_test.py` 与协议 `brake_hold_protocol_v1.md`。
v2 的采集/重算产物全部写入新目录 `identification/brake_hold_v2/`。
重算脚本对 v1 旧日志**只读**（`load_trace` 带列名兼容分支，可"附新分析"，但不写回原目录）。

**核验方式**（哈希比对，运行前后各一次）：

```bash
sha256sum experiments/paper_v3/identification/brake_hold/*/meta.json \
          experiments/paper_v3/identification/brake_hold/*/trace.csv | sort
```

16 个文件（8 run × {meta.json, trace.csv}）的 SHA-256 在本次 P1 执行前后**完全一致**，
证明旧 meta/trace 未被改写。

## 6. 诚实说明与局限

1. 丢帧样例为**合成抑制**（验证状态机），不代表真实链路丢包率；真实丢包率应在 N1 实测。
2. `start_sim.sh` 脚本**内部**自带启动清场（全局 `pkill`，属其自身行为）；按任务范围**不修改** `scripts/`。
   v2 采集器**自身**的回收限定在本会话：用 `start_new_session` 建立进程组，并对**启动前后做仿真进程 PID 差集**。
   实测发现 PX4 的 roslaunch 会经 `setsid` 让各节点成为独立会话首进程（`PGID=SID=自身 PID`），
   仅靠 `os.killpg` 会漏掉它们；故差集兜底逐个 PID 终止，**不使用全局 pkill**。
   两 run 的 `meta.cleanup_leaked_pids=[]` 且运行后系统无残留仿真进程，证明本会话进程被完整回收、不影响其他实验。
3. 阈值响应时间含响应动态，**不得**当作纯运输延迟；纯延迟须由 N1 单独模型辨识给出。
4. 本样例仅为走通 v2 全链路的**可行性检查**，不构成统计安全边界，也不用于冻结动力学参数。