# 03 N1 小规模辨识与独立验证

> 阶段：two　日期：2026-09-25
> 依据：`.trae/specs/execute-two-n1-n4/spec.md`「N1 小规模辨识与独立验证」；`tasks.md` Task 1。
> 接口契约 [execution_contract_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/protocols/execution_contract_v2.md)；
> 采集/指标口径 [07_measurement_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/07_measurement_v2.md)。
> 数据/代码目录：`experiments/paper_v3/identification/brake_hold_v3/`（新目录）、辨识脚本
> `experiments/paper_v3/identification/identify_v3.py`。

---

## 1. 目的与证据级别

在**选定接口**（直接 PX4 位置设定点台架：`/mavros/setpoint_position/local` + OFFBOARD）上补采小规模辨识数据，
按**整次运行**划分拟合/校准/验证，辨识**正确位置接口的低阶标称模型**（速度通道一阶滞后 `T` + 纯延迟区间
`[L_min,L_max]`；位置通道有效增益/滞后），并在**独立验证集**做多步回放与制动峰值/回摆检查，
对照 `Legacy` 与 `B2` 所需范围。

- 证据级别：`sim-observation`（Gazebo/PX4 SITL 无头仿真实测，非真实飞行）。
- 区分口径：本报告同时给出**实测/仿真观测**结论与**模型级**推导（回放预测、区间包络），
  模型级结论不冒充实际飞行观测。
- 措辞纪律：本批**未观测到**时钟异常/丢帧/未停稳；**不得**据此写「已证明安全」。旧 `0.19 s` 响应时间
  **不是**纯延迟，见 §6。

## 2. 时基与真值权限

- 全程 `use_sim_time=true`，动力学一律用**仿真钟**；`/clock` 逐条落盘 `clock.csv`，另记 `time.monotonic()` 墙钟。
- **真值权限（评分侧）**：Gazebo `/gazebo/model_states`（`gz_*`）**只用于评分侧**（overshoot/回摆）。
- **控制侧**：MAVROS 位姿/速度（`mav_*`）为控制侧可见信息，**拟合与回放比对只用控制侧**。
- 真值时间戳为**接收时刻**（≈仿真时刻，误差 ≤ 一个仿真步长），见 `meta.gazebo_sync_precision`。

## 3. 烟雾样例（先烟雾后批量）

先跑 2 个烟雾，逐项核验通过后才批量：

| run | direction | speed | meta.status | use_sim_time | clock 负值/>1s 跳变 | publish 序号 | trace 相位 | leaked PID |
|---|---|---|---|---|---|---|---|---|
| `smoke_long_01` | longitudinal | 0.5 | ok | true | 0 / 0 | 1..1569 连续 | presend→climb→hold→cruise→brake→tail | [] |
| `smoke_lat_01` | lateral | 0.5 | ok | true | 0 / 0 | 1..1564 连续 | presend→climb→hold→cruise→brake→tail | [] |

烟雾全部通过；**未出现** `clock_jump`/`clock_stall`/`drop_frame`/超时，无需修正调用方式。

## 4. Trial 矩阵与用途预分配

- 矩阵：`{longitudinal, lateral} × {0.5, 1.0 m/s} × {r1, r2, r3}` ＝ **12 个新 trial**。
- 用途为**整次运行预分配**（whole-run split）：`r1=fit`、`r2=calib`、`r3=valid`；
  **同一次运行的 trace 不按时间行随机分割**，不跨用途。清单见
  [`trial_manifest.json`](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/identification/brake_hold_v3/trial_manifest.json)
  （含 `run_id/direction/speed_cmd/purpose/run_dir/meta_sha256/status/wall_seconds`）。
- **用途在采集前即由 run-id 约定固定**（`r1/r2/r3` 分别对应 `fit/calib/valid`），
  采集完成后由 `trial_manifest.json` **如实转录**；各 run 的 `meta.json` 由采集器原样写入、
  **未被回写篡改**（保留原始采集记录）。采集器 `brake_hold_test_v2.py` 的 SHA-256 为
  `4aa7e8dc…b250021`，与全部 v3 run 的 `meta.script_sha256` **完全一致**，证明采集期间脚本未变。
- 命令过程同时提供**斜坡**（巡航匀速前进）与**制动激励**（设定点回退到实测位置并保持）。
- 12 个 trial `meta.status` **全为 `ok`**；`clock.csv` 相邻差 `d_clock ∈ [0.004, 0.056] s`（无负值、无 >1.0 s 跳变）；
  `publish_log.csv` 序号 1..N 连续；`meta.cleanup_leaked_pids` 全为 `[]`。**未出现**异常 run。

| run_id | direction | speed_cmd | purpose | status | wall_s |
|---|---|---:|---|---|---:|
| long_0p5_r1 / r2 / r3 | longitudinal | 0.5 | fit / calib / valid | ok | ~50 |
| long_1p0_r1 / r2 / r3 | longitudinal | 1.0 | fit / calib / valid | ok | ~50 |
| lat_0p5_r1 / r2 / r3 | lateral | 0.5 | fit / calib / valid | ok | ~50 |
| lat_1p0_r1 / r2 / r3 | lateral | 1.0 | fit / calib / valid | ok | ~50 |

## 5. 命令速度 / 制动实际初速 / 巡航稳态（三者分开报告）

命令速度＝`args.speed`；制动实际初速＝`t_brake_cmd` 时**实测**速度；巡航稳态＝巡航**后段 1.0 s** 实测速度均值。
按速度档分别报告（fit/calib 8 run 中位数）：

| 命令速度档 | 巡航稳态实测（控制侧） | 制动实际初速实测（控制侧） |
|---:|---:|---:|
| 0.5 m/s | 0.493 m/s | 0.491 m/s |
| 1.0 m/s | 1.012 m/s | **1.085 m/s** |

**对 1.0 档约 1.10 m/s 的解释**（**不**改标签为「1.0 m/s 实测」）：1.0 档巡航行程 3 m ⇒ 巡航时长仅 **3 s**，
系统**尚未进入稳态**且速度环存在**响应超调**——制动触发瞬间实测速度达 1.075–1.105 m/s（> 命令 1.0），
随后巡航稳态回到 1.004–1.032 m/s。故 1.10 m/s 是**制动实际初速偏高的瞬态现象**，不是命令速度，也非稳态值。

## 6. 辨识结果与区间（只用 fit/calib）

模型：速度通道 `v(t)=v_ss·exp(-(t-t_brake-L)/T)`（`el≤0` 段保持 `v_ss`）；位置通道有效增益 + 有效滞后。
减速段用**最小二乘指数拟合**求 `T`；纯延迟给**区间**（由制动命令时刻到速度首次显著下降的**采样间隔**给出，
阈值 `v_0 − max(3σ_noise, 5%v_0)`，σ_noise 取巡航尾段噪声，采样周期 0.02 s）。

- `T`（fit/calib 中位）＝ **0.300 s**；逐 run ∈ [0.284, 0.317] s。
- 纯延迟区间 `[L_min, L_max]` ＝ **[0.18, 0.30] s**（区间，非单点）；逐 run 见下。
- 位置通道：有效增益 `0.983`（≈1）、有效滞后 `1.17 s`（巡航斜坡下累计跟随偏差）。

| run_id | v_brake_init(mav) | v_cruise_ss(mav) | T (s) | [L_min,L_max] (s) | overshoot(gz) (m) | 回摆速度(mav) (m/s) |
|---|---:|---:|---:|---|---:|---:|
| long_0p5_r1 | 0.451 | 0.458 | 0.303 | [0.20,0.22] | 0.278 | 0.194 |
| long_0p5_r2 | 0.482 | 0.487 | 0.314 | [0.18,0.20] | 0.252 | 0.178 |
| long_1p0_r1 | 1.093 | 1.025 | 0.285 | [0.28,0.30] | 0.567 | 0.355 |
| long_1p0_r2 | 1.078 | 1.004 | 0.284 | [0.28,0.30] | 0.537 | 0.343 |
| lat_0p5_r1 | 0.508 | 0.514 | 0.305 | [0.20,0.22] | 0.261 | 0.161 |
| lat_0p5_r2 | 0.499 | 0.500 | 0.315 | [0.18,0.20] | 0.289 | 0.170 |
| lat_1p0_r1 | 1.087 | 1.011 | 0.297 | [0.26,0.28] | 0.558 | 0.321 |
| lat_1p0_r2 | 1.082 | 1.013 | 0.296 | [0.28,0.30] | 0.581 | 0.329 |
| long_0p5_r3 / long_1p0_r3 | 0.488 / 1.075 | 0.493 / 1.013 | 0.308 / 0.299 | [0.18,0.20] / [0.24,0.26] | 0.244 / 0.544 | 0.173 / 0.339 |
| lat_0p5_r3 / lat_1p0_r3 | 0.510 / 1.105 | 0.518 / 1.032 | 0.317 / 0.288 | [0.18,0.20] / [0.28,0.30] | 0.222 / 0.638 | 0.142 / 0.362 |

（*r3 为 valid，仅供对照展示；不参与拟合。）

**按速度档分离**：0.5 档 `T=0.310 s`、`L∈[0.18,0.22]`；1.0 档 `T=0.291 s`、`L∈[0.26,0.30]`。
结论：纯延迟与速度正相关（1.0 档更大），`T` 略小；`B2` **须用区间覆盖两档 + 方向差异**，不能用单点（见 §8）。

## 7. 独立验证集（valid = *r3）多步回放与峰值/回摆

用**实际发布命令序列**（`cmd_*`/`publish_log`）驱动标称模型**回放**，与**控制侧 MAVROS** 逐样本比较；
巡航段误差在**稳态窗口**内评估（避开起步加速瞬态，该瞬态不属低阶标称模型界定范围）。

| 指标（valid 4 run，mean/max） | 值 | 口径 |
|---|---:|---|
| 位置多步误差·巡航（稳态窗） | 0.061 / 0.085 m | mean/max of per-run mean/max |
| 位置多步误差·制动**衰减-峰值段** | 0.046 / 0.085 m | `t_brake`→首个局部极小 |
| 位置多步误差·制动**含回摆全段** | 0.337 / 0.557 m | 低阶模型无回摆 ⇒ 峰值后误差 |
| 速度多步误差·巡航（稳态窗） | 0.062 / 0.114 m/s | — |
| 速度多步误差·制动段 | 0.103 / 0.114 m/s | — |
| overshoot **预测** | 0.446 / 0.595 m | 沿向越过保持点的最大外扩（模型） |
| overshoot **实测（控制侧）** | 0.452 / 0.591 m | 同口径（mav） |
| 回摆速度（实测·控制侧） | 0.254 / 0.362 m/s | 局部极小后速度回升峰值 |

要点：低阶标称模型对**衰减-峰值段**与 **overshoot** 预测良好（预测 vs 实测均值 0.446 vs 0.452 m）；
但**不含回摆**，故全段位置误差（0.34 m 级）主要来自峰值后的回摆——回摆速度本批实测最高 0.362 m/s，
**本批未观测到**回摆导致停稳失败。

## 8. `Legacy` / 正确位置接口标称 / `B2` 所需范围对比

| 方法 | 延迟假设 | 时间常数 | 位置增益 | 制动位置误差(mean/max) | 制动速度误差(mean/max) | 预测 overshoot |
|---|---:|---:|---:|---:|---:|---:|
| `Legacy`（恒速动作映射） | 0（瞬时生效） | 0 | 1.0 | 0.130 / **0.452 m** | 0.135 / **0.796 m/s** | 0（假定） |
| 正确位置接口·低阶标称 | `L∈[0.18,0.30]` | 0.30 s | 0.983 | —/—（衰减-峰值段 0.046/0.085 m） | 0.103 / 0.114 m/s | 0.446 m |
| `B2` 所需范围 | **区间 `[0.18,0.30]`** | 0.30 s（+方向差） | 0.983（+方向差） | 覆盖上述误差界 | — | 覆盖 overshoot/回摆界 |

- `Legacy` 缺失延迟/滞后建模：其制动位置误差上界 0.452 m ≈ 实测外扩整段，速度误差上界 0.796 m/s ≈ 实测初速整段，
  即**把全部制动动态算作误差**；标称模型在衰减-峰值段把位置误差降至 0.085 m、速度误差降至 0.114 m/s。
- **`B2` 的区间应取辨识区间**（`[L_min,L_max]=[0.18,0.30] s`，含 0.5/1.0 档与纵/横方向差异），**而非单点**：
  方向差异量级 `ΔT=0.007 s`、`Δgain=0.021`（模型级）。若用单点（如旧 `0.19 s`）将漏掉 1.0 档更大的延迟。

## 9. 局限与诚实声明

1. **真实丢包率未测**：本批**未构造**丢帧样例，`drop_frame` 未触发；真实链路丢包率仍**未在本批观测**。
2. **不可识别/受噪声限制的参数**：纯延迟受采样周期（0.02 s）与速度噪声限制，**给区间不硬拟合单值**；
   `T` 与位置滞后为低阶近似，未辨识更高阶或时变特性。
3. **小样本非统计边界**：每档每方向仅 3 次（fit/calib/valid 各 1），**不构成统计安全边界**，
   结论限于本接口/本仿真配置。
4. **回摆未建模**：低阶模型不含回摆，含回摆全段的位置误差（0.34 m 级）**已如实报告**，未掩盖。
5. 零违例/未超限一律写「**本批未观测到**」，**不写**「已证明」。
6. 旧 `0.19 s` 为**阈值响应时间**（含响应动态，见 `metrics_v2.json.threshold_response_note`），
   **不得**当纯延迟；纯延迟由本次辨识给出区间 `[0.18,0.30] s`。

## 10. 可复算命令与产物

```bash
# 采集（示例；source ROS 环境；单 run 约 50 s）
source /opt/ros/noetic/setup.bash && source devel/setup.bash
python3 experiments/paper_v3/scripts/brake_hold_test_v2.py \
  --direction longitudinal --speed 0.5 --altitude 2.0 \
  --run-id long_0p5_r1 --output-root experiments/paper_v3/identification/brake_hold_v3

# 辨识 + 独立验证（纯标准库，无随机数，重复运行结果一致）
python3 experiments/paper_v3/identification/identify_v3.py \
  --root experiments/paper_v3/identification/brake_hold_v3

# 只读指标重算（v2 口径）
python3 experiments/paper_v3/scripts/brake_hold_metrics_v2.py \
  --output-root experiments/paper_v3/identification/brake_hold_v3
```

产物（均在 `experiments/paper_v3/identification/brake_hold_v3/`）：

- 14 个 run 目录（2 烟雾 + 12 trial），各含 `trace.csv`/`publish_log.csv`/`clock.csv`/`meta.json`/
  `metrics_v2.json`/`start_sim.log`。
- `trial_manifest.json`（用途预分配 + `meta_sha256`）、`identified_params.json`（辨识 + 验证 + 对比）、
  `summary_v2.json`/`summary_v2.csv`（v2 指标汇总）。

## 11. 冻结核验（Task 5 相关）

- **未改动**：`one/` 冻结数据（`git status --porcelain one/` 为空）；`src/`；`scripts/`；
  `experiments/paper_v3/scripts/brake_hold_test_v2.py` 与 `brake_hold_metrics_v2.py`；
  v1/v2 数据（`identification/brake_hold_v2/` 无 diff）与既有协议文档。新数据**只写新目录**。
- 仿真**全程 headless**，运行结束**无** `gzserver/px4/mavros` 残留进程（逐进程名核对计数均为 0）；
  采集器只回收本会话进程（`meta.cleanup_leaked_pids` 全为 `[]`）。
- 辨识脚本无随机数，重复运行输出（除 `generated_at`）**完全一致**（已核验）。