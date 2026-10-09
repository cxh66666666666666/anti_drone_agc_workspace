# Tasks

> 依据 [`four/07_独立复审与下一步.md`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/four/07_%E7%8B%AC%E7%AB%8B%E5%A4%8D%E5%AE%A1%E4%B8%8E%E4%B8%8B%E4%B8%80%E6%AD%A5.md) §三。新代码/场景/数据一律放 `experiments/paper_v3/postterm_outcome/t6_recheck/`；T5 原始日志、`three/09`、冻结协议 v4.0 只读保留。全程无头运行，进程按会话差集回收，不用全局 `pkill`。

- [x] Task 1（T6.1）：在看结果前冻结 v4.1 协议补充说明，产出 `four/08_v4.1协议补充说明.md`
  - [x] SubTask 1.1：写明 hold 定义（唯一尾段控制者、同一预定 hold、目标取终止拍 MAVROS 估计位姿、禁读真值）、主接触源（origcontact，分列原版接触/真值相交/bumper 对照）
  - [x] SubTask 1.2：写明接触心跳规则、复算口径规则（同一有效集合、`/clock` 连续 2.0 s 停稳、窗口实际仿真时长、分列结局）
  - [x] SubTask 1.3：写死**固定清单**（一个已翻转近障场景、一个未翻转近障场景、一个黑障场景，各自相关终止类型）与**四项技术门槛**
  - [x] SubTask 1.4：预先定义新增窄走廊族参数（机理同 dense_corridor、参数不同，1—2 个族）与 ≤24 回合设计，以及研究范围三选一判据；注明沿用 v4.0 的 ≥1/3 主判据不得事后放宽

- [x] Task 2（T6.2）：实现唯一尾段控制者（探针 v3）并做三原因烟雾核验
  - [x] SubTask 2.1：新增 `episode_tail_probe_v3.py`（由 `episode_tail_probe_v2.py` 派生；新增 `--model-sdf` 选择机体；保留逐拍 trace 与单回合 run 纪律）
  - [x] SubTask 2.2：`done` 后进入统一 hold：单一控制者发布同一 hold 命令（冻结 LQR 参考相位 + 保持 offset，使命令位姿≈终止拍 MAVROS 估计位姿），禁用 Gazebo 真值参与控制；记录命令时间/值/发布者
  - [x] SubTask 2.3：订阅并落盘实际 `/mavros/setpoint_position/local` 与 LQR/其他发布者状态，核实 hold 保持（漂移在预定容差内）且无发布者竞争
  - [x] SubTask 2.4：烟雾回合：`shield_infeasible`、`sensor_invalid`/`perception_timeout`、`collision` 各 ≥1 个，逐回合给出 hold 保持与竞争核查结论
  - [x] SubTask 2.5：保存代码哈希、SDF 哈希、命令参数、场景/种子、时钟、终止/复位原因

- [x] Task 3（T6.3）：主接触源改为原版碰撞体并落盘接触心跳
  - [x] SubTask 3.1：以 `iris_3d_lidar_origcontact.sdf` 运行；确认 `contact.csv` 的 `collision1` 为原版 `base_link_inertia_collision`/`rotor_*_collision`
  - [x] SubTask 3.2：接触心跳：**空 `states` 消息**也落盘（消息时间/序号），可逐尾段窗口判断信号在场；局部失联按窗口标记为基础设施失败
  - [x] SubTask 3.3：分列输出原版接触、bumper 接触（对照）与真值几何相交；给出一例窗口级信号在场/失联样例
  - [x] SubTask 3.4：deliverable：心跳日志、接触源核对记录与复算入口

- [x] Task 4（T6.4）：修复复算口径，产出分析 v2（保留旧版本）
  - [x] SubTask 4.1：新增 `t6_analyze.py`：主值与 `{2,4,8} s` 敏感性使用**同一有效回合集合**（排除复位失败与接触信号缺失/局部失联）
  - [x] SubTask 4.2：按 `/clock` 判定连续 `2.0 s`、`speed<0.05 m/s` 的停稳；记录每窗口实际仿真时长；分列原版接触/真值相交/bumper 对照
  - [x] SubTask 4.3：对既有 T5 数据给出**修订注记**（旧 0.227 的偏差、旧「12/27」停稳口径更正），**不覆盖** T5 原始日志、协议 v4.0 与原分析
  - [x] SubTask 4.4：对新批次（如已运行）与 T5 数据用同一 v2 口径复算，输出分母/缺失/不一致矩阵

- [x] Task 5（T6.5）：固定清单小复核与门槛判定，产出 `four/09_修复后小复核.md`
  - [x] SubTask 5.1：按 v4.1 固定清单运行小复核（一个已翻转近障场景、一个未翻转近障场景、一个黑障场景，各自覆盖相关终止类型）
  - [x] SubTask 5.2：逐回合用 v2 口径复算：立即结算 vs 继续观察、原版接触、真值相交、停稳/右删失、窗口仿真时长
  - [x] SubTask 5.3：判定四项技术门槛（共同 hold、原版 contact、heartbeat、停稳复算）是否**同时**通过，并如实报告未通过项
  - [x] SubTask 5.4：保留全部原始日志、失败与不确定性；结论措辞与证据等级匹配

- [x] Task 6（T6.6）：条件性窄走廊批次与研究范围决定，产出 `four/10_研究范围决定.md`
  - [x] SubTask 6.1：**仅当**四项技术门槛同时通过：按 v4.1 预定义场景总体生成 1—2 个窄走廊族（≤24 回合）并运行；否则跳过并记录原因
  - [x] SubTask 6.2：若已运行：按家族**分层**报翻转率（不合并），同时报右删失比例、基础设施失败率、bumper 与真值几何分歧例数
  - [x] SubTask 6.3：给研究范围决定（稳定可解释 → 设计新批次；仅在挑选近障族 → 改题 + 独立家族验证；主要来自 bumper/尾段控制 → 停止论文主张、保留平台测量修复）
  - [x] SubTask 6.4：更新论文题目与投稿价值的重新裁决，保留负结果/不确定性；登记实际投入

# Task Dependencies

- Task 2 依赖 Task 1（hold 与门槛定义先冻结）
- Task 3 依赖 Task 2（探针 v3 的 `--model-sdf` 与心跳改造）
- Task 4 独立于 Task 2/Task 3 的执行（可并行），但其对新批次的复算需在对应 run 完成后
- Task 5 依赖 Task 2、Task 3、Task 4
- Task 6 依赖 Task 5（四项门槛判定）；未通过则只给测量修复结论，不追加批次
- 各 Task 产物互不覆盖；旧数据只读保留；新增一律写入 `experiments/paper_v3/postterm_outcome/t6_recheck/`