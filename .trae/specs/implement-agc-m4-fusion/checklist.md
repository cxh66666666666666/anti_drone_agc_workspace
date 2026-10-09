# M4 验收 Checklist

## 接口与契约
- [x] `agc-m0-v1.2` 契约文档存在且登记 /agc/air/obs、/agc/obstacles、6 消息、source 语义、confidence 标定、故障语义
- [x] `agc_params.yaml` contract_version=agc-m0-v1.2 且含 perception/fusion/calibration 段
- [x] `/agc/air/obs` 唯一发布者 air_perception；`/agc/obstacles` 唯一发布者 air_fusion（激活时）
- [x] `/agc/targets` 发布者 = /agc/target/source，单一发布者（无 gt_adapter 与 air_fusion 双发）

## 字段与坐标
- [x] 目标/障碍观测/融合输出均含 source(或 sensor_id)、frame_id=world、stamp、confidence
- [x] 融合输出 track.header.stamp = 最新真实观测时刻，不被发布时刻刷新

## 行为
- [x] 融合结果可被 M3 消费：goal 跟随融合目标、任务状态正常结算
- [x] 延迟注入：超龄观测判 stale 不计入融合，输出由其余 inlier 支撑
- [x] 丢失注入：全通道丢失后 ≤容忍窗空数组→停车→TARGET_LOST
- [x] 冲突注入：离群通道判 outlier 且记录原因，融合跟随 inlier，不跨源取最大
- [x] confidence 经验覆盖 P(|err|≤tol) 与声明一致（偏差≤预设容差）

## 障碍接入
- [x] source=air_fusion 时 /agc/obstacles 非空且被 map/planner 消费
- [x] task_manager 净空/碰撞仍用真值场景几何
- [x] source=scenario 时 map 内容哈希与 M2 一致（构造性等价回归）

## 工程
- [x] catkin_make 通过；无编译告警阻塞
- [x] 批跑 `any_leftover=false`，退出后关键进程残留 0
- [x] audit_interfaces nominal 模式 PASS；fault 模式不因超龄误判 FAIL
- [x] 注入 run 打 tag 且排除出成功率分母

## 诚实口径
- [x] 文档显式标注"真值派生的感知仿真"，未宣称真实目标探测
- [x] 全部 run 未出现 INTERCEPT_CONFIRMED，未表述为拦截完成
