# Checklist（change-id: deepen-n5-single-agent-boundaries）

## A 低矮屏障上越 vs 水平绕行
- [x] 两组场景固定可哈希，除屏障高度外逐字段一致（`low_barrier` 顶面 < 飞行高度；`tall_barrier` 贯穿）
- [x] manifest 记录 split、场景数与 sha256 并 `.frozen`
- [x] 复用既有 DQN 评估两组，逐回合 3D 轨迹/净空/竖直动作率/结局/失败分母落盘
- [x] 给出上越（越过屏障顶且 x 越过屏障位）与绕行的可核验判据并对照

## B 实时 LQR 竖直参考
- [x] 运行期向 `/rl_planner/local_offset` 注入含 z 的偏移（固定频率，含清零窗）
- [x] 记录参考 z、实际 z、`position_error`；核验参考随注入变化、无人机跟随、清零后恢复
- [x] 如实记录 >0.5 s 无消息自动清零行为

## C 地面障碍布局扩展与隔离
- [x] 地面障碍布局（`poles`）可按场景注入
- [x] train/val/test 按**布局**隔离，test 使用**未见布局**，场景集哈希记录
- [x] 匹配布局的 Gazebo world 由生成器产出（GUI 一致）或如实标注虚拟几何口径
- [x] 短训+评估落 `episodes.jsonl`/`metrics.json`/轨迹，含失败分母与碰撞率

## D LQR 全程与稳态窗口
- [x] 每条轨迹同时给出全段与稳态窗口的 mean/RMS/max/P95 与样本数
- [x] 稳态窗口边界依据（时间/相位）在报告中明确

## 核验与文档
- [x] `scripts/verify_n5.py` A–D 四段 PASS
- [x] 四部分均 `--gui` 非无头运行，模型/场景/参数哈希锁定
- [x] `docs/项目任务日志/阶段2/N5.md` 含口径、实现、批次、逐项结果、交付物核对、未决
- [x] 未出现 `INTERCEPT_CONFIRMED`，未表述为拦截完成
