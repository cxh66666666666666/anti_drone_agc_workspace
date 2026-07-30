# Tasks

- [x] Task 1: 创建 rule_avoidance.yaml 配置文件
  - [x] 在 `src/rl_obstacle_avoidance/config/` 目录下创建 `rule_avoidance.yaml`
  - [x] 写入当前经过实验验证的全部避障参数（含注释说明每个参数的含义）
  - [x] 包含对 LQR Tracker 端 `rl_velocity_gain` 的备注

- [x] Task 2: 修改 rl_planner_node.launch 引用 YAML
  - [x] 将硬编码参数替换为 `<rosparam file="...">` 引用 `rule_avoidance.yaml`
  - [x] 验证 launch 文件 XML 格式正确性

- [x] Task 3: 更新 README.md 避障系统文档
  - [x] 在目录结构中补充 `rl_obstacle_avoidance` 包
  - [x] 新增"规则避障系统"章节，包含启动顺序说明
  - [x] 新增"实验验证"章节，列出三组已验证场景

- [x] Task 4: 创建 evaluate_experiment.py 评价脚本
  - [x] 实现 CSV 数据读取和 pandas DataFrame 解析
  - [x] 实现六项指标计算：
    - 避障成功率（基于 action 状态变更判断）
    - 最小障碍距离（基于 offset 幅值反推）
    - 最大/平均跟踪误差（position_error 字段统计）
    - 路径长度（pose 序列积分）
    - 绕障耗时（is_avoiding 持续时间）
    - offset 抖动程度（避障期间 offset_magnitude_xy 标准差）
  - [x] 支持单文件和批量 CSV 输入
  - [x] 输出格式化汇总表格（终端输出）

# Task Dependencies
- Task 2 依赖于 Task 1（需要 YAML 文件先存在）
- Task 3 可与 Task 1、2、4 并行
- Task 4 独立于 Task 1、2、3
