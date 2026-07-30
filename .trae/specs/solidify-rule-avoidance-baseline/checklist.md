# Checklist

- [x] `src/rl_obstacle_avoidance/config/rule_avoidance.yaml` 文件存在且包含全部 11 个参数
- [x] YAML 文件中每个参数均有中文注释说明其含义和默认值
- [x] YAML 文件可在 Python 中正常解析（`import yaml; yaml.safe_load()` 通过）
- [x] `src/rl_obstacle_avoidance/launch/rl_planner_node.launch` 使用 `<rosparam>` 引用 YAML 文件
- [x] Launch 文件中不再包含硬编码的参数值（step_xy、safe_distance_norm 等）
- [x] Launch 文件 XML 语法正确（`xmllint --noout` 通过）
- [x] `README.md` 目录结构中包含 `rl_obstacle_avoidance/` 包
- [x] `README.md` 包含"规则避障系统"章节，说明启动顺序（仿真→起飞→感知→规划→日志）
- [x] `README.md` 包含"实验验证"章节，列出无障碍、单障碍、三障碍三组场景
- [x] `README.md` 中使用 `scripts/start_sim.sh` 的完整命令示例
- [x] `scripts/evaluate_experiment.py` 存在且可独立运行（`python3` 语法无错误）
- [x] evaluate_experiment.py 正确处理 `position_error` 列全为 NaN 的情况（优雅降级）
- [x] evaluate_experiment.py 计算路径长度使用 3D 欧氏距离正确积分
- [x] evaluate_experiment.py 绕障耗时统计基于 `is_avoiding` 列连续变化
- [x] evaluate_experiment.py 支持 `python3 evaluate_experiment.py file1.csv file2.csv` 多文件批量分析
- [x] evaluate_experiment.py 输出包含六项指标的汇总表格
