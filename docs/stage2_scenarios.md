# 阶段 2 任务 1：场景生成与清单

`scenario_generator.py` 是无 ROS 的确定性场景工具。它生成包含障碍物几何、轨迹、起终点、飞行高度、速度上限、噪声/延迟和三类随机种子的场景 JSON；可行性检查使用运行时相同的栅格 A*，并记录路径长度、起终点净空和失败原因。

生成默认的 50/12/30 个 train/validation/test 场景：

```bash
python3 src/rl_obstacle_avoidance/scripts/scenario_generator.py \
  --output-dir experiments/stage2 --freeze
```

输出结构为 `scenarios/{train,validation,test}/<scenario_id>.json`、
`manifests/{train,validation,test}.json` 和 `manifests/index.json`。清单哈希写在
`manifest_sha256`，可用 `verify_manifest()` 离线复核。写入 `manifests/.frozen` 后，工具拒绝覆盖；如需重新生成，应使用新的输出目录（不要修改已冻结清单）。

可通过 JSON/YAML 配置覆盖边界、障碍尺寸、规划参数、轨迹类型和每个划分的数量；命令行的 `--train-count`、`--validation-count`、`--test-count` 可用于小规模 smoke 测试。`scenario_seed` 只控制布局，`disturbance_seed`/`policy_seed` 仅被记录，避免把三类随机性混为一个 seed。

已有场景可单独复核：

```bash
python3 src/rl_obstacle_avoidance/scripts/scenario_generator.py \
  --check experiments/stage2/scenarios/test/<scenario_id>.json
python3 src/rl_obstacle_avoidance/scripts/scenario_generator.py \
  --verify experiments/stage2/manifests/test.json
```
