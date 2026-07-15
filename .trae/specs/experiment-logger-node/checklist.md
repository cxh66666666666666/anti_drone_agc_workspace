# Checklist

- [x] `experiment_logger_node.py` 存在且语法正确（Python 可导入）
- [x] 节点订阅了全部 5 个话题：`/mavros/local_position/pose`、`/rl_planner/local_offset`、`/rl_planner/action`、`/lqr_tracker/position_error`、`/lqr_tracker/adjusted_ref`
- [x] CSV 文件在节点启动时自动创建，文件名包含时间戳
- [x] CSV 表头包含所有 spec 定义字段（18 列：timestamp + 17 数据列，含 2 个衍生字段）
- [x] 定时器按 `record_rate` 频率写入记录，每行一个时间戳
- [x] 未收到消息的话题字段填入 NaN，不阻塞写入
- [x] 衍生字段 `is_avoiding` 和 `offset_magnitude_xy` 计算正确
- [x] `experiment_logger.launch` 可正常启动节点
- [x] `CMakeLists.txt` 包含 `experiment_logger_node.py` 的安装项
- [x] 节点不发布任何话题，不修改控制链路
