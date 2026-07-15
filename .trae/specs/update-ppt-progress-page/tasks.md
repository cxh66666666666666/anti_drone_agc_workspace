# Tasks

- [x] Task 1: 修改生成脚本 `scripts/generate_pptx.py`
  - [x] SubTask 1.1: 删除 `slide_11_division`、`slide_12_schedule`、`slide_14_plan` 三个函数及其在 `build()` 中的调用
  - [x] SubTask 1.2: 重写 `slide_13_progress`：标题改为「09  项目当前进展与在研工作」，4 象限布局
  - [x] SubTask 1.3: 新增 `slide_summary`（收尾页），标题「10  总结与 Q&A」
  - [x] SubTask 1.4: 在 `build()` 中按新顺序调用 11 个 slide 函数
  - [x] SubTask 1.5: 在脚本中增加「读 CSV 实时统计」逻辑，5 个指标不写死

- [x] Task 2: 实现 CSV 统计函数
  - [x] SubTask 2.1: 编写 `compute_metrics_from_csv(path)` 函数
  - [x] SubTask 2.2: 输出：运行时长、记录条数、避障激活比、位置误差均值/中位/最大、动作分布
  - [x] SubTask 2.3: 处理 NaN 与空值

- [x] Task 3: 生成 PPTX 并验证
  - [x] SubTask 3.1: 执行 `python3 scripts/generate_pptx.py`
  - [x] SubTask 3.2: 验证：页数=11、第 10 页包含 4 个区块、CSV 数据被正确插入
  - [x] SubTask 3.3: LibreOffice 转 PDF 验证可打开

# Task Dependencies
- Task 2 依赖 Task 1.1（确定统计函数位置）
- Task 3 依赖 Task 1、Task 2
