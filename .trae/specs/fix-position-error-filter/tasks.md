# Tasks

- [x] Task 1: 在 `compute_metrics_from_csv` 中增加过滤逻辑
  - [x] SubTask 1.1: 增加常量 `TARGET_REACH_THRESHOLD = 1.0`
  - [x] SubTask 1.2: 实现「首次达到目标」索引计算
  - [x] SubTask 1.3: 在返回字典中应用过滤 A + B + C
  - [x] SubTask 1.4: 新增字段：`pe_filtered_n` / `pe_filtered_dropped_takeoff` / `pe_filtered_dropped_avoiding` / `pe_filtered_dropped_nan` / `first_reach_time`

- [x] Task 2: 更新 PPT 第 11 页文案
  - [x] SubTask 2.1: `slide_11_csv_data` 中位置误差卡片说明改为「稳态非避障期 / 样本 N」
  - [x] SubTask 2.2: 位置误差详情一行改为新数据（实时计算）+ 过滤说明

- [x] Task 3: 重新生成 PPTX 并验证
  - [x] SubTask 3.1: 执行 `python3 scripts/generate_pptx.py`
  - [x] SubTask 3.2: 验证：过滤后样本数 = 447，均值 ≈ 0.679 m
  - [x] SubTask 3.3: LibreOffice 转 PDF 验证

# Task Dependencies
- Task 2 依赖 Task 1（需先有过滤后的 metrics）
- Task 3 依赖 Task 1、Task 2
