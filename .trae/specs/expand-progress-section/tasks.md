# Tasks

- [x] Task 1: 在 `scripts/generate_pptx.py` 中新增 4 个 slide 函数
  - [x] SubTask 1.1: `slide_10_current_system(prs)` — 当前系统介绍
  - [x] SubTask 1.2: `slide_11_csv_data(prs, metrics)` — 本次实验数据 + 动作分布柱状
  - [x] SubTask 1.3: `slide_12_in_progress(prs)` — 3 个在研工作大卡片
  - [x] SubTask 1.4: `slide_13_demo_placeholders(prs)` — 2×2 占位框
  - [x] SubTask 1.5: `slide_14_summary(prs)` — 收尾页（重编号）

- [x] Task 2: 删除原 `slide_10_progress` 与原 `slide_11_summary`，更新 `build()` 与 `TOTAL_PAGES`
  - [x] SubTask 2.1: 移除 `slide_10_progress` 函数定义
  - [x] SubTask 2.2: 移除原 `slide_11_summary`，被新 `slide_14_summary` 取代
  - [x] SubTask 2.3: `TOTAL_PAGES = 14`
  - [x] SubTask 2.4: `build()` 按 14 页顺序调用

- [x] Task 3: 生成 PPTX 并验证
  - [x] SubTask 3.1: 执行 `python3 scripts/generate_pptx.py`
  - [x] SubTask 3.2: 验证：页数=14、新 4 个子页内容齐全
  - [x] SubTask 3.3: LibreOffice 转 PDF 验证

# Task Dependencies
- Task 2 依赖 Task 1（先准备好新函数再删旧的）
- Task 3 依赖 Task 1、Task 2
