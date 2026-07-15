# Tasks

- [x] Task 1: 创建 PPT 生成脚本骨架与通用样式函数
  - [x] SubTask 1.1: 在 `scripts/generate_pptx.py` 中初始化 16:9 演示文稿，设置母版字体与黑白配色
  - [x] SubTask 1.2: 实现通用函数：添加页面、标题、正文、分隔线、页脚（左下小字 + 右下页码）
  - [x] SubTask 1.3: 定义每页数据结构（标题、副标题、要点列表），便于后续填充

- [x] Task 2: 实现前 7 页内容（封面、背景、意义、研究现状、目标、架构、RL 建模）
  - [x] SubTask 2.1: 封面页 — 标题/副标题/小组信息/教师/日期
  - [x] SubTask 2.2: 背景页 — 应用场景、传统方法局限、引入 AI 动机
  - [x] SubTask 2.3: 研究意义页 — 应用/技术/实践三层面
  - [x] SubTask 2.4: 研究现状页 — 传统/机器学习/深度学习/新技术四类
  - [x] SubTask 2.5: 项目目标页 — 4 模块输入输出
  - [x] SubTask 2.6: 系统架构页 — 4 层架构图（纯黑线 + 文字框）
  - [x] SubTask 2.7: RL 建模页 — 状态/动作/奖励 三块并排

- [x] Task 3: 实现中间页内容（场景、训练流程、评估指标）
  - [x] SubTask 3.1: 场景设计页 — S0/S1/S2/S3 四类 + YAML 片段
  - [x] SubTask 3.2: 训练与验证流程页 — Stage 0-4 + DQN 伪代码要点
  - [x] SubTask 3.3: 评估指标页 — 5 个指标 + 汇总表结构

- [x] Task 4: 实现末 4 页内容（分工、进度、实际进展、下阶段计划）
  - [x] SubTask 4.1: 项目分工页 — 4 人职责表
  - [x] SubTask 4.2: 时间进度页 — 10 天任务表
  - [x] SubTask 4.3: 项目实际进展页 — 已完成 / 未完成 双栏（关键诚实页）
  - [x] SubTask 4.4: 预期成果与下阶段计划页 — 计划项 + 下一阶段任务

- [x] Task 5: 生成 PPTX 文件并验证
  - [x] SubTask 5.1: 执行 `python3 scripts/generate_pptx.py` 生成文件
  - [x] SubTask 5.2: 用 `python-pptx` 读回验证：页数=14、页幅 16:9、文件可被 LibreOffice 打开
  - [x] SubTask 5.3: 检查输出文件存在于 `docs/指导书/项目答辩PPT.pptx`

# Task Dependencies
- Task 2 依赖 Task 1（需通用样式函数就绪）
- Task 3 依赖 Task 1
- Task 4 依赖 Task 1
- Task 5 依赖 Task 2、3、4（所有内容页完成）
