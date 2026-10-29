# Agent 入口

先读 [AGENTS.md](../AGENTS.md)、[架构](architecture.md)和[实现状态](project-status.md)，
再按任务阅读[目录职责](repository-layout.md)与[开发指南](development-guide.md)。

1. 检查 cwd、分支、Git 差异，保护已有修改。
2. 找到负责该行为的模块、配置及消费方测试；实质任务先写计划。
3. 优先 fake/离线的最小纵向修改，明确验收条件与未验证项。
4. 运行 `python tools/check_project.py` 及改动需要的可选环境检查。
5. 更新状态和文档，报告环境、实际结果、风险与下一步。

[计划模板](../plans/templates/task-template.md)与
[验证模板](../plans/templates/validation-report-template.md)可复用。
维护者本地可能另有被忽略的研究、历史计划和现场收据；它们不是公开能力保证，
不得在提交时自动包含。旧文件若不存在，从项目状态与新计划继续，不伪造历史证据。

公开发布遵守 `publication.toml`，先检查文件清单和 Git 历史。
不要运行无差别的 `git add .`、`git push --all`，也不要将真实设备配置复制到示例。
不因读取了计划、手册或已有测试记录而获得机器人运动授权。
