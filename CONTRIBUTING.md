# Contributing to robot-scratch

请先阅读[开发指南](docs/development-guide.md)、[架构](docs/architecture.md)和
[当前状态](docs/project-status.md)。保持驱动、映射、执行与记录的边界；公共配置
使用占位符，测试使用 fake 或合成数据。

提交前执行 `uv run --no-sync python tools/check_project.py`。
PR 说明具体问题、最终行为、验证环境与未验证范围；涉及协议时说明兼容策略。
不要将本地分析、设备身份、数据、凭据或参考仓库加入提交。
真实硬件或真实策略结果不能用静态检查、fake 或相邻设备的测试代替。

公共代码规范与依赖版本以仓库配置为准，不依赖个人编辑器配置。
