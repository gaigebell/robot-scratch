# robot-scratch

Modular infrastructure for robot experiments: teleoperation, data collection,
trajectory replay, and policy integration.

robot-scratch 将设备驱动、输入映射、运动求解、执行仲裁、记录和用户流程分开，
让不同实验尽量复用同一条控制与数据链路。当前是研究与验证阶段的软件；
实际部署依赖对应硬件、驱动、模型与标定，不能仅凭统一接口保证跨机器人兼容。

## 快速开始：无需硬件

需要 Python 3.11+、[uv](https://docs.astral.sh/uv/)。
在仓库根目录执行，Windows PowerShell 与 Linux 均适用：

```bash
uv sync --locked --extra dev
uv run --no-sync robot-scratch validate configs/fake-control.json --dry-run
uv run --no-sync robot-scratch run configs/fake-control.json --output artifacts/demo.mcap --steps 100
uv run --no-sync robot-scratch replay artifacts/demo.mcap
```

输出文件须不存在；重试时更换文件名。这些命令仅运行 fake 设备和只读回放。

假模型端到端验证：

```bash
uv run --no-sync robot-scratch workflow-run configs/workflows/mock-inference.example.json --output artifacts/mock-demo
```

这是 fake 设备 + 假模型，不代表真实 VLA/WAM 已完成部署。

## 已有能力

- 固定 fake 控制闭环、所有权与动作校验、MCAP 记录及只读回放。
- 被动设备发现与显式身份绑定；设备、角色、单位和坐标变换由配置声明。
- 双臂 WebXR 会话、可选 WSS/WebRTC 输入、异步 IK、独立夹爪与三相机预览。
- 相机 capture / preview / record 独立配置；原始 MCAP 或 RGB MP4 + 时间索引。
- episode 异步写入、保存/丢弃、对齐与完整性 QC；原生及转换末端数据分开记录。
- 已发布目标驱动的轨迹回放；独立进程 HTTP 假模型与共同执行入口。

采集、回放、推理各有配置与逻辑图模板，当前运行器只支持明确实现的绑定，
并不是任意图解释器。真实模型部署与完整评测仍待验证。

## 阅读入口

| 目的 | 文档 |
| --- | --- |
| 当前实现与未完成范围 | [项目状态](docs/project-status.md) |
| 模块、数据流与并发边界 | [架构](docs/architecture.md) |
| 环境、规范、检查与贡献 | [开发指南](docs/development-guide.md) |
| 文件放哪里、工具做什么 | [目录职责](docs/repository-layout.md) |
| 本地配置、采集与回放 | [配置指南](docs/configuration-guide.md) |
| 数据字段与时间语义 | [数据格式](docs/data-format.md) |
| 部署与公开发布 | [部署指南](docs/release-and-deployment.md) |
| AI agent 接续 | [AGENTS.md](AGENTS.md)、[入口](docs/agent-onboarding.md) |

## 开发检查

Web 修改另需 Node.js 20+：

```bash
npm ci
uv run --no-sync python tools/check_project.py
```

检查包含 Python 格式与 lint、公共文件/文档检查、配置预检、pytest、Web 格式和
Node 测试，不连接硬件。可选媒体、WebRTC 与 IK 依赖见开发指南。

公开名称和安装包名为 `robot-scratch`；为兼容已有部署，Python 导入名仍为
`robot_infra`，旧命令 `robot-infra` 暂时保留为同一入口的别名。
独立训练格式转换工具、数据集、实验室分析和现场日志不随此仓库发布。
