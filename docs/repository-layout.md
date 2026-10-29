# 目录职责

```text
robot-scratch/
├── src/robot_infra/       # 可安装运行时代码，导入名保留兼容
│   ├── adapters/         # ROS、SDK、原生 CAN 和姿态观测桥接
│   ├── schemas/          # 版本化图/事件 JSON schema
│   └── web/              # WebXR 页面、输入连接、VR 显示与 vendor 许可
├── configs/              # 通用模板、fake 配置和明确拒绝的非法配置 fixture
│   ├── graphs/           # 数采/回放/推理逻辑图
│   └── workflows/        # 已实现的 native 工作流绑定
├── tests/                # 不依赖私人数据的 unit/integration/config/Web 测试
├── tools/                # 开发检查、被动发现、诊断和设备侧兼容入口
├── docs/                 # 公共设计与开发/配置/部署文档
├── plans/templates/      # 任务计划与验证报告模板
├── publication.toml      # 公开文件清单
├── pyproject.toml        # Python 包、可选依赖、Ruff/pytest 配置
├── uv.lock               # Python 依赖锁
└── package.json          # 仅开发用的 Web 格式化/测试命令
```

本地运行生成内容分别放 `artifacts/`、`runs/`、`dataset/` 或 `datasets/`，
真实身份与路径放 `profiles/local/`；这些目录不公开。
`relevant-repo/` 是可选本地参考资料，不是本项目依赖或源码的一部分。
研究、报告生成器、历史计划和现场证据允许在维护者本地保留，但不加入发布清单。

## 运行时导航

- **预检与契约**：contracts、graph、collection、experiment_graph。
- **fake 基线**：fake、runtime、app、recording、collection_runtime、collection_demo。
- **双臂会话**：teleop_session 管状态与控制 tick；live_control 管网关；
  live_teleop 处理会话应用及预览入口；session_launcher 统一预检与生命周期。
- **运动**：kinematics 是纯计算/FK 与旧有实现；pinocchio_model 使用验证广泛的可选包；
  solver_worker 隔离长计算，结果回到会话检查。
- **网络与观测**：webxr_input、control_socket、control_webrtc、camera_ingress、
  observations、preview、pose_feedback。
- **采集**：real_episode 是队列与磁盘 worker；episode_store 管目录事务；
  image_storage 管图像保存后端；episode_qc 读文件检查质量与完整性。
- **数据与执行来源替换**：episode_reader、training_view、trajectory、workflow、
  workflow_runner。回放/策略复用网关，不等于二者有相同输入语义或启动流程。

## 工具导航

| 工具 | 用途 |
| --- | --- |
| check_project.py | 开发者的一条无硬件检查命令 |
| check_publication.py | 公开文件/隐私模式、相对文档链接与暂存检查 |
| check_distribution.py | 构建包中的公共文件与运行资源检查 |
| collect_host_inventory.py | 读取主机元信息；其输出包含本机环境，不提交 |
| discover_devices.py | 被动设备发现的源码兼容入口 |
| audit_camera_delivery.py | 离线对照图像索引与元数据，不改源记录 |
| profile_input_transport.py | 有界 WSS 回环/网络诊断，不执行机器人命令 |
| launch_live_stack.py | 兼容旧 checkout 的启动入口，优先使用正式 CLI |
| robot-infra-device.sh | 设备侧 ROS 环境和显式 deployment 包装器，旧名保留兼容 |

新工具先明确是运行时能力、通用诊断、开发维护还是私人分析。
运行时能力进入包与 CLI，避免把用户流程仅藏在 tools 中；私人分析留在忽略目录。
