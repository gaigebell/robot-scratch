# 文档导航

公共文档描述可复用的设计、接口与操作方式，不发布特定实验室的身份、地址、
运行收据、任务数据或内部分析。历史实验记录和研究资料由维护者在本地单独保管。

推荐顺序：

1. [项目状态](project-status.md)：哪些已实现、哪些仍待验证。
2. [架构](architecture.md)：节点职责、执行权限、线程与数据流。
3. [开发指南](development-guide.md)：安装、修改、测试与提交。
4. [目录职责](repository-layout.md)：模块和开发工具的位置。
5. [配置指南](configuration-guide.md)：从通用模板生成本机配置。
6. [数据格式](data-format.md)：动作、反馈、图像、时间戳与 QC。
7. [部署与发布](release-and-deployment.md)：环境分离、更新回退和公共文件边界。

Agent 从 [AGENTS.md](../AGENTS.md) 与 [agent 入口](agent-onboarding.md) 开始。
设计与实现不一致时，先以代码及可复现测试定位差异，再更新文档，不凭草案
宣称设备、模型或整条物理闭环已经兼容。
