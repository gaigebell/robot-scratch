# 工具

运行时优先使用 `robot-scratch` CLI；此目录保留开发检查、被动发现、离线诊断
和旧部署兼容入口。各工具职责见[目录导航](../docs/repository-layout.md)。

开发入口：`python tools/check_project.py`。
公开文件与暂存内容检查：`python tools/check_publication.py --staged`。

需要私人报告/分析的维护工具不属于公开清单；它们的输出不能作为源码提交。
