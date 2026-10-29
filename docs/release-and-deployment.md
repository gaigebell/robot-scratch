# 部署与公开发布

## 名称与环境

项目/仓库/安装包名为 `robot-scratch`。Python 导入名 `robot_infra`、旧 CLI
`robot-infra`、设备包装器旧名和版本化协议标识暂时保留兼容。
已有设备目录可以继续使用原路径；项目更名不自动搬迁驱动或修改数据文件。

开发主机安装独立环境；设备端保留 ROS 与厂商 SDK 的已知可用环境。
根据设备的操作系统、Python、驱动和模型版本选择依赖，不替换系统 Python。
Windows 可运行核心与 fake 测试，不据此宣称真实驱动兼容。

## 源码 checkout

```bash
git clone https://github.com/gaigebell/robot-scratch.git
cd robot-scratch
git switch m2-validation
uv sync --locked --extra dev
uv run --no-sync python -m pytest -q
uv run --no-sync robot-scratch validate configs/fake-control.json --dry-run
```

部署固定到审阅后的 commit。匹配平台的可选媒体、网络与 motion 依赖单独安装。
本地 deployment/profile 不纳入版本控制，替换版本前保留其备份与对应模型/标定身份。
纯 Git 推送不更新正在运行的机器人服务。

## 更新与回退

先结束本项目 session，确认记录器完成，再更新 checkout 与独立环境。
依次运行静态检查、fake、适用的只读检查和明确授权的设备验证。
回退使用已知 commit 及匹配配置/环境；不对厂商工作区或数据目录执行 reset/clean。
独立服务应能从设备终端启动，不依赖开发机 SSH 长期在线。

## 公开文件与历史

`publication.toml` 明确允许源码、测试、通用模板和公共文档。
内部研究、原始结果、数据、权重、证书、真实身份、运行目录和参考仓库不发布。
`MANIFEST.in` 与源文件检查使用同一发布清单，防止通过 sdist 间接带出内部文件。

已有提交含私人实验记录时，仅在新提交删除文件不能清除历史。
首次公开基线使用无私有父提交的独立分支；本地原分支和文件保留用于维护，
只推送审核后的公开分支，不使用 `--all` 或镜像推送。
之后继续在此公开基线上做小步提交。

发布前运行：

```bash
uv run --no-sync python tools/check_project.py
uv run --no-sync python tools/check_publication.py --staged
uv run --no-sync python -m build --no-isolation
uv run --no-sync python tools/check_distribution.py
```

检查索引与归档内容；输出目录使用 `artifacts/` 或 `dist/`。
仓库公开不自动授予本项目统一开源许可；在维护者明确选择许可证前，
不擅自添加许可条款。随附第三方网页资源的原许可证保留在 vendor 目录。
