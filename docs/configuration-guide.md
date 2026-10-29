# 配置与运行指南

## 选择正确的配置

| 文件 | 含义 |
| --- | --- |
| configs/fake-control.json | M1 固定 fake 控制图，供 validate/run/read-only replay |
| configs/fake-a4s-collection.json | 独立多端点 fake 采集计划 |
| configs/graphs/*.json | 采集、轨迹回放、推理的逻辑图声明 |
| configs/workflows/mock-inference.example.json | fake + 假模型，可独立无硬件运行 |
| configs/workflows/a4s-replay.example.json | native 轨迹输入模板，需本地数据与部署 |
| configs/teleop-session.example.json | 双臂 session、按键、IK、角色与记录配置模板 |
| configs/realsense-streams.example.json | 按相机配置 capture / preview / record |
| configs/deployment.example.json | 设备侧依赖、会话路径、TLS 与驱动环境入口 |

不同 schema 不能混用。设备模板故意包含占位符，并不保证复制后就能驱动任意硬件。
静态图预检不启动设备：

```bash
uv run --no-sync robot-scratch experiment-plan configs/graphs/a4s-collection.json
uv run --no-sync robot-scratch collect-validate configs/fake-a4s-collection.json --dry-run
```

## 准备本机 profile

将所需模板复制到 `profiles/local/`。填写真实设备身份、驱动路径、ROS 环境、
URDF/哈希、关节顺序、相机角色、坐标映射和 TLS 信息。不要改公共示例以保存
真实 IP、序列号或账号，也不要复用其他机器的未知标定。

deployment 路径以该 JSON 所在目录解析；检查其中会话、相机、证书、驱动工作区
和依赖路径确实属于本机。设备侧使用的 Python 必须能访问对应 ROS 与 SDK，
不能用开发环境覆盖系统 ROS。详情见[部署指南](release-and-deployment.md)。

相机 capture 是设备输出，record 是需要保存的数据，preview 是显示消费者。
例如采集 30fps 的原始 RGB 和头部深度，同时用较小 RGB/较低 FPS 显示。
降低预览不应降低原始采集频率；选择 MP4 只影响 RGB 保存，深度仍保留原始值。

## 遥操与 episode 采集

以下命令会连接已配置设备，仅在已完成本机预检、明确批准真实运行时执行。
`--deployment` 是必填项：

```bash
robot-scratch collect \
  --deployment profiles/local/deployment.json \
  --task example_task \
  --dataset-root dataset \
  --start-episode 0 \
  --existing skip \
  --image-storage mp4
```

任务、根目录、起始编号和重复编号策略均显式指定。
`skip` 避开已有编号，`error` 拒绝冲突；`overwrite` 只能用于明确允许覆盖的任务。
只做自由遥操使用 `device-teleop --deployment ...`，不会创建采集任务。

典型按钮流程由配置绑定：

1. 启动/自检后打开网页，进入 VR；待机时主按钮回到显式 home。
2. 到达准备状态，再按主按钮开始遥操/episode。
3. 侧握控制位姿使能；扳机开闭方向、是否独立控制夹爪由 profile 决定。
4. 主按钮结束，等待 writer 排空和 QC；待审阅时主按钮保存，丢弃按钮丢弃。
5. 退出 VR 或输入过期会暂停；不会把暂停自动标成成功示范。

原始记录保存在 `<dataset-root>/<task>/episodes/episode_XXXXXX/`。
QC 是数据/时序检查，不等同任务成功、物理标定正确或操作安全。
离线完整性检查不连接设备：

```bash
robot-scratch episode-qc dataset/example_task/episodes/episode_000000 --deep
```

## 轨迹与模型来源

`trajectory-check` 只构建/校验本地已发布目标，`replay` 命令只读事件时间线；
二者都不是让机器人重放运动。真实轨迹执行使用 `workflow-run` 的 native 绑定，
物理 backend 另需显式 `--execute`，并先单独核对起始对齐、单位、夹爪与现场空间。

假模型示例仅验证来源替换、调度与执行接口。真实模型需独立服务器适配、观测/动作
契约与执行评测，不能只把网络地址替换进 mock 配置后宣称部署完成。
