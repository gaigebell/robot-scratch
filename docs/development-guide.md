# 开发指南

## 环境

Python 3.11+，建议使用 uv 和项目独立虚拟环境。编辑器、CI 与本地共用
`.editorconfig`、`pyproject.toml` 和 `.prettierrc.json`。
Node.js 20+ 仅用于 Web 格式化与测试，不是机器人核心运行依赖。

```bash
uv sync --locked --extra dev
npm ci
uv run --no-sync python tools/check_project.py
```

完整媒体与网络离线测试环境：

```bash
uv sync --locked --extra dev --extra video --extra mp4 --extra teleop --extra webrtc
```

`motion` 提供可选 Pinocchio/SciPy；应在匹配平台安装并验证。ROS、厂商 SDK、
URDF 和驱动工作区由设备侧提供，不写入通用核心依赖，也不替换系统 ROS 环境。
内部报告生成工具的依赖在维护者自己的分析环境中管理，不进入公开运行依赖。
更新依赖时运行 `uv lock` 并审查锁文件；日常安装使用 `--locked`。

## 统一检查

```bash
uv run --no-sync python tools/check_project.py
```

依次运行 Python lint/格式、发布文件与文档检查、JSON/图预检、pytest、Web 格式
与 Node 测试。默认不构造设备、不执行物理控制，不因跳过项而宣称对应能力通过。
修改过程中可分开运行：

```bash
uv run --no-sync python -m ruff check src tests tools
uv run --no-sync python -m ruff format --check src tests tools
uv run --no-sync python -m pytest -q
npm run format:check
npm test
```

自动整理格式：

```bash
uv run --no-sync python -m ruff check src tests tools --fix
uv run --no-sync python -m ruff format src tests tools
npm run format
```

自动修复后审查差异，尤其是依赖预检、延迟导入、线程、清理和异常路径。
第三方 `web/vendor/` 不格式化；保留版权与许可证。

## 编码与命名

- UTF-8 无 BOM、LF、文件末尾换行；Python 四空格，Web/JSON 两空格。
- Python 使用 Ruff 统一格式，目标 100 列；不用分号串联操作或单行复杂分支。
- 模块/函数/变量 snake_case，类 PascalCase，常量 UPPER_SNAKE_CASE。
- 注释解释约束、取舍、线程归属和单位，不逐行复述代码。公共入口、复杂状态机、
  队列/锁、持久化边界要有 docstring；内部低风险细节保持简短。
- 新增公共 API 标注参数/返回类型；修改旧 API 时逐步补齐，不为形式一次性重写全部类型。
- 配置值进入系统前验证，报错包含具体字段。禁止静默选取单位、frame 或机器人映射。
- 不捕获所有异常后装作成功。需要兜底清理时保留原始异常与错误事件。
- 文件使用明确编码与上下文管理；时间字段带单位后缀，区分源时钟、接收时间与展示 PTS。

代码、协议键及标识符使用英文；使用说明可用中文。协议/存储中的历史
`robot-infra` 标识、`robot_infra` 导入名和旧 CLI 是兼容边界，不做文本全局替换。

## 分层与并发

新增设备实现放适配器，映射放运动/输入层，状态转换放会话，文件事务放记录层。
用户应用与模型 worker 都不能越过网关直接控制 SDK。
GUI 只作为配置和状态客户端，不拥有第二套权威机器人状态机。

每个后台任务要写明谁创建、谁关闭、谁持有状态；队列需有边界与溢出策略。
关闭时先阻止新输入，再处理排空/取消、关闭文件与编码器、join 并报告未退出项。
暂停/接管后使旧代次失效；不能仅清空队列而允许迟到 IK 或策略结果重新生效。
不要在相机/网络回调中做 IK、长时间磁盘写入或无界编码。

## 测试与变更流程

先确认问题与消费方，再创建最小可验收修改。为行为、接口或故障修复补足有意义的测试；
纯文档/格式变化运行既有检查，不添加只复述实现的测试。
优先可注入时钟、假模型、临时目录和本地回环，避免依赖真实 sleep、私有数据或外网。

验证层级分别记录：静态、unit/integration、构建安装、仿真、只读设备、真实动作、
任务闭环。Windows fake 通过不能替代 Linux/ROS/硬件验证。
真实设备运行必须有操作者明确授权；旧计划或测试记录不构成授权。

## 提交与公开发布

项目名和仓库名统一为 robot-scratch；安装包同名。提交使用描述具体行为的标题，
例如 `refactor: clarify episode writer lifecycle`，正文说明原因、验证与限制。
避免将功能、无关重构和实验结果混在同一次后续变更中。

`publication.toml` 是公开文件清单；`tools/check_publication.py --staged` 检查索引内容。
检查源码/文档、包归档和 Git 历史，而不只看 `.gitignore`。
本机 profile、设备序列号、登录信息、日志、数据集、私有分析及原始结果留在忽略目录。
必要说明重写成通用文档，不将内部文件仅改名后直接公开。

构建及离开源码目录的安装测试：

```bash
uv run --no-sync python -m build --no-isolation
uv run --no-sync python tools/check_distribution.py
```

wheel 安装到独立环境后，运行 validate/fake demo/read-only replay，并确认网页、schema
和第三方许可证资源齐全。sdist 使用明确的 public 文件列表，不能打包内部报告目录。
