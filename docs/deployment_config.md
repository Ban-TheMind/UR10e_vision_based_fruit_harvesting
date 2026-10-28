# 联合部署配置

联合部署及相关诊断共用 `src/harvesting_config/harvesting_config/deployment.py`。
该 module 只使用 Python 标准库，负责配置读取、默认值、允许的临时覆盖和模式所需条件；它不启动设备，也不判断实时 TF 或地图是否就绪。

## 职责与使用

- `harvesting_config`：共享配置规则和模式决策，不依赖上层 ROS 包。
- `harvesting_bringup`：保存现场 `site.json`，将已经校验的部署结果组织为启动动作。
- `end_effector_description` 的 Foxy MoveIt 启动：使用共享点云规则生成 MoveIt 与 guard 参数；`sensors_3d.yaml` 只声明插件，不重复保存参数默认值。
- `scripts/check_profile.py`：构建前通过普通 Python 读取源码配置包，检查后才加载 ROS 环境。
- `above-check`、`mapping-check`：在现有 ROS overlay 中正常导入安装后的配置包，不再按路径执行 launch 目录中的配置代码。

共享入口为 `resolve_deployment(source, mode, overrides=None)`，返回独立的规范化配置、模式开关、RViz 开关及当前模式使用的点云、桌面几何、执行预算。
`source` 可以是 JSON 路径或已经读取的字典；解析不修改调用方输入。
只读取 JSON 的 `load_profile` 不执行模式校验，运行入口必须使用完整解析入口。

## 按模式检查

| 模式 | 检查的条件 |
| --- | --- |
| fake | UR10e 模型设置与假硬件权限；不要求真实机器人地址、相机、夹爪或任务配置 |
| robot | 真实机器人地址、类型和 headless 配置 |
| camera | 相机序列号、图像规格、外参路径字段及检测解释器；不要求机器人或夹爪配置 |
| gripper | 串口、波特率及 slave ID |
| all | robot、camera、gripper 三者所需配置；不要求已填写桌面抓取参数 |
| mapping | 机器人、相机采集及点云参数；不要求检测解释器、夹爪或抓取几何参数 |
| demo | robot、camera、gripper、任务和所选抓取策略需要的几何参数 |

未使用的配置节可以缺失或保留待填写值；它们不会传给本阶段不启用的节点。
JSON 语法错误、未知模式、非法临时覆盖，以及当前模式使用的非法配置均明确失败。
mapping 的 `calibration_verified` 省略时为 false；不会自动确认标定。

`above-check` 连接已经运行的系统，仅校验自身使用的工具几何与执行预算；显式执行时还检查机器人地址和扫描关节目标。
其高度、方向等现有命令行参数继续按原规则验证；不要求夹爪配置，也不要求 profile 已填完 demo 的示教高度。
`camera-check` 仍通过 camera 模式启动，`mapping-check` 使用与 mapping 启动相同的配置解析。

## 兼容与优先级

现场 JSON 路径和字段名、现有模式和命令保持兼容。
配置来源仍由 `HARVEST_PROFILE` 或已有 `--profile` / ROS `profile:=...` 参数选择；不新增第二套现场配置。
普通联合启动仅接受既有临时覆盖：`motion_enabled`、`rviz`、`execution_timeout`。
优先级为显式临时覆盖、profile 中的值、共享默认值；运动权限仍受模式规则约束，fake/mapping 均拒绝开启执行。
重复覆盖同一参数明确报错，避免预检查与 ROS launch 对重复参数的解释不一致。

底层 Foxy MoveIt 启动保留原有参数；点云字符串参数使用与 JSON profile 相同的校验规则，包括更新频率上限 30 Hz。
直接启动节点时的防御检查和 C++ 地图 guard 继续保留。
地图、TF、点云缺失或过期时的运行时拒绝行为仍由 guard 执行，配置检查通过不代表地图就绪。

`robot-check` / `home` 继续使用已审查的固定设备信息与 Home 目标，不读取联合 profile；其监督和取消策略未改动。

## 构建与验证

本次新增独立 Python ROS 包，首次更新后必须完整运行 `bash scripts/project build`，再停止并重新启动旧节点；只构建原有单个包不足以安装新依赖。
配置包由 bringup 和机器人描述包单向依赖，避免把共享规则放入 bringup 后形成反向依赖。
离线预检查读取源码，ROS 启动及在线诊断读取 overlay 中安装的版本；每次更新后仍需按既有流程重新构建。

`bash scripts/project check` 纳入配置回归：局部 profile、模式隔离、覆盖优先级、启用条件拒绝、共享点云校验、真实启动函数接线和独立安装目录导入。
这些检查不连接硬件，不替代 Foxy 完整构建及现场验收。
