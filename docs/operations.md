# 项目操作手册（内部）

本手册面向实验操作者。对外说明保留在根目录 `README.md`；历史 README 中的 Humble、原夹爪和启动方式不代表当前现场部署。当前维护版本是用户 fork 的 `main`，运行目标为 Ubuntu 20.04 / ROS2 Foxy；其他电脑负责编辑、同步和离线检查。

## 统一入口

在项目根目录使用 `./scripts/project`，不带参数会显示帮助，不启动设备。

| 命令 | 内容 | 是否连接设备／执行动作 |
|---|---|---|
| `./scripts/project check` | 配置、Python 3.8 语法、几何、协议、运动策略及流程回归 | 不连接硬件或 ROS 图 |
| `./scripts/project build` | Foxy colcon 编译 | 只写构建产物 |
| `./scripts/project site-check` | 现场依赖及 Xacro 检查 | 不启动设备节点 |
| `./scripts/project fake` | 假机械臂、MoveIt、规划服务 | 不启动真实相机或夹爪；不支持物理执行开关 |
| `./scripts/project robot` | 真实 UR 驱动、MoveIt、规划服务 | 连接机械臂；服务默认只允许规划 |
| `./scripts/project camera` | 固定 RealSense、视觉服务 | 连接相机，不启动机械臂和夹爪 |
| `./scripts/project gripper` | 夹爪服务 | 默认拒绝串口命令，不自动激活 |
| `./scripts/project all` | 三个子系统联合启动 | 无自动采摘，运动和夹爪命令默认关闭 |
| `./scripts/project demo motion_enabled:=true` | 完整采摘流程，默认 1 轮 | 明确开启后会发送机械臂和夹爪动作 |
| `./scripts/project robot-check` | 独立单终端机械臂状态检查 | 自行启动驱动；不发送轨迹 |
| `./scripts/project home` | 独立单终端 Home 检查及动作 | 会发送运动；保留原有运行状态和最终关节核验 |

`robot-check`、`home` 是独立的现场专用入口，保留已审查的固定 IP 与 Home 目标，不读取联合启动配置。运行前停止已有机械臂控制进程；不要与 `robot`、`all` 或 `demo` 并行。其实现仍在 `tools/ur10e_control.py`。

`motion_enabled:=true` 在 `robot` 或 `all` 模式允许之后的服务请求执行动作，并不自动发送动作。夹爪开关也由同一参数传入。关闭它仅限制项目服务，不能阻止其他节点直接调用控制器。真实驱动连接本身可能启动机器人控制程序，因此真实模式应在现场具备操作条件时使用。

旧入口 `ros2 launch end_effector_description display.launch.py` 现在转入统一启动，默认 `fake`；需要真实设备时显式传入 `mode:=robot` 或 `mode:=all`。历史 `auxiliary.launch.py` 和 horizontal/tmp 演示保留用于追溯，不属于当前受测主入口。

## 配置与环境

联合启动配置在 `src/harvesting_bringup/config/site.json`，采用 JSON，离线解析不依赖 PyYAML。记录机器人 IP/headless、相机序列号和推理 Python、串口参数及流程参数。现场配置的扫描姿态采用已实测 Home，投放姿态采用操作者 2026-10-10 示教的关节位置。抓取偏移和方向仍继承历史示例，尚需核对相机外参、TCP 和抓取几何；默认 `calibration_verified=false`。

关节数组统一为 `[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`，单位弧度。笛卡尔位置为米，姿态为弧度，规划参考系为 `base_link`；夹爪宽度为标称毫米，力为 0–255 原始编码。

任务配置可补充流程节点声明的 `drop_position`、`pick_offset`、超时等参数。`calibration_verified` 是标定状态记录：`false` 会在流程启动时提示，不再阻断执行；是否允许动作由显式 `motion_enabled:=true` 决定。保持真实标定状态，只有现场核验后才记录为 `true`。修改配置后重启相应节点。

```bash
# 使用另一份本地配置，不修改已提交的默认配置。
export HARVEST_PROFILE=/绝对路径/site.json
# 按现场已有位置设置；不安装或改动原推理环境。
export UR10E_FOXY_VENV=/home/ubuntu/Desktop/robot_learning/.venvs/ur10e-project-foxy
export ROS_DOMAIN_ID=61
./scripts/project build
./scripts/project site-check
./scripts/project fake rviz:=false
```

`HARVEST_PROFILE` 建议使用绝对路径，启动前验证未知模式、参数和演示开关。默认 venv 统一为 `ur10e-project-foxy`；若现场实际名称是旧的 `ur10e-foxy`，设置 `UR10E_FOXY_VENV` 指向它即可，不必新建。未找到 venv 时使用当前 Foxy Python；此时必须自行确认依赖齐全。Foxy 节点使用 Python 3.8，YOLO 在独立 VPP 子进程运行。相机 `calibration_file` 留空时读取项目内置固定相机矩阵。

构建和启动统一读取 `build-project/`、`install-project/`、`log-project/`。不要混用旧 `install/`。构建入口不加载旧项目 overlay；构建后启动会加载新的 `install-project/local_setup.bash`。更新源代码后必须重新构建并停止、重启旧节点。

离线检查需要 NumPy，C++ 策略检查需要 g++（缺失时会提示跳过，不能计为通过）。支持指定已有的检查解释器：

```bash
HARVEST_CHECK_PYTHON=/已有环境/bin/python ./scripts/project check
```

该变量只用于离线检查，不能据此用新版本 Python 运行 Foxy。检查模型文件不等于已经执行 YOLO 推理；真实权重和推理环境另用 `tools/test_inference_worker.py` 验证。

## 更新顺序

```bash
git status --short
git fetch origin
git merge --ff-only origin/main
./scripts/project check
```

若存在未提交改动或快进失败，先识别改动和分叉；不要用强制重置掩盖问题。现场再构建、停止旧节点、重启并按 `docs/test_plan.md` 分层验收。Git 更新不会更新运行中的进程，也不自动同步 Codex 对话。


## Home 速度与超时（2026-10-10 更新）

`bash scripts/project home` 接受大于 0、最高 100% 的有效速度倍率；
100% 表示按规划轨迹速度执行，不表示绕过机器人安全限值。
规划速度与加速度缩放仍各为 0.1，不自动改动示教器倍率。

```bash
# 默认：执行总超时 600 秒，无进展超时 15 秒
bash scripts/project home
# 显式设置（秒），两者必须为有限正数，停滞超时不能超过执行总超时
bash scripts/project home --execution-timeout 600 --stall-timeout 15
```

执行前在 `/moveit_simple_controller_manager` 显式设置并回读
`trajectory_execution.execution_duration_monitoring=false`，处理 Foxy 的启动覆盖未生效问题。
参数服务缺失、拒绝设置或回读不符时停止，不发送轨迹。
不再以名义时长约 6 秒判断低速轨迹超时；入口仍负责有限执行时间、停滞取消、
程序/安全/关节状态和最终到位检查。外层会话看门狗随执行超时增加 180 秒，
中断后留 20 秒清理；不会让自定义执行超时受旧固定 720 秒限制。
执行中每 2 秒报告耗时、有效速度倍率与距 Home 最大关节误差。
该改动须现场验收，离线通过不等于真机 Home 已完成。


## 单终端视觉诊断

先 Ctrl+C 停止原 `project camera`，再运行：

```bash
bash scripts/project camera-check
# 默认苹果类别 0、置信度阈值 0.3，可显式调整
bash scripts/project camera-check --class-id 0 --confidence 0.3
```

入口自动加载项目环境，只启动相机驱动与视觉服务，等待 RGB/对齐深度图像，
请求一次 `detect_debug` 后关闭本次相机进程。已有相机节点时拒绝重复启动。
不启动机械臂、夹爪或自动采摘。检测响应最长等待 75 秒，会话最长 180 秒。

`log-project/camera-check-*` 保存 camera.log、result.json、实际推理输入 rgb.png
及原始识别框 detections.png。响应区分 YOLO 原始框、类别/阈值通过、无效深度、
缺少投影/内参与最终三维坐标；零目标时请求可成功，但不代表已找到可抓取点。
YOLO 原始框已经受推理引擎内部默认阈值影响，不是所有候选框。
普通 `detect` 同样返回数量诊断，仅 `detect_debug` 保存图片。
本入口尚需现场运行验收；返回坐标在 MoveIt base_link 下，单位米。


## 现场扫描与投放姿态（2026-10-10）

`site.json` 的任务参数使用标准 UR 关节顺序，配置内单位为弧度：

| 用途 | 示教器关节角（度） |
|---|---|
| Home 扫描 | `[-180, -90, 127, -123, 270, 0]` |
| 示教投放点 | `[-148.38, -87.44, 132.69, -131.78, 269.45, -0.33]` |

`scan_at_home_only=true` 表示在 Home 稳定后检测，不再发送历史示例的额外笛卡尔扫描或扫描重试运动。
`drop_motion=joint` 使用 `drop_joint_pos` 规划投放运动，成功返回后才松爪；失败或状态未知时中止，保持夹持。
`return_to_scan_after_drop=false` 表示本次流程在投放后结束，不再额外返回 Home；若配置多轮，下一轮仍先去 Home。
旧配置不指定这些参数时，保留原来的扫描、笛卡尔投放和结束返回扫描行为。

示教终点记录不等于已验证抓取到投放的整段轨迹。照片中刀具位置栏是在所选特征下的当前 TCP 位姿，
不能当作安装设置中的 TCP 偏移，也不能直接把示教器姿态数值填成规划器欧拉角。
本次只更新扫描和投放；未修改 `pick_offset`、`pick_orientation` 或标定确认值。

```bash
git pull --ff-only origin main
bash scripts/project build --packages-select demo_package harvesting_bringup
```

以上命令只同步与构建，不启动抓取。


## 完整流程显式执行（2026-10-10）

已有的一键入口 `project demo` 启动真实 UR 驱动、MoveIt、视觉服务、夹爪服务和采摘流程。
`calibration_verified=false` 不再是启动门槛，启动预检和流程节点两处均已调整；保留未核验状态提示。
不带 `motion_enabled:=true` 仍会在启动设备前拒绝采摘。fake 模式仍禁止物理执行。
规划或夹爪失败后中止，投放运动成功后才松爪。抓取偏移和方向未在这次修改中调整。

先停止已有相机、机械臂或联合启动进程，再在现场一个终端执行：

```bash
git pull --ff-only origin main
bash scripts/project build --packages-select demo_package harvesting_bringup
bash scripts/project demo motion_enabled:=true rviz:=false
```

最后一条会真实发送动作：Home 检测、抓取、示教点投放、松爪。
使用 ROS 环境自动加载和现有独立推理环境，不需要另开服务调用终端。
`max_cycles=1` 限制轮数，一轮可能处理多个有效检测目标；不表示只抓一个苹果。
离线检查不等于现场全链路执行通过。独立 Home 入口的 600/15 秒执行监护只属于 Home，
完整流程仍使用既有 MoveIt 执行管理和流程服务超时，不应混称为同一套监护。
