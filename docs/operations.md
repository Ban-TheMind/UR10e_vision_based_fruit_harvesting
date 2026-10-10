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
| `./scripts/project demo motion_enabled:=true` | 有限次数采摘 | 需在配置中确认现场标定；会发送动作 |
| `./scripts/project robot-check` | 独立单终端机械臂状态检查 | 自行启动驱动；不发送轨迹 |
| `./scripts/project home` | 独立单终端 Home 检查及动作 | 会发送运动；保留原有运行状态和最终关节核验 |

`robot-check`、`home` 是独立的现场专用入口，保留已审查的固定 IP 与 Home 目标，不读取联合启动配置。运行前停止已有机械臂控制进程；不要与 `robot`、`all` 或 `demo` 并行。其实现仍在 `tools/ur10e_control.py`。

`motion_enabled:=true` 在 `robot` 或 `all` 模式允许之后的服务请求执行动作，并不自动发送动作。夹爪开关也由同一参数传入。关闭它仅限制项目服务，不能阻止其他节点直接调用控制器。真实驱动连接本身可能启动机器人控制程序，因此真实模式应在现场具备操作条件时使用。

旧入口 `ros2 launch end_effector_description display.launch.py` 现在转入统一启动，默认 `fake`；需要真实设备时显式传入 `mode:=robot` 或 `mode:=all`。历史 `auxiliary.launch.py` 和 horizontal/tmp 演示保留用于追溯，不属于当前受测主入口。

## 配置与环境

联合启动配置在 `src/harvesting_bringup/config/site.json`，采用 JSON，离线解析不依赖 PyYAML。记录机器人 IP/headless、相机序列号和推理 Python、串口参数及流程参数。配置中的默认扫描和投放姿态继承历史示例，不视为现场已验收轨迹；默认 `calibration_verified=false`。

关节数组统一为 `[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`，单位弧度。笛卡尔位置为米，姿态为弧度，规划参考系为 `base_link`；夹爪宽度为标称毫米，力为 0–255 原始编码。

任务配置可补充流程节点声明的 `drop_position`、`pick_offset`、超时等参数。`calibration_verified` 只有在机械臂工厂标定、相机外参、扫描/抓取/投放姿态、TCP/负载和作业空间分别核验后才设为 `true`。设置该值不是测量的替代品。修改配置后重启相应节点。

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
