# UR10e 视觉桌面苹果抓取

当前维护版本为 [Ban-TheMind fork](https://github.com/Ban-TheMind/UR10e_vision_based_fruit_harvesting) 的 `main`。
项目基于 [David Nie 的原始水果采摘项目](https://github.com/DaviddNie/UR10e_vision_based_fruit_harvesting)，
保留 ROS2 服务和 MoveIt 规划，适配实验室的外部固定相机与 Robotiq 夹爪。

**2026-10-12 现场结果：操作者确认完成了真实苹果的视觉检测、抓取、搬运和投放。**
本次成功使用固定高度、固定朝向及 `base_link` X 方向 −0.07 m 的经验补偿。
物理链路已经打通；相机外参仍需重新标定，当前结果不是全工作空间精度验收。
首次成功未开启录像；下方原作者视频不代表本实验室的本次试验。

![原项目示意图](img/teaser.png)

## 当前装置与环境

| 项目 | 现场配置 |
| --- | --- |
| 机械臂 | UR10e，IP `192.168.11.60`，headless 远程控制 |
| 夹爪 | Robotiq 2F-85，官方 C++ SDK 桥接，串口 Modbus RTU |
| 串口 | `/dev/ttyUSB0`，115200/8N1，slave 9 |
| 相机 | 固定外部 RealSense D435I，序列号 `406122071837` |
| 图像 | RGB + 对齐深度，640×480、15 FPS |
| 运行环境 | Ubuntu 20.04 / ROS2 Foxy |
| ROS Python | 项目 Python 3.8 venv：`ur10e-project-foxy` |
| YOLO | 独立 VPP Python 子进程；`best.pt` 的类别 0 为 apple |
| 规划 | 现有 MoveIt/OMPL；接近、抓取、撤离分别提交位姿目标 |

ROS/Foxy 系统包由 Ubuntu 提供，项目 venv 读取这些系统包；并非所有依赖都安装在 venv 内。
旧 Humble MoveIt 配置包不参与当前构建。维护入口统一为 `bash scripts/project`，
现场默认使用 `build-project/`、`install-project/`、`log-project/`。

## 已验证的流程

新增的点云建图与只规划入口是 `bash scripts/project mapping`；它不启动 YOLO/夹爪，
不允许轨迹执行。相机 TF、点云清理、MoveIt OctoMap 更新及地图就绪门槛已接入代码，
Foxy 编译与现场地图/规划对照仍待验证。见[点云建图验收说明](docs/pointcloud_mapping.md)。

```text
Home（扫描姿态）→ 检测一个苹果 → 张开夹爪
→ MoveIt 接近上方 → MoveIt 到抓取点 → 闭爪
→ 撤回上方 → 示教的关节投放姿态 → 松爪 → 结束
```

每轮在 Home 获取一次目标，后续固定使用这次坐标；当前没有运动中的连续视觉伺服。
苹果 XY 来自实时检测，不是写死的抓取坐标；抓取高度、方向、残余 XY 补偿和投放姿态固定。
默认仅处理一个苹果，不自动继续处理多个目标。

| 验证项 | 结果与边界 |
| --- | --- |
| 机械臂 Home | 真实动作完成，实测关节接近指定 Home |
| 相机链路 | RGB、对齐深度、内参及 YOLO 检测有返回；曾检测到苹果但无有效深度，调整物品位置后获得坐标 |
| 机器人模型 | 同一组关节角的 FK 与 RTDE 控制器 TCP 经坐标系转换后吻合；该次位置差约 0.005 mm |
| 上方接近 | 2026-10-12 `above-check`：目标/实际 tool0 位置误差 0.468 mm、方向误差 0.0398° |
| 完整抓取投放 | 操作者确认成功；使用 X −70 mm 固定补偿；不是自动抓取成功率统计 |

上述上方误差衡量机械臂到达命令目标的精度，**不是相机定位苹果的绝对误差**。

## 更新与现场运行

先停止旧的驱动、相机或联合启动进程，再更新和构建：

```bash
cd ~/Desktop/robot_learning/UR10e_vision_based_fruit_harvesting
git pull --ff-only origin main
bash scripts/project build
```

主要入口：

| 命令 | 行为 |
| --- | --- |
| `bash scripts/project check` | 离线检查，不连接硬件 |
| `bash scripts/project robot-check` | 独立启动驱动并检查，不发送运动 |
| `bash scripts/project home` | 独立启动驱动并执行 Home |
| `bash scripts/project camera-check` | 检测并保存 RGB/检测图，不启动机械臂或夹爪 |
| `bash scripts/project all motion_enabled:=true rviz:=false` | 启动联合服务，之后接受动作请求；本身不自动抓取 |
| `bash scripts/project above-check --execute ...` | 连接已有 all 服务，Home→检测→上方接近→立即读回位姿，不下降、不操作夹爪 |
| `HARVEST_PROFILE=... bash scripts/project demo motion_enabled:=true rviz:=false` | 自行启动子系统并执行一轮完整抓取投放；不能与 all 并行 |

**仓库默认 `site.json` 的三个示教参数仍为空。**本次成功使用独立试验配置，
没有把临时补偿写成所有工位通用的默认标定。直接运行未配置的 demo 会提示缺少参数。
完整的配置生成、上方试验及抓取复现命令见 [操作手册](docs/operations.md)。

## 本次成功使用的几何参数

| 参数 | 值与定义 |
| --- | --- |
| 抓取高度 | `tool0` 在 `base_link` 中 Z = `0.18425452796496608` m |
| 工具朝向 | RPY = `[3.0721570831557177, -0.004518726175556632, 0.14199025134474497]` rad |
| 工具偏移 | `tool_offset_m=[0, 0, 0.1628]`，从 tool0 沿自身 +Z 到模型夹爪中心 |
| 残余补偿 | `xy_offset_m=[-0.07, 0.0]` m，在 base_link 中作用于相机目标 |
| 上方间距 | `clearance_m=0.12` m |
| 投放关节角 | `[-148.38, -87.44, 132.69, -131.78, 269.45, -0.33]`°，标准 UR 顺序 |

工具变换为 `p_center = p_tool0 + R * tool_offset_m`，
规划目标反算为 `p_tool0 = p_center - R * tool_offset_m`。
高度来自示教 tool0，程序不会再次对示教高度扣减 162.8 mm。
XY 残余补偿与工具长度变换是两项不同参数；正负 X/Y 指机器人 base_link，不能理解为照片里的左右。
夹爪保留张开 85 mm、闭合目标 0 mm、force 原始值 40（不是 40 N）；接触物体可提前停止闭合。

## 已解决的问题与仍需完成的工作

已处理的主要问题包括 Foxy 模型/启动兼容、控制器选择与 headless、RealSense 双层话题名、
真实权重与独立推理环境、执行时长与速度缩放不匹配、树上侧抓流程改为桌面抓取，
以及夹爪中心与 tool0 的几何转换。详细证据见 [问题复盘](docs/troubleshooting_learning.md)。

当前仍需完成：

- 按师兄建议重新进行当前外部相机的标定，替代 X −70 mm 的局部经验补偿；跨位置精度尚未验证。
- 检查 YOLO 框中心/深度取点与真实抓取中心的差异，以及不同苹果尺寸的影响。
- 整理驱动 SIGINT 清理后的 exit −6、相机 HW not ready 参数警告和完整异常堆栈。
- 核实图像绿色偏色原因；目前没有确认根因。
- 增加可量化的夹持成功判定、重复试验记录与现场视频；服务成功不等同于可靠夹持。
- 点云/Octomap 已新增独立 `mapping` 只规划模式，尚待现场验收；既有 all/demo 不自动启用它。动态避障和多目标任务仍未实现。

改变苹果位置会重新检测，但固定补偿不保证在另一位置仍准确。当前工作以熟悉机械臂、
跑通一个完整物理项目为目标，不宣称已实现树上采摘或任意位置抓取。

## 文档导航

- [操作手册：复现命令、配置与坐标定义](docs/operations.md)
- [问题复盘：故障证据、处理与未解决项](docs/troubleshooting_learning.md)
- [Foxy 部署与依赖](docs/foxy_ubuntu2004.md)
- [分层测试与验收边界](docs/test_plan.md)
- [Robotiq SDK 与服务语义](docs/robotiq_serial.md)
- [分支维护约定](docs/branch_alignment.md)

## 原作者演示资料

作者 David Nie，导师 Dr. Leo Wu。以下视频与图片来自原项目，包含水平抓取和树上水果采摘；
其原 OnRobot RG2 HTTP 接口、历史启动步骤和树上抓取偏移不适用于当前实验室入口。


### Vision-based pick and place at varying heights on a horizontal surface

- **Planner**: RRTConnect from ompl
- **Cycle Time**: 17-20s
- **Video Link**: https://youtu.be/pAT8Y0UHFJc
![alt text](img/hor_demo.png)

### Vision-based fruit harvesting

- **Planner**: TRRT from ompl
- **Cycle Time**: ~75s
- **Video Link**: https://youtu.be/1r7PfkH8pU8
![alt text](img/ver_demo.png)
