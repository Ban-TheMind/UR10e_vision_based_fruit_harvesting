# 项目结构与分层测试

组织原则是让每层都有独立入口、前置条件和可判断的结果。夹爪成功不证明机械臂成功，规划成功不证明执行成功，离线坐标正确不证明现场标定正确。

## 目录职责

```text
README.md                    对外项目概述，保留原文
scripts/project              日常统一命令入口
scripts/ros_env.sh           Foxy 环境及构建 overlay 的唯一加载逻辑
scripts/check_profile.py     无 ROS 的部署参数检查
src/harvesting_bringup/       模式编排和现场配置
src/camera/                  固定相机、坐标转换、独立 YOLO 通信
src/moveit_path_planner/      规划／执行请求与策略
src/end_effector_description/ 工具模型、Foxy UR/MoveIt 适配
src/gripper/                 夹爪 ROS 服务
src/robotiq_sdk_bridge/       官方串口 SDK 桥接
src/custom_interface/        子系统间服务契约
src/demo_package/            有限次数流程与公共请求处理
src/ur10e_moveit_config*/    历史 Humble 配置，COLCON_IGNORE 排除
tools/                       单部件诊断、离线几何、静止目标和 Home 检查
tests/                       启动编排及跨部件流程的无硬件回归
docs/operations.md           当前内部操作入口
docs/troubleshooting_learning.md 问题、解决办法及学习复盘
build-project/install-project/log-project/ 本地构建产物，不提交
```

保留 `src` 中 ROS 包名和接口，避免为了视觉上的分类破坏 colcon、依赖或可执行入口。horizontal/tmp 历史流程仍在原路径，未恢复到默认采摘流程；它们没有本次有限循环和跨部件回归的验收保证。

## 验收顺序

| 层级 | 测试入口与前置条件 | 通过条件 | 不能据此推断 |
|---|---|---|---|
| 0 离线 | `project check`，NumPy/g++ 可用 | Python 3.8 语法、包入口、流程开关、几何、协议、执行策略通过 | Foxy 编译或真机成功 |
| 1 部署 | `project build` → `project site-check`，现场 Foxy PC | 包依赖可读、编译成功、Xacro 工具和坐标定义正确 | 控制程序运行或轨迹执行 |
| 2 假机械臂 | `project fake` | 无真实相机/夹爪节点，假关节/TF/MoveIt 就绪，纯规划成功 | 真机通信和安全状态正常 |
| 3 真实机械臂静态 | `project robot` 或独立 `project robot-check` | 实际关节、控制器、runtime、安全状态、reverse 连接及有效速度均可确认 | 机械臂已经完成目标动作 |
| 4 视觉 | `project camera`；先检查实际权重和 VPP 推理 | RGB/对齐深度/内参同源，检测有返回；独立静止点误差满足现场阈值 | 抓取点已经适合直接执行 |
| 5 夹爪 | 默认 `project gripper` 验证拒绝写入；现场显式开启后单独开合 | 宽度/状态可核对、动作有限时返回、故障不继续 | 机械臂可运动 |
| 6 机械臂动作 | 先查 runtime/速度；现场确认目标后 `project home` 或受控请求 | action 成功且实测关节到达目标，记录速度及停止状态 | 联合采摘碰撞环境和姿态已核验 |
| 7 联合静态 | `project all`，不开放动作 | 三类服务、坐标和设备归属正确，无重复控制节点 | 自动采摘可直接投入使用 |
| 8 单次采摘 | 标定、姿态和空间验收后 `project demo motion_enabled:=true` | 默认最多 1 轮；抓取、搬运、投放完成，各响应与实物一致 | 连续生产的成功率与稳定性 |

联合启动中 MoveIt 的 4 秒延时仅是启动顺序安排，不是“硬件已准备”的证明。采摘节点另有服务等待超时；真实动作仍依赖控制层的运行条件。假机械臂模式不包含虚拟视觉/夹爪，不能在此模式直接验收整套采摘。

## 自动回归实际检查什么

`tests/test_project_workflow.py` 替换 ROS 传输和 launch 对象，执行实际编排函数：确认各模式启动的子系统、真实/假控制器选择和默认禁止执行；拒绝未知参数、未显式启用运动和错误布尔值；检查未核验标定只提示、不改变状态记录；执行真实流程代码，确认夹紧后搬运到投放点再松开，夹爪失败后不继续，单个离群检测点也被过滤。它不启动 ROS，不证明现场 launch API 或设备兼容性。

公共流程对服务等待、调用等待、检测尝试和轮次设上限。服务超时后退出并说明状态未知；本地 future 取消不等于远程机器人停下。不要将退出流程当作急停机制。

## 每次实验留下什么

每次记录日期、操作者、`git rev-parse HEAD`、配置副本、ROS/驱动版本、启动模式、期望结果、实际结果、原始日志和结论。机械臂问题额外保存 runtime_state、speed_scaling、target_speed_fraction、控制器名称/状态、reverse 连接、action 返回和动作前后关节。

分别填写“代码通过”“现场通过”“待验证”，避免只有一句“测试成功”。复盘中的历史结论按当时证据保留，新实验结果单独追加，不能把新结果倒填成昨天的结论。
