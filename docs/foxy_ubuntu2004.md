# Ubuntu 20.04 / ROS 2 Foxy 现场适配

当前 main 使用机械臂旁电脑已有的 `/opt/ros/foxy`，不安装另一套 Ubuntu，也不修改 `VPP` 环境。仓库中的 `ur10e_moveit_config*` 是旧的 Humble 配置，已从当前版本的 colcon 构建中排除。Foxy 的 UR 驱动和 MoveIt 配置由 Foxy 软件包提供；本项目的两个 `foxy_*.launch.py` 使用相同的末端工具模型和 `robot_calibration.yaml`。

启动文件和控制器参数以 [Universal Robots 官方 Foxy 分支](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy) 为参照；本地核对时该分支提交为 `18487f58ec17`。项目内置 `config/ur_controllers.yaml`，因此控制启动不再要求单独安装 `ur_bringup`。现场软件包版本和相机包装器的实际参数仍由 `tools/foxy_site_preflight.sh` 检查。

## 依赖

现场已有 Foxy 核心，但缺少 UR 驱动、MoveIt、RealSense ROS 包。安装前再次用 `apt-get -s` 确认不会升级或卸载现有软件包。下面的命令只补充 Foxy 模块，不需要 `ros-foxy-ur-bringup` 或 MongoDB：

```bash
sudo apt-get install --no-install-recommends \
  ros-foxy-ur-robot-driver ros-foxy-ur-controllers \
  ros-foxy-ur-description ros-foxy-ur-moveit-config \
  ros-foxy-controller-manager ros-foxy-joint-state-broadcaster \
  ros-foxy-joint-trajectory-controller \
  ros-foxy-moveit-ros-planning-interface ros-foxy-moveit-ros-move-group \
  ros-foxy-moveit-planners-ompl ros-foxy-moveit-simple-controller-manager \
  ros-foxy-realsense2-camera ros-foxy-xacro
```

统一入口默认读取 `/home/ubuntu/Desktop/robot_learning/.venvs/ur10e-project-foxy`。早期记录使用 `.venvs/ur10e-foxy`；应按现场实际目录设置 `UR10E_FOXY_VENV`，本次没有创建或迁移环境。项目专用 Python 3.8 环境使用 `--system-site-packages` 读取 Foxy 的 `rclpy`、`cv_bridge` 等 Ubuntu 包；通过该环境安装的 Python 包只写入自己的目录。构建工具 `colcon-common-extensions` 和兼容的 `pytest` 已装在此环境。YOLO 推理由单独子进程调用现有 VPP 的 Python 3.10，仅读取其包和模型，不向 VPP 安装任何东西。可通过相机节点参数 `inference_python` 指向日后新建的推理环境。

`src/camera/models/best.pt` 由 Git LFS 管理。普通 Git 检出可能只得到文本占位文件；运行 `tools/test_inference_worker.py` 前必须取回真实权重。当前版本会在启动相机节点时明确检查这一点。

## 编译和无运动检查

在宿舍电脑（Windows 或 Linux）可以先运行：

```bash
python tools/offline_preflight.py
```

此入口检查 Python/ROS 包配置语法、现场 IP/序列号配置、Foxy 包布局、相机像素到空间坐标、深度单位、YOLO 子进程通信协议、夹爪默认禁止设备命令，以及 C++ 运动请求策略（若有 `g++`）。它只处理仓库内的数据，不连接 ROS 图、相机或机械臂。模型文件若仍是 Git LFS 占位符会给出提示；可加 `--require-model` 把该情况作为失败。

回到 Ubuntu 20.04 主机后，先保留并核对现场目录中的未提交改动，再同步当前版本。然后运行：

```bash
bash tools/foxy_site_preflight.sh
bash tools/foxy_site_preflight.sh --build
```

第一条读取 Foxy 包清单、运行离线检查，并在已有构建目录时展开 UR10e Xacro；第二条额外在本项目目录构建后展开 Xacro，核对机械臂、工具和 ros2_control。两条命令均不启动驱动、不发送运动命令，也不安装系统包。脚本会优先使用项目已有的隔离 Python 环境；如位置不同，可通过 `UR10E_FOXY_VENV` 指定。

完整编译通过后，才继续做相机话题、关节状态和静止目标的现场核查。静止目标验证需要独立测量的目标点，离线几何测试只能验证计算逻辑，无法证明手眼标定仍符合当前安装位置。

静止目标核验时，复制 `tools/static_target_template.json`，填入现场 `/camera/aligned_depth_to_color/camera_info` 的内参、目标像素和深度（米），以及**独立测量的 UR 控制器 `base` 坐标**。不要把程序预测坐标再填回测量值。运行 `python tools/check_static_target.py 记录文件.json`；它输出各点误差，并在任一点超过 `max_error_m` 时返回失败。模板的 3 cm 仅是示例阈值，应按现场测量误差和任务精度确定。

### 手工复核命令

```bash
cd /home/ubuntu/Desktop/robot_learning/UR10e_vision_based_fruit_harvesting
source /opt/ros/foxy/setup.bash
source /home/ubuntu/Desktop/robot_learning/.venvs/ur10e-project-foxy/bin/activate
./scripts/project build
source install-project/local_setup.bash
python -m unittest src/gripper/test/test_gripper_safety.py
g++ -std=c++17 -Wall -Wextra -Werror -I src/moveit_path_planner/src \
  src/moveit_path_planner/test/test_movement_request_policy.cpp \
  -o /tmp/test_movement_request_policy
/tmp/test_movement_request_policy
python tools/test_inference_worker.py
```

主入口是 `./scripts/project`，详见 [内部操作手册](operations.md) 和 [分层测试](test_plan.md)。旧 `display.launch.py` 已转入统一启动，默认 fake；真实机械臂使用 `project robot`，联合设备使用 `project all`，应在现场核对设备、控制器和工作空间后再启动。规划服务默认 `allow_execution=false`，夹爪服务默认 `allow_gripper_commands=false`。仅编译或启动并不代表机械臂的运动学、TCP、负载、障碍物和相机外参已经得到现场验证。

## 现场待核对

- `robot_calibration.yaml` 的运动学哈希必须对应 IP `192.168.11.60` 的实际控制器；它与相机到 `base` 的手眼矩阵是两份不同的标定。
- 确认控制器软件和 External Control URCap 与 Foxy UR 驱动 2.0.2 兼容。驱动启动后检查关节状态和 TF，但保持运动执行关闭。
- 确认 RealSense SDK 序列号 `406122071837` 的 ROS 话题来自固定在手外的那台相机，并检查 RGB、对齐深度、CameraInfo 的尺寸和编码。
- 该相机实际支持 640×480 的 6/15/30 fps；Foxy 启动配置使用 RGB 和深度均为 15 fps。
- 在自主运动前，用独立测量的静止点核查 Camera → Base 的位置误差，并逐项核查末端工具、TCP、负载、速度和作业边界。

夹爪后端已与 main 同步为官方 SDK；构建夹爪时会同时构建 `robotiq_sdk_bridge`，无需安装 Python 串口库。固定相机矩阵见相机包的 `camera/calibration/camera_to_base.json`，与 main 共用相同定义。


## Foxy headless 与 scaled 执行

2026-10-09 现场读取确认：已安装的 `/opt/ros/foxy/share/ur_description/urdf/ur.ros2_control.xacro`
将 `headless_mode` 硬编码为 `0`，上层 `ur.urdf.xacro` 也不转发此参数。
项目驱动 launch 在 xacro 展开后，只将真实 UR 硬件的该参数改为请求的 `1`/`0`，
并对缺失或重复参数立即报错。无需修改系统包；fake hardware 不受影响。
`headless_mode:=true` 仍要求示教器远程控制、已上电解除制动且安全状态正常。
启动后应读取 `/controller_manager` 的 `robot_description` 确认参数为 `1`，
并独立核对 runtime_state、速度缩放和 reverse 控制连接，不能将控制器启动当作程序运行成功。

选用 `scaled_joint_trajectory_controller` 时关闭 MoveIt 按名义轨迹时长计算的执行超时，
以允许示教器速度滑块改变实际执行时间；普通控制器保留原监控。
这不解决机器人程序停止或速度为零，也不证明 Home 完成。
发送运动的入口必须另设有限等待、故障/停滞取消，并核对 action 结果与最终关节角。


## 单终端控制入口

在现场已经完成 `install-project` 构建后，运行 `bash tools/ur10e_control.sh` 仅启动并检查驱动；
运行 `bash tools/ur10e_control.sh --home` 执行用户已授权的候选 Home `[-180,-90,127,-123,270,0]` 度。
入口自动加载 Foxy、`ur10e-project-foxy` venv、项目安装环境与 ROS_DOMAIN_ID=61。
Home 接受大于 0、最高 100% 的有效速度倍率，不再要求滑块为 5%；入口不更改滑块、不上电、不解除制动、不解锁停止。规划的速度和加速度缩放仍各为 0.1。
默认只检查，只有明确传入 `--home` 才会发送运动。

入口拒绝同时存在其他驱动或 MoveIt 控制进程，且只停止本次创建的进程组。
它核对示教器模式与安全状态、实际 URDF headless=1、RTDE 程序状态与速度、reverse 连接，
以及 scaled 控制器 active，随后仅做 MoveIt 规划并逐点核对现场六轴限位、起点和终点。
Home 执行使用 ExecuteTrajectory action，默认最长 600 秒；速度为零、程序停止、安全停止、
关节超限或默认 15 秒没有至少 0.05 度关节进展时取消。有效速度倍率高于 100% 或非有限值时拒绝执行。
Ctrl+C 会取消当前 action 并关闭本次启动的进程。全部日志保存在 `log-project/control-*`。
只有 action 成功且实测全部关节距目标不超过 0.5 度时，才输出 Home 完成与 home-result.json。

此入口没有增加桌面、支架、线缆的碰撞模型，不能将规划或限位检查当作完整物理安全验证；
操作人仍需按此前授权的现场路径留意真实空间。机器人程序和 Home 的真机完成状态应以本次输出为准。


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
