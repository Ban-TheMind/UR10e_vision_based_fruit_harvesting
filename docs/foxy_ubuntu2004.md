# Ubuntu 20.04 / ROS 2 Foxy 现场适配

此分支使用现场已有的 `/opt/ros/foxy`，不安装另一套 Ubuntu，也不修改 `VPP` 环境。仓库中的 `ur10e_moveit_config*` 是旧的 Humble 配置，已从本分支的 colcon 构建中排除。Foxy 的 UR 驱动和 MoveIt 配置由 Foxy 软件包提供；本项目的两个 `foxy_*.launch.py` 使用相同的末端工具模型和 `robot_calibration.yaml`。

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

项目专用 Python 3.8 环境位于 `/home/ubuntu/Desktop/robot_learning/.venvs/ur10e-foxy`。它使用 `--system-site-packages` 读取 Foxy 的 `rclpy`、`cv_bridge` 等 Ubuntu 包；通过该环境安装的 Python 包只写入自己的目录。构建工具 `colcon-common-extensions` 和兼容的 `pytest` 已装在此环境。YOLO 推理由单独子进程调用现有 VPP 的 Python 3.10，仅读取其包和模型，不向 VPP 安装任何东西。可通过相机节点参数 `inference_python` 指向日后新建的推理环境。

`src/camera/models/best.pt` 由 Git LFS 管理。普通 Git 检出可能只得到文本占位文件；运行 `tools/test_inference_worker.py` 前必须取回真实权重。本分支会在启动相机节点时明确检查这一点。

## 编译和无运动检查

在宿舍电脑（Windows 或 Linux）可以先运行：

```bash
python tools/offline_preflight.py
```

此入口检查 Python/ROS 包配置语法、现场 IP/序列号配置、Foxy 包布局、相机像素到空间坐标、深度单位、YOLO 子进程通信协议、夹爪默认禁止网络命令，以及 C++ 运动请求策略（若有 `g++`）。它只处理仓库内的数据，不连接 ROS 图、相机或机械臂。模型文件若仍是 Git LFS 占位符会给出提示；可加 `--require-model` 把该情况作为失败。

回到 Ubuntu 20.04 主机后，先保留并核对现场目录中的未提交改动，再同步本分支。然后运行：

```bash
bash tools/foxy_site_preflight.sh
bash tools/foxy_site_preflight.sh --build
```

第一条读取 Foxy 包清单、运行离线检查，并在已有构建目录时展开 UR10e Xacro；第二条额外在本项目目录构建后展开 Xacro，核对机械臂、工具和 ros2_control。两条命令均不启动驱动、不发送运动命令，也不安装系统包。脚本会优先使用项目已有的隔离 Python 环境；如位置不同，可通过 `UR10E_FOXY_VENV` 指定。

完整编译通过后，才继续做相机话题、关节状态和静止目标的现场核查。静止目标验证需要独立测量的目标点，离线几何测试只能验证计算逻辑，无法证明手眼标定仍符合当前安装位置。

静止目标核验时，复制 `tools/static_target_template.json`，填入现场 `/camera/camera/aligned_depth_to_color/camera_info` 的内参、目标像素和深度（米），以及**独立测量的 UR 控制器 `base` 坐标**。不要把程序预测坐标再填回测量值。运行 `python tools/check_static_target.py 记录文件.json`；它输出各点误差，并在任一点超过 `max_error_m` 时返回失败。模板的 3 cm 仅是示例阈值，应按现场测量误差和任务精度确定。

### 手工复核命令

```bash
cd /home/ubuntu/Desktop/robot_learning/UR10e_vision_based_fruit_harvesting
source /opt/ros/foxy/setup.bash
source /home/ubuntu/Desktop/robot_learning/.venvs/ur10e-foxy/bin/activate
colcon build --symlink-install
source install/setup.bash
python -m unittest src/gripper/test/test_gripper_safety.py
g++ -std=c++17 -Wall -Wextra -Werror -I src/moveit_path_planner/src \
  src/moveit_path_planner/test/test_movement_request_policy.cpp \
  -o /tmp/test_movement_request_policy
/tmp/test_movement_request_policy
python tools/test_inference_worker.py
```

主入口是 `ros2 launch end_effector_description display.launch.py`。它会连接真实 UR10e 和相机，因此应在核对设备、控制器和工作空间后再启动。规划服务默认 `allow_execution=false`，夹爪服务默认 `allow_gripper_commands=false`。仅编译或启动并不代表机械臂的运动学、TCP、负载、障碍物和相机外参已经得到现场验证。

## 现场待核对

- `robot_calibration.yaml` 的运动学哈希必须对应 IP `192.168.11.60` 的实际控制器；它与相机到 `base` 的手眼矩阵是两份不同的标定。
- 确认控制器软件和 External Control URCap 与 Foxy UR 驱动 2.0.2 兼容。驱动启动后检查关节状态和 TF，但保持运动执行关闭。
- 确认 RealSense SDK 序列号 `406122071837` 的 ROS 话题来自固定在手外的那台相机，并检查 RGB、对齐深度、CameraInfo 的尺寸和编码。
- 该相机实际支持 640×480 的 6/15/30 fps；Foxy 启动配置使用 RGB 和深度均为 15 fps。
- 在自主运动前，用独立测量的静止点核查 Camera → Base 的位置误差，并逐项核查末端工具、TCP、负载、速度和作业边界。
