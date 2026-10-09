# 真机部署前现场核对清单

状态：**未获准进行自动运动**。`src/harvesting_bringup/config/calibration/camera_to_base.json` 保存了用户提供的 Eye-to-Hand 标定，但尚未接入 TF 或抓取计算。

## 1. 标定坐标系（阻断项）

- [ ] 确认矩阵输入 Camera 的准确 ROS 帧名：`camera_color_optical_frame`、`camera_depth_optical_frame`、`camera_link`，还是其他安装坐标系？记录帧定义和轴方向。
- [ ] 确认矩阵输出 Base 的准确 ROS 帧名：`base`、`base_link`，还是现场定义的 `world`？记录它与 MoveIt planning frame 的关系。
- [ ] 确认相机是固定在环境中的 Eye-to-Hand，还是固定在 UR10e 法兰上的 Eye-in-Hand。当前 `end_effector_withDriverSupport.xacro` 把 `camera_link` 固定在 `flange`，与 Eye-to-Hand 声明冲突。
- [ ] 记录标定来源、日期、所用相机序列号和机械臂编号。确认矩阵方向确为 Camera → Base，单位为米，且是当前安装状态的标定。
- [ ] 用至少三个分布在工作空间中的已知点核对相机测得的 Base 坐标，记录每点误差；验证后才接入 TF 和运动目标。
- [ ] 核对 `robot_calibration.yaml` 的 hash 是否与这台 UR10e 的工厂标定一致。

## 2. 设备与通信

- [ ] 从现场电脑确认 UR10e 地址 `192.168.11.60`、网段和双向通信；确认 External Control URCap 与程序状态。
- [ ] 从 RealSense 驱动日志确认所选设备序列号为 `406122071837`，并确认彩色图、对齐深度图、CameraInfo 的实际话题和帧名。
- [ ] 核对深度图编码和深度单位；当前代码固定按毫米乘 `0.001`，必须与实际消息一致。
- [ ] 确认 Robotiq 2F-85 串口 `/dev/ttyUSB0`、115200/8N1、从站 9、供电和开合方向；先只读状态。

## 3. 运动与安全

- [ ] 核对真实 TCP、工具质量/惯量、相机和夹爪的碰撞几何；当前模型使用简化盒体。
- [ ] 核对规划场景中的墙、桌子、天花板尺寸和位姿，以及固定扫描点、抓取偏移、投放点是否适用于现场。
- [ ] 明确防护区域、急停位置、操作人员和低速单步联调流程。先验证静态感知，再验证空载轨迹，最后验证抓取。
- [ ] 现场验证运动执行结果检查与失败中止：代码已检查规划/执行、夹爪响应和请求超时，但尚未真机验证。

## 4. 代码接入说明

现场参数统一在 `src/harvesting_bringup/config/lab.yaml`，操作入口见 [operation.md](operation.md)。
原有换轴/偏移转换保留为 `legacy` 模式；`tf` 模式使用光学 XYZ 和指定 TF。
未确认标定与帧名之前，不开启自动任务。规划、夹爪失败和请求超时会阻断后续任务动作。
现场需要验证这些中止行为以及真实运动、夹爪响应含义。
