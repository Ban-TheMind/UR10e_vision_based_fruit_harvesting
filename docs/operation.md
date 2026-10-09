# 工位与实验室操作

所有命令从项目根目录执行。启动程序不需要编辑 Python/C++ 脚本。
唯一现场配置入口：`src/harvesting_bringup/config/lab.yaml`。
`scripts/project` 每次直接读取这个源文件；修改配置不需要重新编译。
其他配置文件可用 `HARVEST_PROFILE=/绝对路径/profile.yaml` 选择；路径必须是绝对路径。

## 工位准备

```bash
./scripts/project build
./scripts/project check
./scripts/project fake
```
`fake` 是 UR 驱动假硬件与 MoveIt，不是物理仿真；不会启动真实相机、夹爪或采摘。
默认规划请求只生成计划，MovementRequest.success 为 false（未执行），防止任务误把规划成功当作已经移动；`fake motion_enabled:=true` 可执行到假硬件。

## 实验室按设备逐步测试

```bash
./scripts/project camera
./scripts/project robot
./scripts/project gripper
```
这些命令分别启动相机/感知、真实机械臂/MoveIt、夹爪请求入口。
默认夹爪请求被拒绝，规划请求只规划。不会自动开合夹爪或运行采摘。
夹爪使用 Robotiq 2F-85 串口 Modbus RTU；宽度 0..85 mm，力度为原始值 0..255（不是 N）。
先按 [robotiq_serial.md](robotiq_serial.md) 只读状态；reset 服务是显式复位/激活，可能产生校准动作。
相机独立启动可检查图像并用已加载的标定矩阵返回基座坐标，不需要机械臂在线。
启动多个终端时不要重复启动同一个设备；`all` 已包括三个设备。

完成现场核对后，要允许手动运动/夹爪请求：

```bash
./scripts/project all motion_enabled:=true
```
启动本身不发送运动或夹爪命令，但会启用请求执行。
RViz 自身的 Execute 按钮也可能驱动机械臂，不能把规划模块的 plan-only 当作全系统运动锁。
先以操作人员确认的目标低速单步测试，再运行任务。

## 视觉坐标与标定

默认使用固定外部相机：`camera_mount: external`、`coordinate_mode: eye_to_hand`。
坐标链路为彩色光学帧 XYZ → 标定矩阵 → UR `base` → MoveIt `base_link`；最后一步为 `(-x, -y, z)`。
对齐深度的 `16UC1` 按毫米转换，`32FC1` 直接按米使用；其他编码拒绝处理。
翻转检测图像时先恢复原始像素位置，再取深度和反投影。

唯一默认矩阵位于 `src/camera/camera/calibration/camera_to_base.json`，构建时随 camera 包安装。
可以通过配置中的 `camera_calibration_file` 或 camera 参数 `calibration_file` 选择同格式文件。
矩阵记录输入 `camera_color_optical_frame`、输出 `base`、单位米；加载时检查帧名与矩阵一致性。
默认不额外发布标定 TF。可选静态 TF 只用于显示，必须使用相同标定并避免重复父帧。
完成 [field_verification.md](field_verification.md) 的已知点检查后再标记 `calibration_verified: true`；目前仍为 false。

## 自动采摘（第二个终端）

```bash
./scripts/project demo motion_enabled:=true
```
需要前一个终端已用 `all motion_enabled:=true` 启动设备，并且配置标记已现场验证。
启动前检查 `lab.yaml` 中模型类别、扫描点、抓取偏移、投放点、夹爪宽度和规划场景。
默认只运行一轮；任何运动或夹爪失败、请求超时都会退出，不再发后续动作。
请求超时无法取消服务器上已经执行的运动；结果未知时应停止测试并检查设备。
不再自动复位/断电夹爪，也不再在运输前提前打开夹爪。

任务使用旧的关节请求顺序，配置 `joint_names` 明确记录该顺序；不要直接把标准顺序关节数组填入旧的扫描点。
水平与临时历史策略不作为当前实验室默认任务；临时策略保存在 `experiments/legacy/`。

## 记录现场数据

```bash
./scripts/project record -o log/lab_session_01
```
记录彩色图、对齐深度、内参、TF 和关节状态，便于返回工位分析。
录制入口自动读取配置中的图像、深度和内参话题名称。
Ctrl+C 停止程序不等价于机械臂急停；现场操作按实验室规则进行。
