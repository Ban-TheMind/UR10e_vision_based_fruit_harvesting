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
相机独立启动可以检查图像；要返回基座坐标，还需有效 TF。
启动多个终端时不要重复启动同一个设备；`all` 已包括三个设备。

完成现场核对后，要允许手动运动/夹爪请求：

```bash
./scripts/project all motion_enabled:=true
```
启动本身不发送运动或夹爪命令，但会启用请求执行。
RViz 自身的 Execute 按钮也可能驱动机械臂，不能把规划模块的 plan-only 当作全系统运动锁。
先以操作人员确认的目标低速单步测试，再运行任务。

## 视觉坐标与标定

默认 `coordinate_mode: legacy` 保留原有换轴、偏移和符号逻辑，仅供追溯和对比；没有认证其正确性。
现场确认 optical frame、规划目标帧、深度单位和相机安装方式后：

1. 设置 `coordinate_mode: tf`；`camera_frame` 填实际光学帧，`target_frame` 填实际规划目标帧。
2. 若相机已有正确 TF，不额外发布静态 TF。
3. 若需要发布 Eye-to-Hand 静态 TF，填写 deployment 的 source/target 帧，启用 `publish_camera_tf`。
   标定文件保存 Camera → Base 的坐标变换；TF 的 parent 是 target，child 是 source。
   Eye-to-Hand 固定相机设置 `camera_mount: external`，自动选用无腕部相机连接的模型。若静态标定直接连接光学帧，还须设 `camera_publish_tf: false`，避免重复 TF parent。
4. 完成 `field_verification.md` 后再将 `deployment.calibration_verified` 设为 true。

当前标定默认保存于 `src/harvesting_bringup/config/calibration/camera_to_base.json`。
不要因为程序能启动就将标定标记为已验证。

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
