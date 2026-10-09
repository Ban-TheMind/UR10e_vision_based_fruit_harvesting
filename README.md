# Vision-based fruit havesting

**Author**: David Nie  
**Supervisor**: Dr. Leo Wu

**Robot**: UR10e  
**Laboratory gripper**: Robotiq 2F-85, USB/RS-485 Modbus RTU (115200/8N1, slave 9)
**Depth Camera**: Intel RealSense (confirm laboratory device configuration)
**Dev Env**: `Ubuntu 22.04`  
**Dev tools**: `MoveIt!`, `ROS2 Humble`, `YOLOv11`, `hand-eye calibration`  

![alt text](img/teaser.png)

## Table of Contents
- [Vision-based fruit havesting](#vision-based-fruit-havesting)
	- [Table of Contents](#table-of-contents)
		- [TO-DO list](#to-do-list)
		- [Workspace setup](#workspace-setup)
	- [Demo videos](#demo-videos)
		- [Vision-based pick and place at varying heights on a horizontal surface](#vision-based-pick-and-place-at-varying-heights-on-a-horizontal-surface)
		- [Vision-based fruit harvesting](#vision-based-fruit-harvesting)
	- [How to run the demos](#how-to-run-the-demos)
		- [Horizontal Pick \& Place](#horizontal-pick--place)
		- [Vertical Fruit Harvesting](#vertical-fruit-harvesting)
	- [Depth Camera Visualisation](#depth-camera-visualisation)
	- [YOLO-based object detection + transformation using the depth camera](#yolo-based-object-detection--transformation-using-the-depth-camera)
	- [How to test Gripper](#how-to-test-gripper)
		- [End-effector Visualisation](#end-effector-visualisation)

### TO-DO list 
* Improve fruit harvesting cycle time by trying out different planners
  * RRT*
  * CForest which supports multi-threading
* Design custom gripper to improve gripping efficiency/accuracy


### Workspace setup

本工位使用 Ubuntu 24.04、glibc 2.39 和 Pixi 隔离的 ROS 2 Humble 环境。
原始研究开发环境为 Ubuntu 22.04；当前锁定依赖要求 glibc 2.39。

```bash
./scripts/pixi install
./scripts/project build
./scripts/project check
./scripts/project fake
```

- [工位与实验室操作](docs/operation.md)
- [模块职责](docs/architecture.md)
- [真机部署核对清单](docs/field_verification.md)

现场配置入口为 `src/harvesting_bringup/config/lab.yaml`；修改配置无需重新构建。
`fake` 使用假硬件，不是物理仿真。默认启动不运行自动采摘。

## Demo videos

### Vision-based pick and place at varying heights on a horizontal surface

**Planner**: RRTConnect from ompl  
**Cycle Time**: 17-20s  
**Video Link**: https://youtu.be/pAT8Y0UHFJc  
![alt text](img/hor_demo.png)

### Vision-based fruit harvesting

**Planner**: TRRT from ompl  
**Cycle Time**: ~75s  
**Video Link**: https://youtu.be/1r7PfkH8pU8  
![alt text](img/ver_demo.png)

## How to run the demos

Current laboratory entry points:

```bash
./scripts/project camera
./scripts/project robot
./scripts/project gripper
```

See [operation.md](docs/operation.md) for explicit actuator enablement and the separate automatic task entry.
The original horizontal strategy remains as source for comparison; the default field task is vertical harvesting.
Do not use the original historical launch commands as the field procedure.

## Depth Camera Visualisation

![alt text](img/depth_camera_visual.png)


## YOLO-based object detection + transformation using the depth camera

![alt text](img/apple1.png)
![alt text](img/apple2.png)

## How to test Gripper

Use `./scripts/project gripper` to start the request endpoint without actuating the device.
Enable commands explicitly as described in [operation.md](docs/operation.md).
Robotiq 2F-85 uses the pinned official [Robotiq C++ SDK](https://github.com/robotiq/grippers) over serial Modbus RTU. The SDK and its serial transport source are included; build with `./scripts/project build`. Serial settings are configured in `lab.yaml`.
See [robotiq_serial.md](docs/robotiq_serial.md) for units, read-only status, and explicit activation.

### End-effector Visualisation

![](img/end_effector_visualisation.png)
