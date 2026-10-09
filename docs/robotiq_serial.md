# Robotiq 2F-85: official SDK through ROS 2 services

## Dependency and execution chain

The device driver is now the official [Robotiq C++ SDK](https://github.com/robotiq/grippers), version 1.1.0 at commit `9017b5707cb28a8ea21fd7c4e84a50f7c56aaa5d`. The unmodified SDK source and BSD-3-Clause license are in `src/robotiq_sdk_bridge/vendor/robotiq`. Its libserialport 0.1.2 dependency is included as an unmodified source release with a checked SHA256 and LGPL license/source. No network or manual library installation is required during builds.

This is not the community `castetsb/pyRobotiqGripper` Python library previously used in the reference project. The custom RTU packing/CRC/serial implementation has been removed.

```
ROS client -> /gripper_cmd or /reset_gripper_cmd
  -> gripper_server.py (permission and input checks)
  -> robotiq_sdk.py (bounded subprocess call)
  -> robotiq_sdk_command (project policy adapter)
  -> official Robotiq::GripperModbusClient -> official serial transport -> device
```

We deliberately use the SDK's supported transaction API rather than its continuously exchanging Gripper class: each service request is bounded, and the status diagnostic must perform only a read without writing any command/activation registers. Project code selects commands and waits for completion; the official SDK owns register serialization, RTU framing, CRC, response validation, and serial transport. This is a synchronous service integration, not the official ros2_control/action stack. The helper owns and closes the connection per operation; constructing the Python adapter or starting the ROS node does not connect or activate anything.

Recorded field settings: `/dev/ttyUSB0`, 115200, 8N1, slave 9. Confirm the port on each computer; a stable `/dev/serial/by-id/...` path is preferable. Only one program should control the port. Serial access requires device permissions (usually the `dialout` group). There is no automatic scanning, activation or retry, and no host latency/sysfs modification.

## Build and startup

On this Ubuntu Humble workstation:

```bash
./scripts/pixi install --frozen
./scripts/project build
./scripts/project check
./scripts/project gripper
```

On a system ROS workspace, use its ROS environment and build `colcon build --packages-up-to gripper`, then source `install/setup.bash`. The bridge needs a C/C++17 compiler, CMake >=3.16 and make; it builds the pinned transport dependency itself. No pyserial/minimalmodbus/pymodbus installation is needed for this backend. The main and Foxy deployment branches use the same official SDK and fixed-camera geometry; another computer must still pull and rebuild its selected branch.

Parameters on `gripper_server`: `serial_port`, `baudrate`, `slave_id`, `serial_timeout` (0.001..3600 seconds), `action_timeout` (seconds), `speed` (raw 0..255). Set these in `src/harvesting_bringup/config/lab.yaml`. `allow_gripper_commands` remains false by default on Foxy. Enable it only explicitly for supervised tests.

## Service units and completion

`GripperCmd.width`: nominal opening in mm, 0..85; 85=open, 0=closed. The linear raw-position conversion is not a calibrated finger-gap measurement; verify real fingertips and intermediate openings on site.

`GripperCmd.force`: raw 0..255, NOT newtons; default 40 means raw 40. Speed defaults to raw 64. These values are not certified fruit-safe; raw force 0 means the device's minimum force.

The pinned SDK uses FC03 to read status and FC16 to write commands. The previous self-written driver used FC04 for status reads. The SDK/installed-firmware combination must therefore be verified first with the read-only status command on the real device; prior successful FC04 communication does not prove this new backend works.

Move requests check readiness before writing and wait for matching target echo, readiness, go-to flag and terminal object status. Closing contact or reaching target returns success, but neither guarantees reliable grasping. Opening contact is an error. Fault/transport failure/timeout returns failure. A timeout or stopping the helper does not guarantee physical motion stopped; inspect before retrying.

`ResetGripperCmd(reset_gripper=true)` explicitly clears activation, waits for reset, sets activation and waits for fault-free readiness. Device firmware performs calibration, possibly moving fingers. It is not tool-power reset or robot homing. Normal move requests never automatically activate an unready device.

## Read-only field check

After building, with no other controller using the port:

```bash
./scripts/pixi run --frozen ros2 run gripper gripper_status --port /dev/ttyUSB0
```

This issues one SDK status read and no activation/motion writes. A successful read verifies communication, not grasping. Keep actuator commands disabled until supervised field testing. Tool geometry, TCP, calibration and robot poses still need separate verification.

## Offline verification

```bash
./scripts/pixi run --frozen python -m unittest discover -s src/gripper/test -p 'test_*safety.py'
./scripts/pixi run --frozen python -m unittest discover -s src/gripper/test -p 'test_robotiq_sdk.py'
./scripts/pixi run --frozen ctest --test-dir build/robotiq_sdk_bridge --output-on-failure
```

Python tests verify the service guards and SDK process boundary. Native tests link the actual official SDK with a simulated serial fixture (construction sends nothing, status sends only a read, corrupt replies fail), and test project completion/fault/activation policies without devices. Hardware motion has not been tested on this workstation.
