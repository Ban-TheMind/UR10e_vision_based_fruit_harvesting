# Robotiq 2F-85 serial control

The laboratory gripper is Robotiq 2F-85, controlled over RS-485 Modbus RTU via a USB adapter. The previous OnRobot RG2/Eye Box HTTP calls do not control this device.

Recorded field settings: `/dev/ttyUSB0`, 115200 baud, 8 data bits, no parity, 1 stop bit, slave address 9. Confirm the adapter path on each computer; a stable `/dev/serial/by-id/...` path is preferable. No node opens or activates a device at startup. Only one controller should use the serial port at a time. On Linux, the driver requests exclusive access; the operator needs serial-device permissions (usually the `dialout` group).

## Dependencies and configuration

Foxy/system ROS: install `python3-serial` in the Python environment used by ROS (or `pyserial` in an existing virtual environment). The Humble Pixi environment includes `pyserial`; update it with `./scripts/pixi install --frozen` and rebuild the workspace. No `pymodbus` dependency is needed.

Parameters on `gripper_server`: `serial_port`, `baudrate`, `slave_id`, `serial_timeout` (seconds), `action_timeout` (seconds), `speed` (raw 0..255). Humble configures these in `src/harvesting_bringup/config/lab.yaml`; Foxy can override them with ROS parameters. Preserve the branch-specific command enable switch: `commands_enabled` on Humble and `allow_gripper_commands` on Foxy, both default false.

## Service units and completion

`GripperCmd.width`: nominal opening in millimetres, 0..85; 85 means fully open and 0 fully closed. The mapping to raw position 0..255 is a nominal linear conversion, not a calibrated measurement of actual finger spacing. Fingertips and installation affect intermediate gaps; verify them on site.

`GripperCmd.force`: raw Robotiq register value 0..255, **not newtons**. The client default 40 now means raw 40. `speed` is also raw 0..255 (default 64). Force is not an absolute calibrated force or a fruit-safe setting; choose it in supervised field tests. Raw force 0 still commands the device's minimum force.

The driver reads status with FC04 starting at 0x07D0 and writes three command registers with FC16 starting at 0x03E8. CRC, slave, function, response length and write acknowledgement are checked. Motion returns success only after readiness, command-position echo, and terminal object status. Closing contact is reported as a completed grip; reaching the requested position does not prove an object was grasped. Opening contact returns failure. Faults, incomplete replies and timeouts return failure. A timeout does not guarantee that physical motion stopped: stop the task and inspect the device before retrying.

`ResetGripperCmd(reset_gripper=true)` explicitly clears and sets activation and waits for activation status. This is **not** the old tool-power reset, nor a robot homing command. Activation can move the fingers during device calibration. Ordinary move requests never automatically activate an unready gripper.

## Read-only first check

After building and sourcing the workspace, with no other serial controller running:

```bash
ros2 run gripper gripper_status --port /dev/ttyUSB0
```

This sends only a status read; it never writes activation or motion registers. A successful status read verifies communication, not physical grasping.

Keep actuation disabled until supervised testing. Then explicitly enable the branch's command switch, activate through the reset service if required, and test a chosen opening/force. The existing tool meshes and collision model have not been replaced or validated for the Robotiq installation; verify actual geometry and TCP before planning near objects. Automatic harvesting and calibration remain subject to the existing branch safeguards.

## Offline verification

```bash
python3 -m unittest discover -s src/gripper/test -p 'test_*safety.py'
python3 -m unittest discover -s src/gripper/test -p 'test_robotiq_rtu.py'
```

Tests use simulated serial replies and do not access hardware. Actual serial wiring, installed firmware, calibration and physical motion still require field verification.

Protocol reference: [Robotiq official control manual](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM-OMRON_InstructionManual_HTML5_20190118.zip/2F-85_2F-140_TM-OMRON_InstructionManual_HTML5/Content/4.%20Control.htm).
