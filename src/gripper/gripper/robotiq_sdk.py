"""Project service adapter for Robotiq's official C++ SDK.

No protocol packing, CRC, register IO or serial transport is implemented here.
The installed robotiq_sdk_bridge executable owns each bounded device operation.
"""
import json
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path


def position_for_width(width):
    if not math.isfinite(width) or not 0 <= width <= 85:
        raise ValueError('width must be 0..85 mm')
    return round((85 - width) * 255 / 85)


@dataclass(frozen=True)
class Status:
    flags: int
    fault: int
    requested: int
    position: int
    current: int

    @property
    def ready(self):
        return bool(self.flags & 1) and (self.flags >> 4) & 3 == 3

    @property
    def object_state(self):
        return self.flags >> 6 & 3


class RobotiqSDK:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, slave_id=9,
                 serial_timeout=0.5, action_timeout=10.0, speed=64):
        if not port or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError('Invalid serial port or baudrate')
        if not isinstance(slave_id, int) or not 1 <= slave_id <= 247:
            raise ValueError('slave_id must be 1..247')
        if not isinstance(speed, int) or not 0 <= speed <= 255:
            raise ValueError('speed must be a raw value in 0..255')
        if not math.isfinite(serial_timeout) or not 0.001 <= serial_timeout <= 3600:
            raise ValueError('serial_timeout must be 0.001..3600 seconds')
        if not math.isfinite(action_timeout) or action_timeout <= 0:
            raise ValueError('action_timeout must be finite and positive')
        self.port, self.baudrate, self.slave_id = port, baudrate, slave_id
        self.serial_timeout, self.action_timeout, self.speed = serial_timeout, action_timeout, speed

    def __enter__(self):
        # Creating/entering this object does not start a process or touch hardware.
        return self

    def __exit__(self, *args):
        return False

    def _run(self, operation, width=0, force=0):
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix('robotiq_sdk_bridge')) / 'lib/robotiq_sdk_bridge/robotiq_sdk_command'
        argv = [str(executable), operation, self.port, str(self.baudrate), str(self.slave_id),
                str(self.serial_timeout), str(self.action_timeout), str(self.speed), str(width), str(force)]
        # Activation has two waits. Killing the helper does not stop physical motion.
        timeout = 2 * self.action_timeout + 6 * self.serial_timeout + 5
        try:
            result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, check=False)
        except subprocess.TimeoutExpired as error:
            raise TimeoutError('SDK process timed out; physical device state unknown') from error
        if result.returncode:
            raise IOError(result.stderr.strip() or 'Official SDK command failed')
        try:
            return Status(**json.loads(result.stdout))
        except (ValueError, TypeError) as error:
            raise IOError('Invalid official SDK bridge response') from error

    def read_status(self):
        return self._run('status')

    def move(self, width, force):
        position_for_width(width)
        if not isinstance(force, int) or not 0 <= force <= 255:
            raise ValueError('force must be a raw Robotiq value in 0..255, not newtons')
        return self._run('move', width, force)

    def activate(self):
        """Explicit reset/activation; device calibration may move fingers."""
        return self._run('activate')


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Read Robotiq status using official SDK without actuating')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--slave-id', type=int, default=9)
    args = parser.parse_args()
    with RobotiqSDK(port=args.port, baudrate=args.baudrate, slave_id=args.slave_id) as gripper:
        print(gripper.read_status())
