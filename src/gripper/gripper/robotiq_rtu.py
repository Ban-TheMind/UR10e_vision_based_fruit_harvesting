"""Robotiq 2F-85 Modbus RTU transport; importing this module opens no device."""
import math
import os
import struct
import time
from dataclasses import dataclass


def frame(payload):
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return payload + struct.pack('<H', crc)


def position_for_width(width):
    if not math.isfinite(width) or not 0 <= width <= 85:
        raise ValueError('width must be 0..85 mm')
    # Nominal linear mapping, not a calibrated finger-gap measurement.
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
        return self.flags >> 6


class RobotiqRTU:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, slave_id=9,
                 serial_timeout=0.5, action_timeout=10.0, speed=64,
                 serial_factory=None):
        if not port or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError('Invalid serial port or baudrate')
        if not isinstance(slave_id, int) or not 1 <= slave_id <= 247:
            raise ValueError('slave_id must be 1..247')
        if not isinstance(speed, int) or not 0 <= speed <= 255:
            raise ValueError('speed must be a raw value in 0..255')
        if not all(math.isfinite(x) and x > 0 for x in (serial_timeout, action_timeout)):
            raise ValueError('Serial and action timeouts must be finite and positive')
        self.port, self.baudrate, self.slave_id = port, baudrate, slave_id
        self.serial_timeout, self.action_timeout = serial_timeout, action_timeout
        self.speed, self.serial_factory = speed, serial_factory
        self.serial = None

    def __enter__(self):
        factory = self.serial_factory
        if factory is None:
            import serial
            factory = serial.Serial
        options = dict(port=self.port, baudrate=self.baudrate, bytesize=8,
                       parity='N', stopbits=1, timeout=self.serial_timeout,
                       write_timeout=self.serial_timeout)
        if os.name == 'posix':
            options['exclusive'] = True
        self.serial = factory(**options)
        return self

    def __exit__(self, *args):
        self.serial.close()
        self.serial = None

    def _read_exact(self, size):
        data = bytearray()
        deadline = time.monotonic() + self.serial_timeout
        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Incomplete Modbus reply')
            self.serial.timeout = remaining
            chunk = self.serial.read(size - len(data))
            if not chunk:
                raise TimeoutError('No complete Modbus reply')
            data.extend(chunk)
        return bytes(data)

    def _exchange(self, function, body):
        # Robotiq specifies at least 5 ms between commands.
        time.sleep(0.005)
        self.serial.reset_input_buffer()
        request = frame(bytes([self.slave_id, function]) + body)
        if self.serial.write(request) != len(request):
            raise IOError('Incomplete Modbus request write')
        self.serial.flush()
        header = self._read_exact(3)
        if header[1] & 0x80:
            response = header + self._read_exact(2)
        elif function == 4:
            if header[2] != 6:
                raise IOError('Unexpected Modbus status length')
            response = header + self._read_exact(8)
        else:
            response = header + self._read_exact(5)
        if frame(response[:-2]) != response:
            raise IOError('Modbus CRC mismatch')
        if response[0] != self.slave_id:
            raise IOError('Unexpected Modbus slave')
        if response[1] == function | 0x80:
            raise IOError('Modbus exception {}'.format(response[2]))
        if response[1] != function:
            raise IOError('Unexpected Modbus function')
        return response

    def read_status(self):
        response = self._exchange(4, struct.pack('>HH', 0x07D0, 3))
        data = response[3:9]
        return Status(data[0], data[2] & 0x0F, data[3], data[4], data[5])

    def _command(self, action, position=0, force=0):
        data = bytes([action, 0, 0, position, self.speed, force])
        body = struct.pack('>HHB', 0x03E8, 3, 6) + data
        response = self._exchange(16, body)
        if response[2:6] != struct.pack('>HH', 0x03E8, 3):
            raise IOError('Modbus write acknowledgement differs from request')

    def _wait(self, predicate, activation=False):
        deadline = time.monotonic() + self.action_timeout
        while time.monotonic() < deadline:
            status = self.read_status()
            # 0x05/0x07 are transient before activation is complete.
            if status.fault and not (activation and status.fault in (5, 7)):
                raise IOError('Gripper fault 0x{:02x}'.format(status.fault))
            if predicate(status) and (not status.fault or activation):
                return status
            time.sleep(0.05)
        raise TimeoutError('Gripper action did not finish before timeout')

    def activate(self):
        """Explicit reset/activation sequence; may move fingers during calibration."""
        self._command(0)
        self._wait(lambda status: not status.flags & 1, activation=True)
        self._command(1)
        return self._wait(lambda status: status.ready and not status.fault, activation=True)

    def move(self, width, force):
        position = position_for_width(width)
        if not isinstance(force, int) or not 0 <= force <= 255:
            raise ValueError('force must be a raw Robotiq value in 0..255, not newtons')
        initial = self.read_status()
        if initial.fault or not initial.ready:
            raise IOError('Gripper is not ready; explicitly activate with reset service first')
        self._command(9, position, force)
        status = self._wait(lambda s: s.ready and bool(s.flags & 8)
                            and s.requested == position and s.object_state != 0)
        if status.object_state == 1:
            raise IOError('Gripper stopped on contact while opening')
        return status


def main():
    """Read status only: no activation, motion, or output-register writes."""
    import argparse
    parser = argparse.ArgumentParser(description='Read Robotiq status without actuating')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--slave-id', type=int, default=9)
    args = parser.parse_args()
    with RobotiqRTU(port=args.port, baudrate=args.baudrate, slave_id=args.slave_id) as gripper:
        print(gripper.read_status())
