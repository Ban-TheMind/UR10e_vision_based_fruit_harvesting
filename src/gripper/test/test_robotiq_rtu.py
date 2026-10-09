"""Offline tests with exact RTU replies; never open a hardware port."""
import sys
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gripper.robotiq_rtu import RobotiqRTU, frame, position_for_width


def status(flags=0xF9, requested=255, position=255, fault=0):
    return frame(bytes([9, 4, 6, flags, 0, fault, requested, position, 0]))


ACK = bytes.fromhex('09 10 03 e8 00 03 01 30')


class SerialFixture:
    def __init__(self, replies):
        self.replies = list(replies)
        self.buffer = b''
        self.writes = []
        self.closed = False

    def reset_input_buffer(self):
        self.buffer = b''

    def write(self, data):
        self.writes.append(data)
        self.buffer = self.replies.pop(0)
        return len(data)

    def read(self, size):
        # Fragmented reads ensure transport does not assume a single read.
        value, self.buffer = self.buffer[:min(size, 2)], self.buffer[min(size, 2):]
        return value

    def flush(self):
        pass

    def close(self):
        self.closed = True


class RTUTests(unittest.TestCase):
    def driver(self, replies, **kwargs):
        fixture = SerialFixture(replies)
        factory = Mock(return_value=fixture)
        driver = RobotiqRTU(serial_factory=factory, **kwargs)
        return driver, fixture, factory

    def test_manual_crc_and_width_endpoints(self):
        self.assertEqual(frame(bytes.fromhex('09 10 03 e8 00 03 06 01 00 00 00 00 00')),
                         bytes.fromhex('09 10 03 e8 00 03 06 01 00 00 00 00 00 72 e1'))
        self.assertEqual(position_for_width(85), 0)
        self.assertEqual(position_for_width(0), 255)
        for value in (-1, 100, float('nan')):
            with self.assertRaises(ValueError):
                position_for_width(value)

    def test_read_only_serial_settings_and_close(self):
        driver, fixture, factory = self.driver([status()])
        with driver:
            self.assertTrue(driver.read_status().ready)
        options = factory.call_args.kwargs
        self.assertEqual((options['port'], options['baudrate'], options['bytesize'],
                          options['parity'], options['stopbits']),
                         ('/dev/ttyUSB0', 115200, 8, 'N', 1))
        self.assertEqual(fixture.writes, [frame(bytes.fromhex('09 04 07 d0 00 03'))])
        self.assertTrue(fixture.closed)

    def test_motion_waits_for_echo_and_contact(self):
        driver, fixture, _ = self.driver([status(), ACK, status(requested=0),
                                         status(flags=0x39), status(flags=0xB9, position=190)])
        with driver:
            result = driver.move(0, 40)
        self.assertEqual(result.object_state, 2)
        self.assertEqual(result.position, 190)
        self.assertEqual(fixture.writes[1], frame(bytes.fromhex('09 10 03 e8 00 03 06 09 00 00 ff 40 28')))
        self.assertEqual(len(fixture.writes), 5)

    def test_activation_sequence(self):
        driver, fixture, _ = self.driver([ACK, status(flags=0), ACK,
                                         status(flags=0x11), status(flags=0x31)])
        with driver:
            self.assertTrue(driver.activate().ready)
        self.assertEqual(fixture.writes[0][7], 0)
        self.assertEqual(fixture.writes[2][7], 1)

    def test_bad_crc_timeout_exception_wrong_slave_and_ack_fail(self):
        bad = status()[:-1] + bytes([status()[-1] ^ 1])
        for reply in (bad, b'', frame(bytes([9, 0x84, 2])),
                      frame(bytes([8, 4, 6, 0xF9, 0, 0, 255, 255, 0]))):
            driver, fixture, _ = self.driver([reply])
            with driver, self.assertRaises((IOError, TimeoutError)):
                driver.read_status()
            self.assertTrue(fixture.closed)
        driver, _, _ = self.driver([status(), frame(bytes.fromhex('09 10 03 e9 00 03'))])
        with driver, self.assertRaises(IOError):
            driver.move(0, 40)

    def test_unready_fault_and_opening_contact_fail(self):
        for replies, width in (([status(flags=0)], 0), ([status(fault=14)], 0),
                               ([status(), ACK, status(flags=0x79, requested=0)], 85)):
            driver, _, _ = self.driver(replies)
            with driver, self.assertRaises(IOError):
                driver.move(width, 40)

    def test_invalid_force_no_writes(self):
        driver, fixture, _ = self.driver([])
        with driver, self.assertRaises(ValueError):
            driver.move(85, 256)
        self.assertEqual(fixture.writes, [])

    def test_action_timeout(self):
        driver, _, _ = self.driver([status(), ACK, status(flags=0x39)], action_timeout=0.01)
        with patch('gripper.robotiq_rtu.time.sleep'), patch(
                'gripper.robotiq_rtu.time.monotonic', side_effect=[0, 0, 0, 0, 0, 0, 0, 0, 0, 1]):
            # Separate predicate wait avoids timing serial reads in this test.
            driver.read_status = Mock(return_value=__import__('gripper.robotiq_rtu', fromlist=['Status']).Status(0x39, 0, 255, 100, 0))
            with self.assertRaises(TimeoutError):
                driver._wait(lambda s: s.object_state != 0)


if __name__ == '__main__':
    unittest.main()
