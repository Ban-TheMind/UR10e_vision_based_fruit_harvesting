#!/usr/bin/env python3
"""Check the YOLO transport protocol without model weights or ROS."""

import json
from pathlib import Path
import socket
import struct
import sys
import threading
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "camera"))
from camera.inference_client import InferenceClient  # noqa: E402


class RunningProcess:
    def poll(self):
        return None


def recv_exact(sock, length):
    data = bytearray()
    while len(data) < length:
        part = sock.recv(length - len(data))
        if not part:
            raise EOFError("socket closed")
        data.extend(part)
    return bytes(data)


class InferenceProtocolTest(unittest.TestCase):
    def test_frame_and_detection_round_trip(self):
        client_socket, worker_socket = socket.socketpair()
        client_socket.settimeout(2)
        worker_socket.settimeout(2)
        client = InferenceClient.__new__(InferenceClient)
        client._lock = threading.Lock()
        client._socket = client_socket
        client._process = RunningProcess()
        frame = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        observations = []

        def fake_worker():
            try:
                height, width, length = struct.unpack("!III", recv_exact(worker_socket, 12))
                image = recv_exact(worker_socket, length)
                observations.append((height, width, image))
                payload = json.dumps({"detections": [
                    {"box": [0, 0, 2, 1], "class_id": 47, "confidence": 0.9}
                ]}).encode("utf-8")
                worker_socket.sendall(struct.pack("!I", len(payload)) + payload)
            finally:
                worker_socket.close()

        thread = threading.Thread(target=fake_worker)
        thread.start()
        try:
            result = client.predict(frame)
            self.assertEqual(observations, [(2, 3, frame.tobytes())])
            self.assertEqual(result[0]["class_id"], 47)
            self.assertEqual(result[0]["box"], [0, 0, 2, 1])
        finally:
            client_socket.close()
            thread.join(timeout=3)
        self.assertFalse(thread.is_alive())

    def test_non_bgr_frame_is_rejected(self):
        client = InferenceClient.__new__(InferenceClient)
        client._process = RunningProcess()
        with self.assertRaises(ValueError):
            client.predict(np.zeros((2, 2, 1), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
