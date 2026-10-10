"""Private inference process; communicate only over the inherited socket."""

import json
import socket
import struct
import sys

import numpy as np
from ultralytics import YOLO


def recv_exact(sock, length):
    chunks = bytearray()
    while len(chunks) < length:
        chunk = sock.recv(length - len(chunks))
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


def main():
    sock = socket.socket(fileno=int(sys.argv[1]))
    model = YOLO(sys.argv[2])
    model.fuse()
    while True:
        header = recv_exact(sock, 12)
        if header is None:
            break
        height, width, length = struct.unpack("!III", header)
        if height == 0 or width == 0 or length != height * width * 3:
            break
        data = recv_exact(sock, length)
        if data is None:
            break
        frame = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        try:
            result = model(frame, verbose=False)[0].boxes
            payload = {"detections": [
                {"box": box.tolist(), "class_id": int(cls), "confidence": float(conf)}
                for box, cls, conf in zip(
                    result.xyxy.cpu().numpy(),
                    result.cls.cpu().numpy(),
                    result.conf.cpu().numpy())
            ]}
        except Exception as exc:
            payload = {"error": str(exc)}
        encoded = json.dumps(payload).encode("utf-8")
        sock.sendall(struct.pack("!I", len(encoded)) + encoded)


if __name__ == "__main__":
    main()
