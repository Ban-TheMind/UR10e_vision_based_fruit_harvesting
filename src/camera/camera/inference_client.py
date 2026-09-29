"""Run YOLO in its Python 3.10 environment, separate from Foxy Python 3.8."""

import json
import os
import socket
import struct
import subprocess
import threading


class InferenceClient:
    def __init__(self, python_executable, model_path, timeout=60):
        self._lock = threading.Lock()
        self._socket, child = socket.socketpair()
        self._socket.settimeout(timeout)
        worker = os.path.join(os.path.dirname(__file__), "yolo_worker.py")
        worker_env = os.environ.copy()
        worker_env.pop("PYTHONPATH", None)
        worker_env.pop("PYTHONHOME", None)
        worker_env.pop("VIRTUAL_ENV", None)
        # Foxy shared libraries and Python 3.8 modules must not shadow the
        # separate Python 3.10 inference environment.
        worker_env["LD_LIBRARY_PATH"] = ":".join(
            path for path in worker_env.get("LD_LIBRARY_PATH", "").split(":")
            if path and "/opt/ros/foxy" not in path and "/install/" not in path)
        try:
            self._process = subprocess.Popen(
                [python_executable, worker, str(child.fileno()), model_path],
                pass_fds=(child.fileno(),),
                stdin=subprocess.DEVNULL,
                env=worker_env,
            )
        except Exception:
            self._socket.close()
            child.close()
            raise
        child.close()

    def predict(self, frame):
        if self._process.poll() is not None:
            raise RuntimeError("YOLO worker exited before inference")
        height, width, channels = frame.shape
        if channels != 3:
            raise ValueError("YOLO worker expects BGR8 images")
        data = frame.tobytes()
        with self._lock:
            self._socket.sendall(struct.pack("!III", height, width, len(data)))
            self._socket.sendall(data)
            length = struct.unpack("!I", self._recv_exact(4))[0]
            if length > 16 * 1024 * 1024:
                raise RuntimeError("YOLO worker response is too large")
            result = json.loads(self._recv_exact(length).decode("utf-8"))
        if "error" in result:
            raise RuntimeError(result["error"])
        return result["detections"]

    def _recv_exact(self, length):
        chunks = bytearray()
        while len(chunks) < length:
            chunk = self._socket.recv(length - len(chunks))
            if not chunk:
                raise RuntimeError("YOLO worker disconnected")
            chunks.extend(chunk)
        return bytes(chunks)

    def close(self):
        self._socket.close()
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
