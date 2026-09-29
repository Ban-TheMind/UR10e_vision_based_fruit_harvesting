#!/usr/bin/env python3
"""Check the YOLO subprocess without ROS, a camera, or robot motion."""

import os
import sys

import numpy as np


REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "camera"))

from camera.inference_client import InferenceClient  # noqa: E402


def main():
    python = os.environ.get(
        "UR10E_INFERENCE_PYTHON", "/home/ubuntu/miniconda3/envs/VPP/bin/python")
    model = os.path.join(REPO, "src", "camera", "models", "best.pt")
    worker = InferenceClient(python, model)
    try:
        detections = worker.predict(np.zeros((480, 640, 3), dtype=np.uint8))
        assert isinstance(detections, list)
        print("YOLO worker OK; detections on blank image:", len(detections))
    finally:
        worker.close()


if __name__ == "__main__":
    main()
