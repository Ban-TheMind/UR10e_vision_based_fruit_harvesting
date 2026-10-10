#!/usr/bin/env python3
"""Compare observed fixed targets with independent UR base measurements."""

import argparse
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "camera"))
from camera.geometry import camera_point_to_base, deproject_pixel  # noqa: E402


def check_record(path):
    record = json.loads(Path(path).read_text(encoding="utf-8"))
    info = SimpleNamespace(**record["camera_info"])
    tolerance = float(record["max_error_m"])
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("max_error_m must be a positive finite number")
    observations = record["observations"]
    if not observations:
        raise ValueError("At least one independently measured target is required")
    failed = False
    for index, item in enumerate(observations, start=1):
        pixel = item["pixel"]
        depth_m = float(item["depth_m"])
        measured = item["measured_base_m"]
        if len(pixel) != 2 or len(measured) != 3 or any(
                value is None or not math.isfinite(float(value)) for value in measured):
            raise ValueError("Each target needs a pixel and independent base XYZ")
        optical = deproject_pixel(pixel[0], pixel[1], depth_m, info)
        if optical is None:
            raise ValueError("Target {} has invalid pixel or depth".format(index))
        predicted = camera_point_to_base(*optical)
        delta = tuple(predicted[i] - float(measured[i]) for i in range(3))
        error = math.sqrt(sum(value * value for value in delta))
        label = item.get("name", "target_{}".format(index))
        print("{}: predicted base={} m, measured={} m, error={:.4f} m".format(
            label, tuple(round(value, 4) for value in predicted),
            tuple(measured), error))
        failed |= error > tolerance
    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record", help="JSON record using tools/static_target_template.json")
    args = parser.parse_args()
    try:
        status = check_record(args.record)
    except (KeyError, TypeError, ValueError, OSError) as exc:
        parser.exit(2, "Invalid target record: {}\n".format(exc))
    print("FAIL: at least one target exceeds the tolerance" if status else
          "PASS: all measured targets are within the tolerance")
    return status


if __name__ == "__main__":
    sys.exit(main())
