#!/usr/bin/env python3
"""Repeatable checks that never connect to the robot, camera, or ROS graph."""

import argparse
import ast
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def run(label, command):
    print("CHECK", label, flush=True)
    subprocess.run(command, cwd=str(ROOT), check=True)


def check_source_files():
    python_files = list((ROOT / "src").rglob("*.py")) + list((ROOT / "tools").glob("*.py"))
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path),
                  feature_version=(3, 8))
    packages = list((ROOT / "src").rglob("package.xml"))
    names = set()
    for path in packages:
        name = ET.parse(str(path)).getroot().findtext("name")
        if not name or name in names:
            raise ValueError("Missing or duplicate ROS package name in " + str(path))
        names.add(name)
    print("PASS syntax:", len(python_files), "Python files,", len(packages), "ROS packages")


def check_console_scripts():
    count = 0
    for setup_file in (ROOT / "src").rglob("setup.py"):
        tree = ast.parse(setup_file.read_text(encoding="utf-8"))
        setup_call = next((node for node in ast.walk(tree)
                           if isinstance(node, ast.Call)
                           and isinstance(node.func, ast.Name)
                           and node.func.id == "setup"), None)
        if setup_call is None:
            continue
        entries = next((ast.literal_eval(keyword.value)
                        for keyword in setup_call.keywords
                        if keyword.arg == "entry_points"), {})
        for entry in entries.get("console_scripts", []):
            target = entry.split("=", 1)[1].strip()
            module_name, function_name = target.split(":", 1)
            module_file = setup_file.parent.joinpath(*module_name.split(".")).with_suffix(".py")
            if not module_file.is_file():
                raise ValueError("Missing console script module: " + target)
            module_tree = ast.parse(module_file.read_text(encoding="utf-8"))
            if not any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                       and node.name == function_name for node in module_tree.body):
                raise ValueError("Missing console script function: " + target)
            count += 1
    print("PASS", count, "console script targets")


def check_configuration():
    launch = (ROOT / "src/end_effector_description/launch/display.launch.py").read_text()
    xacro = ET.parse(str(ROOT / "src/end_effector_description/urdf/end_effector_withDriverSupport.xacro"))
    links = {element.get("name") for element in xacro.iter("link")}
    required = ("192.168.11.60", "_406122071837", "640,480,15")
    if not all(value in launch for value in required):
        raise ValueError("Robot IP, camera serial or supported profiles differ from the site configuration")
    if "simple_ee_link" not in links:
        raise ValueError("End effector link is missing from Xacro")
    controller_config = ROOT / "src/end_effector_description/config/ur_controllers.yaml"
    cmake = (ROOT / "src/end_effector_description/CMakeLists.txt").read_text()
    control_launch = (ROOT / "src/end_effector_description/launch/foxy_ur_control.launch.py").read_text()
    if (not controller_config.exists() or "config" not in cmake or
            'default_value="end_effector_description"' not in control_launch):
        raise ValueError("Foxy controller configuration is missing or not installed")
    for package in ("ur10e_moveit_config", "ur10e_moveit_config_official"):
        if not (ROOT / "src" / package / "COLCON_IGNORE").exists():
            raise ValueError("Old Humble MoveIt package is still included: " + package)
    print("PASS launch configuration and Foxy package layout")


def check_model(require_model):
    model = ROOT / "src/camera/models/best.pt"
    head = model.open("rb").read(32)
    pointer = head.startswith(b"version https://git-lfs")
    if pointer:
        message = "best.pt is a Git LFS pointer; fetch weights before inference tests"
        if require_model:
            raise ValueError(message)
        print("NOTE", message)
    else:
        print("PASS model file is present (inference still requires the target environment)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-model", action="store_true",
                        help="fail if best.pt is still a Git LFS pointer")
    args = parser.parse_args()
    check_source_files()
    check_console_scripts()
    check_configuration()
    run("Foxy hardcoded headless compatibility", [sys.executable, "tools/test_foxy_control_config.py"])
    check_model(args.require_model)
    run("camera geometry and depth units", [sys.executable, "tools/test_offline_geometry.py"])
    run("YOLO transport protocol without weights", [
        sys.executable, "tools/test_inference_protocol.py"])
    run("gripper defaults block device commands",
        [sys.executable, "src/gripper/test/test_gripper_safety.py"])
    compiler = shutil.which("g++")
    if compiler:
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / ("policy.exe" if sys.platform == "win32" else "policy"))
            run("compile movement request policy", [
                compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                "-Isrc/moveit_path_planner/src",
                "src/moveit_path_planner/test/test_movement_request_policy.cpp",
                "-o", binary])
            run("movement request policy", [binary])
    else:
        print("NOTE g++ unavailable; C++ policy check skipped")
    print("PASS offline preflight; no hardware was contacted")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print("FAIL", exc, file=sys.stderr)
        sys.exit(1)
