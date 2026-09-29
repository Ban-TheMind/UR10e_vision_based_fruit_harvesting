#!/usr/bin/env bash
# Read-only environment and URDF checks. --build only writes colcon products here.
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo 'FAIL: /opt/ros/foxy/setup.bash is missing' >&2
  exit 1
fi
set +u
source /opt/ros/foxy/setup.bash
set -u
venv="${UR10E_FOXY_VENV:-/home/ubuntu/Desktop/robot_learning/.venvs/ur10e-foxy}"
if [[ -f "$venv/bin/activate" ]]; then
  set +u
  source "$venv/bin/activate"
  set -u
  echo "Python environment: $venv"
fi
venv="${UR10E_FOXY_VENV:-/home/ubuntu/Desktop/robot_learning/.venvs/ur10e-foxy}"
if [[ -f "$venv/bin/activate" ]]; then
  source "$venv/bin/activate"
  echo "Python environment: $venv"
fi
if [[ "${ROS_DISTRO:-}" != foxy ]]; then
  echo "FAIL: expected Foxy, got ${ROS_DISTRO:-unset}" >&2
  exit 1
fi
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  echo "OS: ${PRETTY_NAME:-unknown}"
  if [[ "${VERSION_ID:-}" != 20.04 ]]; then
    echo "FAIL: this branch targets Ubuntu 20.04, got ${VERSION_ID:-unknown}" >&2
    exit 1
  fi
fi

missing=0
for package in ur_description ur_robot_driver ur_controllers ur_moveit_config \
               realsense2_camera moveit_ros_move_group controller_manager xacro; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "FOUND: $package"
  else
    echo "MISSING: $package" >&2
    missing=1
  fi
done
if (( missing )); then
  echo 'Install only the missing Foxy packages after reviewing apt-get -s.' >&2
  exit 1
fi

realsense_launch="$(ros2 pkg prefix realsense2_camera)/share/realsense2_camera/launch/rs_launch.py"
python3 - "$realsense_launch" <<'PY'
from pathlib import Path
import sys
launch_file = Path(sys.argv[1])
if not launch_file.is_file():
    raise SystemExit('FAIL: RealSense rs_launch.py is missing')
source = launch_file.read_text()
required = ('serial_no', 'rgb_camera.profile', 'depth_module.profile',
            'align_depth.enable', 'enable_sync')
missing = [name for name in required if name not in source]
if missing:
    raise SystemExit('FAIL: RealSense wrapper launch arguments differ: ' + ', '.join(missing))
print('PASS RealSense Foxy launch arguments')
PY

cd "$repo_dir"
python3 tools/offline_preflight.py

if [[ "${1:-}" == --build ]]; then
  colcon build --symlink-install
fi
if [[ -f install/setup.bash ]]; then
  set +u
  source install/setup.bash
  set -u
fi
if ! ros2 pkg prefix end_effector_description >/dev/null 2>&1; then
  echo 'NOTE: build this workspace to enable the Xacro expansion check.'
  exit 0
fi

ur_share="$(ros2 pkg prefix ur_description)/share/ur_description"
driver_share="$(ros2 pkg prefix ur_robot_driver)/share/ur_robot_driver"
ee_share="$(ros2 pkg prefix end_effector_description)/share/end_effector_description"
urdf_file="$(mktemp --suffix=.urdf)"
trap 'rm -f "$urdf_file"' EXIT
xacro "$ee_share/urdf/end_effector_withDriverSupport.xacro" \
  ur_type:=ur10e name:=ur robot_ip:=192.168.11.60 \
  joint_limit_params:="$ur_share/config/ur10e/joint_limits.yaml" \
  kinematics_params:="$ee_share/etc/robot_calibration.yaml" \
  physical_params:="$ur_share/config/ur10e/physical_parameters.yaml" \
  visual_params:="$ur_share/config/ur10e/visual_parameters.yaml" \
  script_filename:="$driver_share/resources/ros_control.urscript" \
  input_recipe_filename:="$driver_share/resources/rtde_input_recipe.txt" \
  output_recipe_filename:="$driver_share/resources/rtde_output_recipe.txt" \
  safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 \
  prefix:='' use_fake_hardware:=true fake_sensor_commands:=false > "$urdf_file"
python3 - "$urdf_file" <<'PY'
import sys
import xml.etree.ElementTree as ET
import math
root = ET.parse(sys.argv[1]).getroot()
links = {element.attrib['name'] for element in root.findall('link')}
required = {'base_link', 'base', 'tool0', 'simple_ee_link'}
if not required <= links or root.find('ros2_control') is None:
    raise SystemExit('FAIL: expanded URDF is missing UR links, tool, or ros2_control')
base_joint = next((joint for joint in root.findall('joint')
                   if joint.find('parent') is not None
                   and joint.find('child') is not None
                   and joint.find('parent').get('link') == 'base_link'
                   and joint.find('child').get('link') == 'base'), None)
if base_joint is None:
    raise SystemExit('FAIL: UR base_link-to-base joint is missing')
rpy = [float(value) for value in base_joint.find('origin').get('rpy').split()]
if abs(abs(rpy[2]) - math.pi) > 0.01:
    raise SystemExit('FAIL: planning-frame rotation differs from the camera conversion')
print('PASS Foxy UR10e Xacro:', len(links), 'links')
PY
echo 'PASS site preflight; robot controller and camera were not started'
