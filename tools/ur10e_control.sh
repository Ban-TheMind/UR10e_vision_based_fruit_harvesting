#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/foxy/setup.bash
source "${UR10E_FOXY_VENV:-/home/ubuntu/Desktop/robot_learning/.venvs/ur10e-project-foxy}/bin/activate"
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=61
source install-project/local_setup.bash
exec timeout --signal=INT --kill-after=20 720 python -u tools/ur10e_control.py "$@"
