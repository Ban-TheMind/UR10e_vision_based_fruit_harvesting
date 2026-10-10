#!/usr/bin/env bash
# Source from a wrapper; no device or ROS node is started here.
if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo 'Foxy environment missing: run project check here; build and launch on the Ubuntu 20.04 robot PC.' >&2
  return 1
fi
set +u
source /opt/ros/foxy/setup.bash
venv="${UR10E_FOXY_VENV:-/home/ubuntu/Desktop/robot_learning/.venvs/ur10e-project-foxy}"
if [[ -f "$venv/bin/activate" ]]; then source "$venv/bin/activate"; fi
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
if [[ "${HARVEST_LOAD_INSTALL:-1}" == 1 ]]; then
  if [[ ! -f "$repo_dir/install-project/local_setup.bash" ]]; then
    echo 'Workspace not built: run ./scripts/project build first.' >&2
    return 1
  fi
  source "$repo_dir/install-project/local_setup.bash"
fi
