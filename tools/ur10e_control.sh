#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
source scripts/ros_env.sh
exec timeout --signal=INT --kill-after=20 720 python3 -u tools/ur10e_control.py "$@"
