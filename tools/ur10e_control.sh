#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
source scripts/ros_env.sh
session_timeout="$(python3 tools/ur10e_control.py --print-session-timeout "$@")"
exec timeout --signal=INT --kill-after=20 "$session_timeout" python3 -u tools/ur10e_control.py "$@"
