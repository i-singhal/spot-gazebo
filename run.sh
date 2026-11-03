#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# ROS vendor packages provide Gazebo Harmonic and its Python bindings here.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m spot_gazebo.launch "$@"
