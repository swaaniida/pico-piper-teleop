#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_dir"

if [[ ! -f src/piper_sdk/setup.py || ! -f src/XRoboToolkit-PC-Service-Pybind/setup.py ]]; then
  echo "Initializing SDK submodules..."
  git submodule update --init --recursive
fi

python -m pip install -e src/piper_sdk
python -m pip install -e src/XRoboToolkit-PC-Service-Pybind
python -m pip install -e .

python -m single_piper_teleop.preflight --software-only
echo "Installation complete. Source ROS 2 and the Tracer workspace before integrated teleoperation."
