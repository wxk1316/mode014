#!/usr/bin/env bash
# Offline validation only: no ROS nodes, DDS traffic or vehicle UDP.
set -euo pipefail
rda_workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
rda_python="${RDA_TEST_PYTHON:-python3}"
rda_compiler="${RDA_TEST_CXX:-c++}"
rda_temp_dir="$(mktemp -d "${TMPDIR:-/tmp}/rda-validation.XXXXXX")"
trap 'rm -rf "$rda_temp_dir"' EXIT
cd "$rda_workspace_root"

"$rda_python" -m pytest -q tests docs/validation/test_perception_safety.py
"$rda_python" docs/validation/check_ros_adapters.py > docs/validation/ros_adapter_result.json
"$rda_python" docs/validation/run_closed_loop.py
"$rda_python" docs/validation/run_spin_session.py
"$rda_compiler" -std=c++17 -Wall -Wextra -Wpedantic \
  tests/test_mode_switch_guard.cpp -o "$rda_temp_dir/mode_switch_guard"
"$rda_temp_dir/mode_switch_guard"
"$rda_python" - <<'PY'
import ast
from pathlib import Path
roots = ['car_launch/launch', 'rda_dwa/rda_dwa', 'rda_dwa_real',
         'localization/FAST_LIO/scripts', 'tests', 'docs/validation']
paths = sorted({path for root in roots for path in Path(root).rglob('*.py')})
for path in paths:
    ast.parse(path.read_text(), filename=str(path))
print(f'Parsed {len(paths)} Python/launch/validation sources; no syntax errors.')
PY
