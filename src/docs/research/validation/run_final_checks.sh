#!/usr/bin/env bash
# Offline only: no ROS launch, DDS, vehicle sockets or UDP.
set -euo pipefail
rda_source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
rda_test_python="${RDA_TEST_PYTHON:-python3}"
rda_test_cxx="${RDA_TEST_CXX:-c++}"
rda_check_temp="$(mktemp -d "${TMPDIR:-/tmp}/rda-final-check.XXXXXX")"
trap 'rm -rf "$rda_check_temp"' EXIT
cd "$rda_source_root"
"$rda_test_python" -m pytest -q tests docs/validation/test_perception_safety.py
"$rda_test_python" docs/validation/check_ros_adapters.py
"$rda_test_python" docs/research/validation/verify_contracts.py
"$rda_test_python" docs/research/validation/verify_patch.py
"$rda_test_cxx" -std=c++17 -Wall -Wextra -Wpedantic tests/test_mode_switch_guard.cpp -o "$rda_check_temp/original"
"$rda_check_temp/original"
"$rda_test_cxx" -std=c++17 -Wall -Wextra -Wpedantic tests/test_research_mode_switch.cpp -o "$rda_check_temp/extended"
"$rda_check_temp/extended"
