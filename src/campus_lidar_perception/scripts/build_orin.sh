#!/usr/bin/env bash
set -euo pipefail
task_package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != aarch64 ]]; then
  echo "This build targets the Linux ARM64 Jetson Orin; the current host is not the vehicle." >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
task_perception_venv="${CAMPUS_VENV:-${task_package_root}/.venv-orin}"
python3 -m venv --system-site-packages "${task_perception_venv}"
touch "${task_perception_venv}/COLCON_IGNORE"
source "${task_perception_venv}/bin/activate"
python -m pip install -r "${task_package_root}/requirements-orin.txt"
python -m pip install colcon-common-extensions
if [[ -n "${JETSON_TORCH_WHEEL:-}" ]]; then
  python -m pip install "${JETSON_TORCH_WHEEL}"
fi
python - <<'PY'
try:
    import torch
except ImportError as exc:
    raise SystemExit('CUDA Jetson PyTorch is missing. Install the NVIDIA wheel matching JetPack/CUDA, or set JETSON_TORCH_WHEEL. See README.md.') from exc
if not torch.cuda.is_available():
    raise SystemExit('PyTorch has no working CUDA backend; a CPU wheel cannot run HARP-NeXt on this vehicle.')
print('CUDA Torch:',torch.__version__,'GPU:',torch.cuda.get_device_name(0))
PY
cmake -S "${task_package_root}/native" -B "${task_package_root}/native/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=87 \
  -DCMAKE_INSTALL_PREFIX="${task_package_root}"
cmake --build "${task_package_root}/native/build" --parallel 6
cmake --install "${task_package_root}/native/build"
echo "Native backend built. Keep this venv active when running colcon and ROS2; see README.md."
