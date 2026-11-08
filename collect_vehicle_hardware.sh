#!/usr/bin/env bash
# Run on the Ubuntu vehicle. Queries only; no installs, power changes or car commands.
set -u

usage() {
  cat <<'HELP'
Usage: bash tools/collect_vehicle_hardware.sh [output_parent] [--ros] [--torch-probe]

Creates a unique vehicle_hardware_* directory and, if tar is available, a .tar.gz.
Default output parent: current directory.
--ros          Inspect an already-running /rslidar_points topic, with bounded queries.
               Also probes the ring field distribution (ring count, not cloud height).
--torch-probe  Import installed PyTorch and query CUDA availability; no inference.
Neither option starts the sensor, localization, planning or vehicle control stack.
HELP
}

vehicle_output_parent='.'
vehicle_parent_set=false
vehicle_ros_probe=false
vehicle_torch_probe=false
for vehicle_arg in "$@"; do
  case "$vehicle_arg" in
    --help|-h) usage; exit 0 ;;
    --ros) vehicle_ros_probe=true ;;
    --torch-probe) vehicle_torch_probe=true ;;
    --*) printf 'Unknown option: %s\n' "$vehicle_arg" >&2; exit 2 ;;
    *)
      if "$vehicle_parent_set"; then
        printf 'Only one output parent is accepted.\n' >&2; exit 2
      fi
      vehicle_output_parent="$vehicle_arg"
      vehicle_parent_set=true
      ;;
  esac
done

vehicle_report_dir="$vehicle_output_parent/vehicle_hardware_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$vehicle_report_dir" || exit 1
have() { command -v "$1" >/dev/null 2>&1; }
capture() {
  local vehicle_file="$1"
  shift
  if have "$1" || [[ -x "$1" ]]; then
    "$@" >"$vehicle_report_dir/$vehicle_file.txt" 2>&1
    local vehicle_rc=$?
    if [[ $vehicle_rc -ne 0 ]]; then
      printf '\nquery_exit_code=%s (unavailable/error; no automatic repair)\n' "$vehicle_rc" >>"$vehicle_report_dir/$vehicle_file.txt"
    fi
  else
    printf 'unavailable: %s\n' "$1" >"$vehicle_report_dir/$vehicle_file.txt"
  fi
}
copy_readable() {
  local vehicle_src="$1" vehicle_dest="$2"
  local vehicle_out="$vehicle_report_dir/$vehicle_dest.txt"
  if [[ ! -r "$vehicle_src" ]]; then
    printf 'unavailable: %s\n' "$vehicle_src" >"$vehicle_out"
    return 0
  fi
  # Read the source with cat and drop the bytes straight into a regular file.
  # No command substitution is used, so bash has no chance to warn about NUL
  # bytes (device-tree attributes end with one).
  if cat -- "$vehicle_src" >"$vehicle_out" 2>/dev/null; then
    # Strip NULs by reading the file we just wrote. read() on a regular file
    # never returns EINVAL, so tr is safe here, unlike on sysfs attributes.
    if tr -d '\000' <"$vehicle_out" >"$vehicle_out.clean" 2>/dev/null; then
      mv -f "$vehicle_out.clean" "$vehicle_out"
    else
      rm -f "$vehicle_out.clean"
    fi
  else
    printf 'unreadable: %s (kernel returned an error; sysfs link-down attributes do this)\n' \
      "$vehicle_src" >"$vehicle_out"
  fi
}

capture 00_date date -Iseconds

# --- added: flag an unset clock (Jetson without an RTC battery boots near 1970) ---
vehicle_year="$(date +%Y)"
if [[ "$vehicle_year" -lt 2020 ]]; then
  printf 'warning: system clock looks unset (%s).\n' "$(date -Iseconds)" >>"$vehicle_report_dir/00_date.txt"
  printf '         fix: sudo timedatectl set-ntp true (with network), or sudo date -s "YYYY-MM-DD HH:MM:SS".\n' >>"$vehicle_report_dir/00_date.txt"
  printf '         ROS header stamps, rosbag time, log rotation and MQTT timestamps are all wrong until fixed.\n' >>"$vehicle_report_dir/00_date.txt"
fi
capture 01_arch uname -m
capture 02_kernel uname -r
copy_readable /etc/os-release 03_ubuntu
capture 04_cpu lscpu
capture 05_cpu_available nproc
capture 06_memory free -b
capture 07_root_disk df -hT /
capture 08_storage lsblk -b -o NAME,TYPE,SIZE,MODEL,FSTYPE,MOUNTPOINT
capture 09_python python3 --version
printf 'ROS_DISTRO=%s\nRMW_IMPLEMENTATION=%s\n' "${ROS_DISTRO-unknown}" "${RMW_IMPLEMENTATION-unknown}" >"$vehicle_report_dir/10_ros_environment.txt"
capture 11_ros_installed ls -1 /opt/ros
capture 12_gpu nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
capture 13_gpu_compute_capability nvidia-smi --query-gpu=compute_cap --format=csv,noheader

# --- added: Jetson-native GPU/memory query (nvidia-smi is usually absent on Jetson) ---
if have jtop; then
  capture 12b_jtop_json timeout 8s jtop --json
elif have tegrastats; then
  capture 12c_tegrastats timeout 3s tegrastats --interval 1000
else
  printf 'unavailable: neither jtop (jetson-stats) nor tegrastats present\n' >"$vehicle_report_dir/12b_jtop_json.txt"
fi

if have nvcc; then
  capture 14_cuda_toolkit nvcc --version
else
  capture 14_cuda_toolkit /usr/local/cuda/bin/nvcc --version
fi
copy_readable /usr/local/cuda/version.json 15_cuda_version
copy_readable /proc/device-tree/model 16_board_model

# --- added: SoC family string (tegra234=Orin, tegra194=Xavier, tegra210=Nano) ---
copy_readable /proc/device-tree/compatible 16b_soc_compatible

copy_readable /etc/nv_tegra_release 17_jetson_l4t
capture 18_jetson_power_query nvpmodel -q

# --- added: nvpmodel needs root and usually lives in /usr/sbin; record the reason ---
if ! have nvpmodel; then
  printf 'unavailable: nvpmodel not in PATH (usually /usr/sbin, requires root)\n' >>"$vehicle_report_dir/18_jetson_power_query.txt"
fi

capture 19_debian_runtime_packages dpkg-query -W '-f=${binary:Package}\t${Version}\n' \
  nvidia-l4t-core nvidia-jetpack 'libnvinfer*' 'python3-libnvinfer*' 'libcudnn*' \
  'cuda-toolkit-*' 'ros-*-rclpy' 'ros-*-ros2topic' 'ros-*-ros-base'
capture 20_time_sync timedatectl show -p NTPSynchronized -p NTP -p Timezone

# --- added: NTP property is absent on some systemd builds; keep a separate safe query ---
capture 20b_time_sync_ntp_synced timedatectl show -p NTPSynchronized -p Timezone

# --- added: board vendor / hostname / packages ---
capture 22_board_hostname hostnamectl
capture 23_vendor_packages dpkg-query -W '-f=${binary:Package}\t${Version}\n' 'miivii*' 'nvidia-*'

# --- added: USB devices, network topology, serial sensors ---
capture 24_usb_devices lsusb
capture 25_net_addr ip -br addr
capture 26_net_neigh ip neigh
for vehicle_tty in /dev/ttyUSB*; do
  [ -e "$vehicle_tty" ] || continue
  printf 'present: %s\n' "$vehicle_tty" >"$vehicle_report_dir/serial_${vehicle_tty##*/}.txt"
  capture "serial_${vehicle_tty##*/}_udev" udevadm info -q property -n "$vehicle_tty"
done

if have python3; then
  python3 - "$vehicle_torch_probe" >"$vehicle_report_dir/21_python_packages.json" 2>&1 <<'PY'
import importlib.metadata as md
import json, os, platform, sys
result = {'python_executable':sys.executable, 'python_version':sys.version,
          'machine':platform.machine(), 'packages':{},
          'inside_docker_marker':os.path.exists('/.dockerenv'),
          'package_metadata_is_not_cuda_operator_validation':True}
try:
    result['cpu_affinity_count']=len(os.sched_getaffinity(0))
except (AttributeError,OSError):
    result['cpu_affinity_count']=None
for key,path in [('cgroup_cpu_max','/sys/fs/cgroup/cpu.max'),('cgroup_memory_max','/sys/fs/cgroup/memory.max')]:
    try:
        with open(path) as stream: result[key]=stream.read().strip()
    except OSError:
        result[key]=None
names=('torch','torchvision','torchaudio','scipy','numpy','spconv','cumm','tensorrt','nvidia-cuda','rclpy','ros2cli','ros2topic')
for dist in md.distributions():
    name=dist.metadata.get('Name','')
    if any(name.lower()==n or name.lower().startswith(n+'-') for n in names):
        result['packages'][name]=dist.version
if sys.argv[1]=='true':
    try:
        import torch
        result['torch_probe']={'version':torch.__version__,'compiled_cuda':torch.version.cuda,
                               'cuda_available':torch.cuda.is_available(),'inference_run':False}
        if result['torch_probe']['cuda_available']:
            result['torch_probe']['devices']=[{'name':torch.cuda.get_device_name(i),
                'compute_capability':list(torch.cuda.get_device_capability(i))} for i in range(torch.cuda.device_count())]
    except Exception as exc:
        result['torch_probe']={'error':type(exc).__name__+': '+str(exc)}
print(json.dumps(result,ensure_ascii=False,indent=2))
PY
fi

if [[ -d /sys/class/net ]]; then
  for vehicle_iface_path in /sys/class/net/*; do
    vehicle_iface="${vehicle_iface_path##*/}"
    [[ "$vehicle_iface" == lo ]] && continue
    for vehicle_metric in operstate speed duplex mtu statistics/rx_dropped statistics/rx_errors statistics/tx_dropped statistics/tx_errors; do
      vehicle_metric_name="${vehicle_metric//\//_}"
      copy_readable "$vehicle_iface_path/$vehicle_metric" "net_${vehicle_iface}_${vehicle_metric_name}"
    done
  done
fi

# --- added: read one frame and report ring distribution instead of cloud height ---
cat >"$vehicle_report_dir/probe_rings.py" <<'PY'
import sys
try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
except Exception as exc:
    print('import_error: %s: %s' % (type(exc).__name__, exc))
    sys.exit(1)

topic = sys.argv[1] if len(sys.argv) > 1 else '/rslidar_points'
field = sys.argv[2] if len(sys.argv) > 2 else 'ring'
rclpy.init()
node = rclpy.create_node('vehicle_ring_probe')
state = {'done': False}

def cb(msg):
    if state['done']:
        return
    state['done'] = True
    print('topic=%s field=%s height=%d width=%d point_step=%d row_step=%d'
          % (topic, field, msg.height, msg.width, msg.point_step, msg.row_step))
    try:
        vals = [int(row[0]) for row in pc2.read_points(msg, field_names=(field,), skip_nans=True)]
        uniq = sorted(set(vals))
        print('field_min=%s field_max=%s unique_values=%d'
              % (uniq[0] if uniq else 'n/a', uniq[-1] if uniq else 'n/a', len(uniq)))
        print('unique_list_head=%s' % (uniq[:64],))
    except Exception as exc:
        print('field_read_error: %s: %s' % (type(exc).__name__, exc))
        print('available_fields=%s' % [f.name for f in msg.fields])
    rclpy.shutdown()

node.create_subscription(PointCloud2, topic, cb, qos_profile_sensor_data)
rclpy.spin(node)
PY

if "$vehicle_ros_probe"; then
  if have ros2 && have timeout; then
    capture ros_01_pointcloud_info timeout 5s ros2 topic info /rslidar_points -v
    capture ros_02_pointcloud_rate timeout --signal=INT 12s ros2 topic hz /rslidar_points --window 100
    if ros2 topic echo --help 2>&1 | grep -q -- '--once'; then
      capture ros_03_pointcloud_fields timeout 6s ros2 topic echo /rslidar_points --once --field fields --qos-reliability best_effort
      if ros2 topic echo --help 2>&1 | grep -q -- '--no-arr'; then
        capture ros_04_pointcloud_metadata timeout 6s ros2 topic echo /rslidar_points --once --no-arr --qos-reliability best_effort
      fi
    else
      printf 'This ROS CLI lacks --once; metadata echo skipped. Do not echo cloud.data.\n' >"$vehicle_report_dir/ros_03_pointcloud_fields.txt"
    fi

    # --- added: ring distribution on raw and converted clouds ---
    if have python3; then
      capture ros_05_ring_probe timeout 15s python3 -u "$vehicle_report_dir/probe_rings.py" /rslidar_points ring
      capture ros_06_ring_probe_velodyne timeout 15s python3 -u "$vehicle_report_dir/probe_rings.py" /velodyne_points ring
    else
      printf 'python3 unavailable; ring probe skipped.\n' >"$vehicle_report_dir/ros_05_ring_probe.txt"
    fi
  else
    printf 'ros2/timeout unavailable. Source the correct existing ROS environment before --ros.\n' >"$vehicle_report_dir/ros_unavailable.txt"
  fi
fi

cat >"$vehicle_report_dir/README.txt" <<'INFO'
Queries only: no sudo, installation, benchmark, model inference or car commands.
Missing commands/permission errors are recorded, not automatically repaired.
nvidia-smi CUDA support, nvcc toolkit and actual CUDA operator compatibility are different.
Jetson uses shared memory and JetPack/L4T; no nvidia-smi result does not mean no GPU.
ROS topic hz reports arrival rate; source progress/age need header/diagnostic analysis.
Manual information still needed: sensor/IMU model, mounting height/pitch, driver version,
time settings, power/cooling, obstacle classes/size/distance and end-to-end latency target.
INFO

# --- added: how to read the extra files (appended to the same README) ---
cat >>"$vehicle_report_dir/README.txt" <<'INFO'

Added v2 notes
--------------
16b_soc_compatible: tegra234 = Orin family, tegra194 = Xavier family, tegra210 = Nano.
                   Read it together with 16_board_model to pin the exact module.
12_gpu / 13_gpu_compute_capability: expected to fail or return N/A on Jetson (no dGPU driver).
12b_jtop_json / 12c_tegrastats: the real GPU model, clocks and unified memory usage.
Jetson has no separate VRAM; use 06_memory (free -b) and jtop MEM, not memory.total.
18_jetson_power_query: nvpmodel needs root and usually sits in /usr/sbin, so this entry is
                       normally "unavailable" by design (this script never elevates).
19/23 packages: a non-zero dpkg-query exit means "some patterns matched nothing",
                it does NOT mean those packages are absent.
20/20b time sync: 20 may fail entirely if this systemd build has no NTP property; 20b is the
                  safe subset (NTPSynchronized + Timezone).
ros_05/ros_06 ring probe: reports the ring field distribution of one frame.
    Height is NOT the ring count here: dense_points=true and ros_send_by_rows defaults to
    false, so /rslidar_points publishes width=1/height=<point count> and /velodyne_points
    publishes height=1/width=<point count>. Ring count must come from the ring field.
LiDAR exact model is NOT derivable from ROS alone: send_packet_ros is false, so there is no
    /rslidar_packets topic to read DIFOP from. Use the nameplate/SN, or temporarily enable
    send_packet_ros for a packet capture (that changes runtime behaviour: manual step only).
"unreadable: <path>" means the kernel rejected the read (EINVAL). Typical case is
    /sys/class/net/*/speed or duplex while the link is down. It is neither a permission
    problem nor a missing device, and it no longer prints tr errors to the console.
00_date showing 1970 means the clock/RTC is unset (common on a Jetson without an RTC
    battery). The warning and the fix command are appended to 00_date.txt.
INFO

printf 'Hardware report: %s\n' "$vehicle_report_dir"
if have tar; then
  tar -czf "$vehicle_report_dir.tar.gz" -C "$vehicle_output_parent" "${vehicle_report_dir##*/}"
  printf 'Archive: %s.tar.gz\n' "$vehicle_report_dir"
fi
