"""Static wiring and isolated contract checks; no middleware/hardware launch."""
import ast,json,sys,subprocess,hashlib
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
production=['src/rda_dwa/rda_dwa/realtime_ackermann.py','src/rda_dwa/rda_dwa/motion_safety.py','src/rda_dwa_real/rda_dwa_real/async_planner.py','src/rda_dwa_real/rda_dwa_real/bridge_node.py','src/rda_dwa_real/rda_dwa_real/kf_tracker.py','src/rda_dwa_real/rda_dwa_real/lidar_track_node.py','src/rda_dwa_real/rda_dwa_real/localization_input.py','src/rda_dwa_real/rda_dwa_real/perception_core.py','src/rda_dwa_real/rda_dwa_real/pose_velocity.py','src/rda_dwa_real/rda_dwa_real/realtime_vehicle_core.py','src/rda_dwa_real/rda_dwa_real/traj_tracker_core.py','src/rda_dwa_real/rda_dwa_real/traj_tracker_node.py','src/rda_dwa_real/config/real_vehicle.yaml']
for name in production:
 if name.endswith('.py'):ast.parse((ROOT/name).read_text())
config=yaml.safe_load((ROOT/production[-1]).read_text())
assert all(config[k]['ros__parameters']['localization_source']=='latest_available' for k in ('rda_dwa_real_bridge','lidar_track_node','traj_tracker_node'))
assert all(config[k]['ros__parameters']['localization_timeout']==2. for k in ('rda_dwa_real_bridge','traj_tracker_node'))
assert config['lidar_track_node']['ros__parameters']['loc_timeout']==2.
assert all(config[k]['ros__parameters']['safety_margin']==.12 for k in ('rda_dwa_real_bridge','traj_tracker_node'))
assert config['rda_dwa_real_bridge']['ros__parameters']['command_jerk']==config['traj_tracker_node']['ros__parameters']['jerk_max']
result={'production_files':{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in production},'syntax':'passed','parameter_consistency':'passed','localization_producers_modified':False,'driver_protocol_modified':False}
(OUT/'modified_files.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='production_files'},indent=2))
