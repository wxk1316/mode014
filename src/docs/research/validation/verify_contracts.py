"""Read-only configuration and protected-localization acceptance check."""
from pathlib import Path
import hashlib,json,math,sys,yaml
ROOT=Path(__file__).resolve().parents[3]
def main():
 config=yaml.safe_load((ROOT/'rda_dwa_real/config/real_vehicle.yaml').read_text())
 b=config['rda_dwa_real_bridge']['ros__parameters'];t=config['traj_tracker_node']['ros__parameters'];l=config['lidar_track_node']['ros__parameters']
 checks={
  'centre_compensation_zero':all(p['body_to_base_link_x']==0 for p in (b,t,l)),
  'footprint_centre_zero':b['footprint_center_offset']==b['spin_pivot_offset']==l['vehicle_center_x']==0,
  'footprint_geometry':b['ack_length']==b['vehicle_length']==2*l['vehicle_half_length']==1.2 and b['ack_width']==b['vehicle_width']==2*l['vehicle_half_width']==.8,
  'wheelbase_shared':b['ack_wheelbase']==t['wheelbase']==.69,
  'rear_reference_shared':b['reference_center_to_rear']==t['reference_center_to_rear']==.345,
  'speed_limits_shared':b['v_max']==b['ack_v_max']==t['max_speed']==t['v_max'] and b['v_min']==b['ack_v_min']==t['v_min'],
  'accel_brake_shared':b['ack_a_max']==t['acc_lin_max'] and b['ack_dec_max']==t['dec_lin_max'],
  'steer_shared':b['ack_delta_max_deg']==t['max_steer_deg'] and b['ack_delta_dot_max_deg']==t['steer_rate_deg'],
  'physical_switch_minimum':min(b['spin_wait_time'],b['spin_recover_time'],t['spin_wait_time'],t['recover_delay'])>=3,
  'physical_stop_timeout_shared':b['spin_stop_timeout']==t['spin_stop_timeout'],
  'physical_stop_gates_enabled':t['spin_require_stopped_feedback'] and t['spin_require_yaw_stop_feedback'],
  'reaction_covers_watchdog':b['reaction_time']>=t['cmd_timeout']+.1,
  'planning_budget_within_period':0<b['planning_budget_s']<1/b['control_rate'] and b['planning_budget_s']<b['planning_timeout'],
  'tracking_correction_unenabled':t['tracking_correction_max_deg']==0.,
 }
 baseline=json.loads((ROOT/'docs/research/baseline_manifest.json').read_text())['sha256']
 protected={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha for name,sha in baseline.items() if name.startswith('localization/') or name=='car_launch/launch/run_fastlio.launch.py'}
 checks['protected_localization_byte_identical']=all(protected.values())
 result={'date':'2026-11-06','checks':checks,'protected_files_checked':len(protected),'failure_files':[p for p,ok in protected.items() if not ok],
  'centre_to_rear_status':'symmetric-wheelbase assumption, not independently measured',
  'vehicle_limit_status':'configured motion bounds, not a calibration certificate',
  'steering_measurement_available':False,'scope':'read-only YAML and SHA256; no ROS, DDS or vehicle'}
 print(json.dumps(result,indent=2));return not all(checks.values())
if __name__=='__main__':raise SystemExit(main())
