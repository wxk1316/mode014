"""Read-only vehicle diagnostics, schema 2.

Every subscribed small message is saved with source/arrival clocks and sequence.
20 Hz snapshots are asynchronous caches, never synchronized physical truth.
A bounded writer thread owns JSON files. Optional independent rosbag captures
raw pointcloud, IMU, TF and core messages. Configuration copies are distinguished
from actual read-only ROS parameter-service snapshots.
"""
from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from datetime import datetime
from .telemetry_recording import AsyncJsonlWriter,BagCapture,safe_data,source_header,parameter_value,code_provenance,SCHEMA_VERSION,BAG_TOPICS

# ====================== 纯函数区（无 ROS 依赖，可本地冒烟） ======================


def parse_kv_string(s: str) -> dict:
    """解析 'k=v k=v2' 形式的诊断串（/rda/status、/rda/ctrl_status）为 dict。

    数值型字段尽量转 float；失败保留原字符串。不抛异常。
    """
    out: dict = {}
    if not s:
        return out
    for tok in str(s).split():
        if '=' not in tok:
            continue
        k, v = tok.split('=', 1)
        try:
            out[k] = float(v)
        except (ValueError, TypeError):
            out[k] = v
    return out


def _finite(v):
    """NaN/Inf → None（JSON 不可序列化）。"""
    if v is None:
        return None
    try:
        return float(v) if math.isfinite(float(v)) else None
    except (TypeError, ValueError):
        return v


# ---------- 2026-11-06 新增纯函数（重构后取证需要，均可离线冒烟） ----------

def _yaw_from_quat(q) -> float:
    """四元数 → yaw（与 bridge_node/跟踪层同口径）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def body_frame(dx: float, dy: float, yaw: float) -> tuple:
    """map 系相对向量 (dx,dy) → 车体系 (前向 x, 左向 y)。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


def summarize_road_base(poses) -> dict:
    """把 /rda/traj（前方 2.5m 路基）摘要成可分析字段。纯函数。

    口径（2026-11-06 重构后）：
      每点 position=(x, y, v)，即 position.z = 该点参考速度 v(m/s)；
      header.stamp = now + 该点相对时刻（由规划侧按里程/速度换算）。
    返回：n_pts / arc_len / v0 / v_last / t_span / 首末点坐标 / yaw0 /
          k_est（首三点 Menger 曲率，应≈规划输出的 ω/v，可反查积分是否对）。
    """
    out = {
        'n_pts': len(poses), 'arc_len': 0.0, 'v0': None, 'v_last': None,
        't_span': None, 'x0': None, 'y0': None, 'yaw0': None,
        'x_last': None, 'y_last': None, 'k_est': None,
        'front_len': None,   # 首点到末点的直线距离（≠弧长说明是弯的）
    }
    if not poses:
        return out
    p0 = poses[0].pose
    out['x0'] = _finite(p0.position.x)
    out['y0'] = _finite(p0.position.y)
    out['v0'] = _finite(p0.position.z)
    out['yaw0'] = _finite(_yaw_from_quat(p0.orientation))
    plast = poses[-1].pose
    out['x_last'] = _finite(plast.position.x)
    out['y_last'] = _finite(plast.position.y)
    out['v_last'] = _finite(plast.position.z)
    arc = 0.0
    for i in range(1, len(poses)):
        a = poses[i - 1].pose.position
        b = poses[i].pose.position
        arc += math.hypot(b.x - a.x, b.y - a.y)
    out['arc_len'] = arc
    if out['x0'] is not None and out['x_last'] is not None:
        out['front_len'] = math.hypot(out['x_last'] - out['x0'], out['y_last'] - out['y0'])
    # 时间轴（末点相对时刻 − 首点相对时刻，单位 s）
    try:
        t0 = poses[0].header.stamp
        t1 = poses[-1].header.stamp
        dt = (t1.sec - t0.sec) + (t1.nanosec - t0.nanosec) * 1e-9
        out['t_span'] = _finite(dt) if dt > 0 else None
    except Exception:
        out['t_span'] = None
    # 首三点 Menger 曲率（|k| = 4A/(abc)）；直路 ≈0
    if len(poses) >= 3:
        try:
            a = poses[0].pose.position
            b = poses[1].pose.position
            c = poses[2].pose.position
            ab = math.hypot(b.x - a.x, b.y - a.y)
            bc = math.hypot(c.x - b.x, c.y - b.y)
            ca = math.hypot(a.x - c.x, a.y - c.y)
            cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
            if ab > 1e-6 and bc > 1e-6 and ca > 1e-6:
                out['k_est'] = round(2.0 * cross / (ab * bc * ca), 4)
        except Exception:
            out['k_est'] = None
    return out


def normalize_state_turnmode(state_tm) -> int:
    """CarState.turnmode（反馈侧）→ CarCtr.turnmode（指令侧）同一编号。

    依据 control/car_ctr/src/car_ctr.cpp:46 map_state_turnmode_to_cmd（2026-11-06 实车标定后）：
      0=阿克曼→0 / 1=斜行→1 / 2=横移→2 / 4=原地自转→4；5=自转占位(轮速强制0)→-1（未到位）；
      其他/未知→-1。**不用 CarState.msg 旧注释（2=自转/4=斜行）**——那是旧残留，会导致
      "下发 4、反馈 2" 被误判成模式不一致。
    """
    try:
        tm = int(state_tm)
    except (TypeError, ValueError):
        return -1
    return {0: 0, 1: 1, 2: 2, 4: 4}.get(tm, -1)


def build_snapshot_row(cache: dict, t: float, wall: str,
                       rates: dict | None = None) -> dict:
    """把最新缓存 cache 拼成一行快照（扁平 dict，key 见文档 06）。

    cache 结构（节点回调维护；缺字段视为 None）：
      loc:        {x,y,theta,stamp,age}
      cmd_vel:    {v,w}
      traj:       {n_pts, arc_len}
      spin_target:{data}
      traj_mode:  {data}
      bridge_state:{data}
      rda_status: dict(parse_kv_string)
      ctrl_status: dict(parse_kv_string)
      car_cmd:    {enable,turnmode,speed,angle,ctr_mode,source,up_down_flag,heartbeat}
      car_state:  {enable,ctrmode,turnmode,speed_mean,speeds:[4],errcode:[8]}
      obstacles:  {n,nearest_dist,nearest_id,nearest_type}
      goal:       {x,y}
      diag:       dict(JSON /rda/diag)
      gps:        {lat,lon}
    """
    loc = cache.get('loc') or {}
    cv = cache.get('cmd_vel') or {}
    tr = cache.get('traj') or {}
    cc = cache.get('car_cmd') or {}
    cs = cache.get('car_state') or {}
    # ★2026-11-06：/car_state 断流后 cache 仍保留最后一帧（实车 run_19700108_135820 断流
    #   170s，仍一直报 cs_enable=0 / cs_ctrmode=1），会让人误判"底盘有回传"。
    #   按 age 判陈旧：>1.0s 没有新帧 → cs_* 一律输出 None，并标 cs_stale=1。
    cs_age = (rates or {}).get('car_state_age')
    #   age 缺失（例如离线用 build_snapshot_row 造 row、或该 run 从未配置 rates）→ 不做陈旧判定
    cs_stale = 1 if (cs_age is not None and float(cs_age) > 1.0) else 0
    if cs_stale:
        cs = {}
    ob = cache.get('obstacles') or {}
    goal = cache.get('goal') or {}
    dg = cache.get('diag') or {}
    rs = cache.get('rda_status') or {}
    ks = cache.get('ctrl_status') or {}

    row = {
        't': _finite(t),
        'wall': wall,
        # ---- 定位 ----
        'loc_x': _finite(loc.get('x')),
        'loc_y': _finite(loc.get('y')),
        'loc_theta': _finite(loc.get('theta')),
        'loc_stamp': _finite(loc.get('stamp')),
        'loc_age': _finite(loc.get('age')),
        # ---- 障碍 ----
        'n_obs': ob.get('n'),
        'nearest_obs_dist': _finite(ob.get('nearest_dist')),
        'nearest_obs_id': ob.get('nearest_id'),
        'nearest_obs_type': ob.get('nearest_type'),
        # ★2026-11-06：最近障碍在**车体系**的坐标（x 前向 / y 左向）+ 正前窄条带最近距离，
        # 用于判断"障碍是正前/偏左/偏右、横向余量够不够"，只有 map 系距离看不出来。
        'nbx': _finite(ob.get('nbx')),
        'nby': _finite(ob.get('nby')),
        'n_near4': ob.get('n_near4'),
        'front_min': _finite(ob.get('front_min')),
        # ---- 规划（/rda/status + /rda/diag 融合） ----
        'v_cmd': _finite(cv.get('v')),
        'w_cmd': _finite(cv.get('w')),
        'plan_ok': rs.get('ok'),
        'reason': rs.get('reason'),
        'dist_goal': _finite(rs.get('dg')),
        'trap_active': dg.get('trap_active'),
        'goal_spin_requests':dg.get('goal_spin_requests'),
        'recovery_goal':dg.get('recovery_goal'),
        'recovery_failed':dg.get('recovery_failed'),
        'recovery_elapsed':dg.get('recovery_elapsed'),
        'planner':dg.get('planner'),
        'stall': dg.get('stall'),
        'stall_run': _finite(dg.get('stall_run')),
        'freeze_stall': dg.get('freeze_stall'),
        'bridge_state': dg.get('bridge_state', (cache.get('bridge_state') or {}).get('data')),
        'spin_target': _finite((cache.get('spin_target') or {}).get('data')),
        'traj_mode': (cache.get('traj_mode') or {}).get('data'),
        # ★2026-11-06：仲裁内部量落盘（/rda/diag 已发布但此前未记录）——
        # 排查"自转/陷阱不激活"必须看到 arbit 与 v_streak 的实际值。
        'arbit': dg.get('arbit'),
        'v_streak': dg.get('v_streak'),
        # ★2026-11-06 用户口径仲裁量（决定"何时切自转/为何没切/trap 为何没进"）
        'ack_bad_cnt': dg.get('ack_bad_cnt'),      # 连续不可行帧数（阈值 5）
        'spin_count': dg.get('spin_count'),        # 自转累计激活次数（≥2 才可能 trap）
        'spin_gap': _finite(dg.get('spin_gap')),   # 最近两次自转点间距 m（<0.5 → trap）
        'spin_target_apf': _finite(dg.get('spin_target_apf')),
        'clear_seq': dg.get('clear_seq'),          # 清空跟踪路径命令序号
        'road_base_len': _finite(dg.get('road_base_len')),  # 下发路基弧长（0=自转/停车）
        'trap_sub_dist0': _finite(dg.get('trap_sub_dist0')),
        'stall_time': _finite(dg.get('stall_time')),
        'omega_ref': _finite(dg.get('omega_ref')),
        'sub_goal_x': _finite(dg.get('sub_goal_x')),
        'sub_goal_y': _finite(dg.get('sub_goal_y')),
        'traj_arc_len': _finite(dg.get('traj_arc_len', tr.get('arc_len'))),
        'traj_n_pts': tr.get('n_pts'),   # 2026-11-04：轨迹点数（<2 说明退化为平轨迹，配合弧长判据取证）
        # ★2026-11-06：路基内容摘要（验证"按 (v,ω) 积分出的 2.5m 路基"是否与规划一致）
        'tr_v0': _finite(tr.get('v0')),          # 首点参考速度（纵向 PID 目标来源）
        'tr_v_last': _finite(tr.get('v_last')),
        'tr_t_span': _finite(tr.get('t_span')),  # 时间轴跨度 s（≈ len/|v|）
        'tr_yaw0': _finite(tr.get('yaw0')),      # 首点朝向（应≈规划时的车头）
        'tr_k_est': _finite(tr.get('k_est')),    # 首三点曲率 1/m（应≈ω/v）
        'tr_front_len': _finite(tr.get('front_len')),  # 首末直线距离（≠弧长→弯的）
        # 清空命令（/rda/traj_clear 话题直收；与 diag 的 clear_seq、tracker 的 tk_clear_seq 三方对照）
        'tc_seq': (cache.get('traj_clear') or {}).get('seq'),
        # ---- 控制 /car_cmd 全字段 ----
        'cc_enable': cc.get('enable'),
        'cc_turnmode': cc.get('turnmode'),
        'cc_speed': _finite(cc.get('speed')),
        'cc_angle': _finite(cc.get('angle')),
        'cc_ctr_mode': cc.get('ctr_mode'),
        'cc_source': cc.get('source'),
        'cc_up_down_flag': cc.get('up_down_flag'),
        'cc_heartbeat': cc.get('heartbeat'),
        # ---- 底盘 /car_state ----
        'cs_stale': cs_stale,
        'cs_enable': cs.get('enable'),
        'cs_ctrmode': cs.get('ctrmode'),
        # ★2026-11-06：cs_turnmode 已归一化到 **指令侧(CarCtr)** 同一编号（0/1/2/4，-1=未到位），
        # 与 cc_turnmode 直接可比；原始反馈编号另存 cs_turnmode_raw（诊断底盘回显是否滞后）。
        'cs_turnmode': cs.get('turnmode_norm'),
        'cs_turnmode_raw': cs.get('turnmode'),
        'v_act': _finite(cs.get('speed_mean')),
        'cs_speed_fl': _finite((cs.get('speeds') or [None] * 4)[0]),
        'cs_speed_fr': _finite((cs.get('speeds') or [None] * 4)[1]),
        'cs_speed_rl': _finite((cs.get('speeds') or [None] * 4)[2]),
        'cs_speed_rr': _finite((cs.get('speeds') or [None] * 4)[3]),
        'cs_errcode': (cs.get('errcode') or [None] * 8),
        # ---- 跟踪 /rda/ctrl_status 解析（★2026-11-06 重构后字段：mode/branch/v/steer/
        #      v_ref/e_y/e_th/k_ref/s/ahead/n/spin_err/clear_seq/seq）----
        'tk_mode': ks.get('mode'),            # 0=ACK 1=空窗 2=自转中 3=到位 4=超时
        'tk_branch': ks.get('branch'),
        'tk_spin_err': _finite(ks.get('spin_err')),
        'tk_v_ref': _finite(ks.get('v_ref')),      # 路基该点参考速度（纵向 PID 目标）
        'tk_v_out': _finite(ks.get('v')),          # 实际下发速度
        'tk_steer': _finite(ks.get('steer')),      # 下发前轮转角 deg
        'tk_e_y': _finite(ks.get('e_y')),          # 横向误差 m（LQR 输入）
        'tk_e_th': _finite(ks.get('e_th')),        # 航向误差 deg
        'tk_k_ref': _finite(ks.get('k_ref')),      # 参考曲率 1/m
        'tk_preview_distance':_finite(ks.get('preview')),
        'tk_preview':((cache.get('control_diag') or {}).get('tracking') or {}).get('preview'),
        'tk_s': _finite(ks.get('s')),              # 车沿轨迹里程 m
        'tk_ahead': _finite(ks.get('ahead')),      # 车前可用参考长度 m
        'tk_n': ks.get('n'),                       # 滑窗参考点数
        'tk_clear_seq': ks.get('clear_seq'),       # 已消费的清空命令序号
        'tk_seq': ks.get('seq'),                   # 最近接受到的参考序号
        'tk_traj_arc': _finite(tr.get('arc_len')),  # 兼容旧字段（改由 traj 缓存给）
        # 注：tk_nearest_idx / tk_lx / tk_ly 已删除——2026-11-06 重构后跟踪层不再发布
        #     "最近点索引/最近点坐标"，等价信息改由 tk_s（沿轨迹里程）+ tk_e_y/e_th 提供。
        # ---- 目标点 ----
        'goal_x': _finite(goal.get('x')),
        'goal_y': _finite(goal.get('y')),
        # ---- GPS（可选） ----
        'gps_lat': _finite((cache.get('gps') or {}).get('lat')),
        'gps_lon': _finite((cache.get('gps') or {}).get('lon')),
    }
    driver=cache.get('driver_diag') or {};ages=driver.get('wheel_feedback_age_s',[None]*4)
    extra_age=(rates or {}).get('driver_diag_age')
    valid=[bool(extra_age is not None and a is not None and a+extra_age<.3) for a in ages]
    row['wheel_feedback_valid']=valid;row['cs_speeds_raw']=cs.get('speeds');row['v_act_raw_car_state']=row['v_act']
    for i,key in enumerate(['cs_speed_fl','cs_speed_fr','cs_speed_rl','cs_speed_rr']):
        if not valid[i]:row[key]=None
    if not all(valid):row['v_act']=None
    row['actual_steering_feedback_available']=False # current firmware interface provides no measured steer
    if rates:
        row.update(rates)
    return row


def diff_events(prev: dict | None, cur: dict, t: float, wall: str) -> list:
    """对比相邻两行快照，生成事件行列表。纯函数。

    事件类型：trap_on/trap_off、freeze_on/freeze_off、bridge_state_change、
    ctrl_mode_change、stage_change、branch_change、goal_update。
    prev 为 None（首行）只返回 goal 初始化事件（不把初始化当事件）。
    """
    evs = []
    if prev is None:
        return evs

    def emit(name, detail):
        evs.append({'t': t, 'wall': wall, 'event': name, 'detail': detail})

    # trap 激活/退出
    if prev.get('trap_active') != cur.get('trap_active'):
        emit('trap_on' if cur.get('trap_active') else 'trap_off',
             {'from': prev.get('trap_active'), 'to': cur.get('trap_active'),
              'stall_run': cur.get('stall_run')})
    # freeze 开始/结束
    if prev.get('freeze_stall') != cur.get('freeze_stall'):
        emit('freeze_on' if cur.get('freeze_stall') else 'freeze_off',
             {'from': prev.get('freeze_stall'), 'to': cur.get('freeze_stall'),
              'bridge_state': cur.get('bridge_state')})
    # 执行状态切换（0/1/2）
    if prev.get('bridge_state') != cur.get('bridge_state'):
        emit('bridge_state_change',
             {'from': prev.get('bridge_state'), 'to': cur.get('bridge_state')})
    # 跟踪状态机 mode 切换（0=ACK 1=空窗HANGUP 2=自转中 3=到位 4=超时）
    if prev.get('tk_mode') != cur.get('tk_mode'):
        emit('ctrl_mode_change',
             {'from': prev.get('tk_mode'), 'to': cur.get('tk_mode'),
              'branch': cur.get('tk_branch'), 'n_ref': cur.get('tk_n')})
    # ★2026-11-06：清空跟踪路径命令边沿（模式切换 / trap 激活 / trap 退出）
    if prev.get('clear_seq') != cur.get('clear_seq'):
        emit('path_clear',
             {'from': prev.get('clear_seq'), 'to': cur.get('clear_seq'),
              'arbit': cur.get('arbit'), 'trap': cur.get('trap_active'),
              'traj_mode': cur.get('traj_mode')})
    # ★2026-11-06：/rda/traj_clear 话题直收的边沿（与上面 path_clear 对照：
    #   若这个话题跳了而 tracker 的 tk_clear_seq 没跟 → 命令丢失/订阅未生效）
    if prev.get('tc_seq') != cur.get('tc_seq'):
        emit('traj_clear_cmd',
             {'from': prev.get('tc_seq'), 'to': cur.get('tc_seq'),
              'tk_clear_seq': cur.get('tk_clear_seq'), 'tk_mode': cur.get('tk_mode')})
    # ★2026-11-06：trap 子目标漂移取证（用户口径要求"距离和方向保持不变"）
    if (prev.get('sub_goal_x') != cur.get('sub_goal_x')
            or prev.get('sub_goal_y') != cur.get('sub_goal_y')):
        emit('sub_goal_change',
             {'from': [prev.get('sub_goal_x'), prev.get('sub_goal_y')],
              'to': [cur.get('sub_goal_x'), cur.get('sub_goal_y')],
              'trap': cur.get('trap_active')})
    # ★2026-11-06：自转激活次数递增 = 真正进入自转（目标角见 spin_target/APF）
    if prev.get('spin_count') != cur.get('spin_count'):
        emit('spin_start' if (cur.get('spin_count') or 0) > (prev.get('spin_count') or 0)
             else 'spin_reset',
             {'from': prev.get('spin_count'), 'to': cur.get('spin_count'),
              'gap': cur.get('spin_gap'), 'spin_target': cur.get('spin_target'),
              'apf': cur.get('spin_target_apf')})
    # 双分支切换（A/B/C）
    if prev.get('tk_branch') != cur.get('tk_branch'):
        emit('branch_change',
             {'from': prev.get('tk_branch'), 'to': cur.get('tk_branch'),
              'mode': cur.get('tk_mode')})
    # 全局目标更新
    if (prev.get('goal_x') != cur.get('goal_x')
            or prev.get('goal_y') != cur.get('goal_y')):
        emit('goal_update',
             {'from': [prev.get('goal_x'), prev.get('goal_y')],
              'to': [cur.get('goal_x'), cur.get('goal_y')]})
    return evs


def append_jsonl(path: str, row: dict) -> None:
    """追加一行 JSON（ensure_ascii=False）。"""
    with open(path, 'a', encoding='utf-8') as f:
        f.write(json.dumps(row, ensure_ascii=False) + '\n')


# 算法参数清单（2026-11-04 修复A：录制开始即把运行时生效值写入 params.json）。
# 来源链：代码默认值（TrajTrackerParams / rda_dwa.params.Params）+ real_vehicle.yaml
#   traj_tracker_node 段覆盖（实车 yaml 覆盖了代码默认值，如 traj_len_threshold=0.075、
#   spin_w_deadband=0.06）。trap/规划参数 yaml 未覆盖，取代码默认值。
_TRACKER_PARAM_KEYS = [
    # ★2026-11-06 重构后实际生效的跟踪层参数（里程域接续融合 + B样条 + LQR + PID）
    'fusion_len', 'keep_behind', 'window_len', 'tail_join_tol', 'min_pts',
    'bspline_ds', 'bspline_min_pts',
    'lqr_ds', 'lqr_q_y', 'lqr_q_th', 'lqr_r', 'preview_time', 'max_steer_rate_deg',
    'pid_kp', 'pid_ki', 'pid_kd', 'pid_i_limit', 'acc_max', 'dec_max', 'v_meas_gain',
    'spin_w_max', 'spin_wait_time', 'spin_timeout', 'spin_stop_hold',
    'rotate_precision_deg', 'rotate_decel_deg', 'wheelbase', 'max_steer_deg',
    'cmd_timeout', 'traj_timeout', 'ref_window_stale',
]
# 规划侧模式仲裁参数（BridgeConfig；2026-11-06 用户口径）
_ARBITRATION_PARAM_KEYS = [
    'ack_hold_enable', 'ack_hold_v_ratio', 'ack_hold_w_ratio',
    'ack_bad_count', 'ack_bad_ratio', 'ack_guard_time',
    'spin_count_for_trap', 'spin_gap_eps', 'stall_obs_dist', 'stall_escape_time',
    'trap_exit_dist', 'trap_exit_far', 'trap_exit_ratio', 'trap_exit_ang_deg',
    'trap_exit_sector_deg', 'trap_exit_sector_range', 'trap_exit_block_max',
    'road_base_len', 'road_base_ds',
]
_TRAP_PLANNER_PARAM_KEYS = [
    # trap 状态机（rda_dwa.params.Params）
    'trap_stall_speed', 'trap_stall_time', 'trap_nominal_dt', 'trap_near_goal_stall_m',
    'trap_stall_steps', 'trap_progress_eps', 'trap_min_steps', 'trap_max_steps',
    'trap_max_duration', 'trap_scan_range', 'trap_subgoal_dist',
    'trap_exit_dist', 'trap_exit_goal_ang_deg', 'trap_exit_sector_deg',
    'trap_exit_sector_range',
    # 采样/代价/绕行/APF（决定"为什么直冲、为什么不绕"的直接输入）
    'dt', 'sim_time', 'v_resolution', 'omega_resolution', 'alpha_smooth', 'sticky_eps',
    'cost_horizon', 'd_safe', 'goal_tolerance',
    'max_linear_velocity', 'max_angular_velocity',
    'enable_detour', 'detour_look_dist', 'detour_min_along', 'detour_clear_margin',
    'detour_pass_ahead', 'detour_max_lateral',
    'apf_k_att', 'apf_k_rep', 'apf_d0',
    'backup_enabled', 'spin_enabled',
]


def collect_algorithm_params(tracker_params=None, planner_params=None,
                            yaml_path: str | None = None) -> dict:
    """收集代码默认/配置副本（不代表远程节点实时生效值），供 params.json 落地。纯函数可本地冒烟。

    tracker_params / planner_params：可注入已有实例（便于测试）；None 时惰性 import。
    yaml_path：real_vehicle.yaml 路径；存在则用其 traj_tracker_node 段覆盖跟踪类参数。
    任何 import/解析失败都降级为该字段 None，不阻断录制。
    """
    ap: dict = {}
    try:
        if tracker_params is None:
            from rda_dwa_real.traj_tracker_core import TrajTrackerParams
            tracker_params = TrajTrackerParams()
        for k in _TRACKER_PARAM_KEYS:
            ap[k] = getattr(tracker_params, k, None)
    except Exception:
        for k in _TRACKER_PARAM_KEYS:
            ap[k] = None
    try:
        if planner_params is None:
            from rda_dwa.params import Params
            planner_params = Params()
        for k in _TRAP_PLANNER_PARAM_KEYS:
            ap[k] = getattr(planner_params, k, None)
    except Exception:
        for k in _TRAP_PLANNER_PARAM_KEYS:
            ap[k] = ap.get(k, None)
    # 仲裁参数代码默认（BridgeConfig）——yaml 缺项时回落，避免 analyze 时看到一片 None
    arb_defaults: dict = {}
    try:
        from rda_dwa_real.vehicle_bridge import BridgeConfig
        _bc = BridgeConfig()
        for k in _ARBITRATION_PARAM_KEYS:
            arb_defaults[k] = getattr(_bc, k, None)
    except Exception:
        arb_defaults = {}
    # yaml traj_tracker_node 段覆盖（实车真实生效值）
    if yaml_path:
        try:
            import yaml
            with open(yaml_path, encoding='utf-8') as f:
                doc = yaml.safe_load(f)
            tt = (doc.get('traj_tracker_node') or {}).get('ros__parameters') or {}
            for k in list(ap.keys()):
                if k in tt:
                    ap[k] = tt[k]
            # 仲裁参数来自 rda_dwa_real_bridge 段（同一 yaml 另一节，缺项回落代码默认）
            br = (doc.get('rda_dwa_real_bridge') or {}).get('ros__parameters') or {}
            for k in _ARBITRATION_PARAM_KEYS:
                ap[k] = br.get(k, arb_defaults.get(k))
        except Exception:
            pass
    else:
        for k in _ARBITRATION_PARAM_KEYS:
            ap[k] = arb_defaults.get(k)
    return ap


# ====================== ROS 节点区 ======================


def _make_node():
    """工厂：延迟 import rclpy，便于在无 ROS 环境做纯函数冒烟。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Float32, Int32, String
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import NavSatFix, Imu
    from rcl_interfaces.msg import ParameterEvent
    from rcl_interfaces.srv import ListParameters,GetParameters

    try:
        from control_msgs.msg import CarCtr, CarState
    except Exception:
        CarCtr = None
        CarState = None
    try:
        from rda_dwa_real_msgs.msg import ObstacleArray
    except Exception:
        ObstacleArray = None

    class _DiagRecorderNode(Node):
        """纯观测记录节点：订阅→缓存→定时快照落地 JSONL。"""

        def __init__(self):
            super().__init__('diag_recorder_node')

            # ---------------- 参数 ----------------
            self.declare_parameter('rec_hz', 20.0)                 # 快照频率
            self.declare_parameter('out_dir', '')                 # 空串 → ~/ros2_diag
            self.declare_parameter('record_gps', False)            # 是否订阅 /fix
            self.declare_parameter('rate_window', 2.0)             # 消息率统计窗 s
            self.declare_parameter('drop_timeout', 3.0)            # 连续无到达秒数 → topic_drop 事件
            # 启动空窗：run_rda_dwa 里本节点立即起、算法栈 12s 后才起，
            # 若不设空窗会在开头误报一堆 topic_drop（/cmd_vel、/rda/traj 等）
            self.declare_parameter('start_grace', 15.0)            # 启动后 N 秒内不报 topic_drop

            for key,value in {'config_file':'','experiment_label':'','bag_extra_topics':'','record_bag':True,'bag_max_gb':8.0,'bag_min_free_gb':2.0,'writer_queue_records':4096,'writer_queue_mb':32.0,'message_segment_mb':64.0}.items():self.declare_parameter(key,value)
            self._rec_hz = float(self.get_parameter('rec_hz').value)
            out_dir_param = str(self.get_parameter('out_dir').value).strip()
            self._out_dir_base = out_dir_param or os.path.expanduser('~/ros2_diag')
            self._record_gps = bool(self.get_parameter('record_gps').value)
            self._rate_window = float(self.get_parameter('rate_window').value)
            self._drop_timeout = float(self.get_parameter('drop_timeout').value)
            self._start_grace = float(self.get_parameter('start_grace').value)

            # ---------------- 批次目录 ----------------
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._run_dir = os.path.join(self._out_dir_base, f'run_{stamp}_{time.time_ns()%10**9:09d}')
            os.makedirs(self._run_dir, exist_ok=True)
            self._snap_path = os.path.join(self._run_dir, 'snapshots.jsonl')
            self._event_path = os.path.join(self._run_dir, 'events.jsonl')
            self._params_path = os.path.join(self._run_dir, 'params.json')
            self._writer=AsyncJsonlWriter(self._run_dir,max_records=int(self.get_parameter('writer_queue_records').value),max_bytes=int(float(self.get_parameter('writer_queue_mb').value)*1024**2),rotate_bytes=int(float(self.get_parameter('message_segment_mb').value)*1024**2))
            self._t0=time.monotonic();self._global_seq=0;self._topic_seq={};self._envelopes={};self._source_history={};self._health_before=None;self._last_snapshot_t=None;self._runtime_clients={};self._metadata_at=-1e9
            self._bag=BagCapture(self._run_dir,topics=BAG_TOPICS+str(self.get_parameter('bag_extra_topics').value).replace(',',' ').split(),enabled=bool(self.get_parameter('record_bag').value),total_bytes=int(float(self.get_parameter('bag_max_gb').value)*1024**3),min_free_bytes=int(float(self.get_parameter('bag_min_free_gb').value)*1024**3))

            # ---------------- 缓存 ----------------
            self.cache: dict = {}
            self._prev_row: dict | None = None
            # 消息到达时间环（话题 → deque[monotonic]）
            self._arrival: dict[str, deque] = {}
            # 2026-11-05 完善：固定 hz 字段 + last_seen + topic_drop
            # 全部声明话题（hz/age 每行固定字段；未到达 hz=0.0、age=None）
            self._rate_topics = [
                'loc', 'cmd_vel', 'traj', 'spin_target', 'traj_mode', 'traj_clear',
                'bridge_state', 'ctrl_status', 'rda_status', 'car_cmd',
                'car_state', 'obstacles', 'goal', 'diag','plan','session','perception_diag','control_diag','driver_diag','driver_io','imu','parameter_events','test_note',
            ] + (['gps'] if self._record_gps else [])
            # 仅预期连续发布的话题做 drop 检测；goal/diag/gps/traj_mode 为事件型或可选，
            # 正常可长期无到达，不纳入（避免误报）
            self._drop_topics = [
                'loc','car_cmd','obstacles','plan','control_diag','driver_diag',
            ]
            self._last_arrival: dict[str, float] = {}
            self._drop_state: dict[str, bool] = {}
            # P0-5 (U9)：turnmode 错位计数（纯观测，不影响控制/规划）。
            # cmd(cc)==4(自转) 且 state(cs)!=4 的累计帧数与首次/最近错位时间。
            self._tm_mismatch_count = 0
            self._tm_first_mis_t: float | None = None
            self._tm_last_mis_t: float | None = None

            # ---------------- QoS ----------------
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST)
            qos_best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE,
                                  history=HistoryPolicy.KEEP_LAST)

            self._registered_topics=[]
            def sub(name, topic, typ, cb, q=qos):
                if typ is None:
                    return None
                self._registered_topics.append(topic)
                def observed(msg):
                    self._record_message(name,topic,typ,msg)
                    cb(msg)
                return self.create_subscription(typ,topic,observed,q)

            # ---------------- 订阅 ----------------
            sub('loc', '/localization', Odometry, self._cb_loc, qos_best)
            sub('cmd_vel', '/cmd_vel', Twist, self._cb_cmd)
            sub('traj', '/rda/traj', Path, self._cb_traj, qos_best)
            sub('traj_clear', '/rda/traj_clear', Int32, self._cb_traj_clear)   # ★2026-11-06 新增
            sub('spin_target', '/rda/spin_target', Float32, self._cb_spin)
            sub('traj_mode', '/rda/traj_mode', Int32, self._cb_traj_mode)
            sub('bridge_state', '/car_bridge_state', Int32, self._cb_bridge_state)
            sub('ctrl_status', '/rda/ctrl_status', String, self._cb_ctrl_status)
            sub('rda_status', '/rda/status', String, self._cb_rda_status)
            sub('car_cmd', '/car_cmd', CarCtr, self._cb_car_cmd)
            sub('car_state', '/car_state', CarState, self._cb_car_state, qos_best)
            sub('obstacles', '/rda/obstacles', ObstacleArray, self._cb_obstacles, qos_best)
            sub('goal', '/move_base_simple/goal', PoseStamped, self._cb_goal)
            sub('diag', '/rda/diag', String, self._cb_diag)
            if self._record_gps:
                sub('gps', '/fix', NavSatFix, self._cb_gps, qos_best)

            sub('plan','/rda/plan',String,self._cb_atomic_plan,qos_best)
            sub('session','/rda/session_state',String,self._cb_session_result,qos_best)
            for key,topic in [('perception_diag','/rda/perception_diag'),('control_diag','/rda/control_diag'),('driver_diag','/car_driver/diag')]:
                sub(key,topic,String,lambda msg,k=key:self._cb_json(k,msg),qos_best)
            sub('driver_io','/car_driver/io',String,lambda msg:None,qos_best)
            sub('imu','/imu/data',Imu,lambda msg:None,qos_best)
            sub('test_note','/rda/test_note',String,lambda msg:self._writer.submit('operator_notes.jsonl',dict(self._envelopes.get('test_note',{}),note=msg.data),critical=True),qos_best)
            sub('parameter_events','/parameter_events',ParameterEvent,self._cb_parameter_event,qos)
            # ---------------- 时间基线 ----------------
            self._wall0 = datetime.now().isoformat(timespec='seconds')

            # 写 params 副本
            # 修复A：收集运行时生效算法参数（代码默认 + yaml traj_tracker 段覆盖）。
            yaml_path = self._find_yaml()
            algo_params = collect_algorithm_params(yaml_path=yaml_path)
            # ★2026-11-06：把 yaml 里各节点段的全部 ros__parameters 原文落盘，
            # 免得分析时只能靠白名单猜到"某个参数到底生效成多少"。
            yaml_sections = {}
            if yaml_path:
                try:
                    import yaml as _yaml
                    with open(yaml_path, encoding='utf-8') as f:
                        _doc = _yaml.safe_load(f) or {}
                    for name, body in _doc.items():
                        if isinstance(body, dict) and 'ros__parameters' in body:
                            yaml_sections[name] = body['ros__parameters']
                except Exception:
                    yaml_sections = {}
            with open(self._params_path, 'w', encoding='utf-8') as f:
                json.dump({
                    'rec_hz': self._rec_hz,
                    'out_dir': self._run_dir,
                    'record_gps': self._record_gps,
                    'start_grace': self._start_grace,
                    'drop_timeout': self._drop_timeout,
                    'started_wall': self._wall0,
                    'yaml_path': yaml_path,
                    'algorithm_params': algo_params,
                    'parameter_status':'configured defaults/YAML only; actual live values are in runtime_parameters.jsonl',
                    'requested_config_file':str(self.get_parameter('config_file').value),
                    'actual_subscriptions':self._registered_topics,
                    'schema_version':SCHEMA_VERSION,
                    'snapshot_is_synchronized':False,
                    'record_bag':self._bag.enabled,
                    'experiment_label':str(self.get_parameter('experiment_label').value),
                    'yaml_sections': yaml_sections,
                    'subscribed': [
                        '/localization', '/cmd_vel', '/rda/traj', '/rda/traj_clear',
                        '/rda/spin_target', '/rda/traj_mode', '/car_bridge_state',
                        '/rda/ctrl_status', '/rda/status', '/car_cmd',
                        '/car_state', '/rda/obstacles',
                        '/move_base_simple/goal','/rda/diag','/rda/plan','/rda/session_state','/rda/perception_diag','/rda/control_diag','/car_driver/diag','/car_driver/io','/imu/data','/parameter_events','/rda/test_note',
                    ] + (['/fix'] if self._record_gps else []),
                }, f, ensure_ascii=False, indent=2)

            paths=[__file__,os.path.join(os.path.dirname(__file__),'telemetry_recording.py')]
            import importlib.util
            for name in ['rda_dwa.realtime_ackermann','rda_dwa.motion_safety','rda_dwa_real.realtime_vehicle_core','rda_dwa_real.async_planner','rda_dwa_real.traj_tracker_core','rda_dwa_real.adaptive_preview','rda_dwa_real.bridge_node','rda_dwa_real.traj_tracker_node','rda_dwa_real.lidar_track_node']:
                try:paths.append(importlib.util.find_spec(name).origin)
                except (ImportError,AttributeError):pass
            if yaml_path:paths.append(yaml_path)
            try:
                from ament_index_python.packages import get_package_prefix
                paths.append(os.path.join(get_package_prefix('car_ctr'),'lib','car_ctr','car_ctr'))
            except Exception:pass
            self._writer.submit('provenance.jsonl',code_provenance(paths),critical=True)
            for name in ['/rda_dwa_real_bridge','/traj_tracker_node','/lidar_track_node','/car_ctr','/car_ctr_node']:
                self._runtime_clients[name]={'list':self.create_client(ListParameters,name+'/list_parameters'),'get':self.create_client(GetParameters,name+'/get_parameters'),'stage':'idle','next':0.}
            self._bag.start()
            self.create_timer(1.,self._health_tick)

            # ---------------- 快照定时器 ----------------
            self.create_timer(1.0 / max(1.0, self._rec_hz), self._snapshot_tick)
            self.get_logger().info(
                f'diag_recorder 启动: 输出目录={self._run_dir} rec_hz={self._rec_hz} '
                f'gps={self._record_gps}')

        # ------------------------------------------------------------------
        # 计时辅助
        # ------------------------------------------------------------------
        def _mark(self, topic: str) -> float:
            """记录一次消息到达，返回 stopwatch 秒。"""
            now = time.monotonic()
            dq = self._arrival.setdefault(topic, deque())
            dq.append(now)
            self._last_arrival[topic] = now
            # 裁剪窗口（2026-11-06 修正：只保留 rate_window 内的消息。
            #   旧实现保留 window+1.0s 的消息却按 window 作除数 → 所有话题 hz 虚高
            #   (window+1)/window = 1.5 倍，run_19700108_135820 实测 car_cmd 75Hz(=50Hz)、
            #   traj 15Hz(=10Hz)、loc 65Hz(=43Hz) 全是这条引起的误判）
            while dq and now - dq[0] > self._rate_window:
                dq.popleft()
            return now - self._t0

        def _rates(self) -> dict:
            """各话题近窗口消息率 Hz（2026-11-05：全部声明话题固定输出，0 到达 → 0.0）。"""
            out = {}
            now = time.monotonic()
            for topic in self._rate_topics:
                dq = self._arrival.get(topic, deque())
                while dq and now - dq[0] > self._rate_window:
                    dq.popleft()
                out[f'{topic}_hz'] = round(len(dq) / self._rate_window, 2)
            return out

        def _last_seen_ages(self) -> dict:
            """各话题距上次到达秒数（从未到达 → None）——断流可见性。"""
            out = {}
            now = time.monotonic()
            for topic in self._rate_topics:
                last = self._last_arrival.get(topic)
                out[f'{topic}_age'] = round(now - last, 3) if last is not None else None
            return out

        @staticmethod
        def _stamp_sec(header) -> float:
            try:
                return header.stamp.sec + header.stamp.nanosec * 1e-9
            except Exception:
                return 0.0

        def _find_yaml(self) -> str | None:
            """定位 real_vehicle.yaml（实车 install share 或源码 config 相对路径）。找不到返回 None。"""
            configured=str(self.get_parameter('config_file').value).strip()
            if configured:return os.path.abspath(configured) if os.path.isfile(configured) else None
            candidates = [configured] if configured else []
            candidates += [
                os.path.abspath(os.path.join(os.path.dirname(__file__),
                                             '..', 'config', 'real_vehicle.yaml')),
            ]
            try:
                from ament_index_python.packages import get_package_share_directory
                candidates.append(os.path.join(
                    get_package_share_directory('rda_dwa_real'), 'config',
                    'real_vehicle.yaml'))
            except Exception:
                pass
            for c in candidates:
                try:
                    if os.path.exists(c):
                        return c
                except Exception:
                    continue
            return None

        # ------------------------------------------------------------------
        # 回调（只做"消息→纯数据"提取，业务计算全在纯函数）
        # ------------------------------------------------------------------
        def _cb_loc(self, msg):
            p = msg.pose.pose
            q = p.orientation
            # 四元数 → yaw（与 bridge 同口径）
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            stamp = self._stamp_sec(msg.header)
            # loc_age = now(ros 时钟) - stamp；stamp 为 0 或时钟不同步时为 None
            age = None
            if stamp > 0:
                try:
                    age = self.get_clock().now().nanoseconds * 1e-9 - stamp
                except Exception:
                    age = None
            self.cache['loc'] = {
                'x': p.position.x, 'y': p.position.y, 'theta': yaw,
                'stamp': stamp, 'age': age,
            }

        def _cb_cmd(self, msg):
            self.cache['cmd_vel'] = {'v': msg.linear.x, 'w': msg.angular.z}

        def _cb_traj(self, msg):
            # ★2026-11-06：路基摘要（弧长/首末速度/时间跨度/首点朝向/首三点曲率）
            self.cache['traj'] = summarize_road_base(msg.poses)

        def _cb_traj_clear(self, msg):
            self.cache['traj_clear'] = {'seq': int(msg.data)}

        def _cb_spin(self, msg):
            self.cache['spin_target'] = {'data': msg.data}

        def _cb_traj_mode(self, msg):
            self.cache['traj_mode'] = {'data': msg.data}

        def _cb_bridge_state(self, msg):
            self.cache['bridge_state'] = {'data': msg.data}

        def _cb_ctrl_status(self, msg):
            self.cache['ctrl_status'] = parse_kv_string(msg.data)

        def _cb_rda_status(self, msg):
            self.cache['rda_status'] = parse_kv_string(msg.data)

        def _cb_car_cmd(self, msg):
            self.cache['car_cmd'] = {
                'enable': int(msg.enable), 'turnmode': int(msg.turnmode),
                'speed': float(msg.speed), 'angle': float(msg.angle),
                'ctr_mode': int(msg.ctr_mode), 'source': int(msg.source),
                'up_down_flag': int(msg.up_down_flag), 'heartbeat': int(msg.heartbeat),
            }

        def _cb_car_state(self, msg):
            speeds = [float(s) for s in msg.speed]
            mean = sum(speeds) / len(speeds) if speeds else None
            self.cache['car_state'] = {
                'enable': int(msg.enable), 'ctrmode': int(msg.ctrmode),
                'turnmode': int(msg.turnmode),
                # 归一化到指令侧编号，与 /car_cmd.turnmode 直接可比（见 normalize_state_turnmode）
                'turnmode_norm': normalize_state_turnmode(msg.turnmode),
                'speed_mean': mean, 'speeds': speeds, 'speed_len': len(speeds),
                'errcode': [int(e) for e in msg.errcode],
            }

        def _cb_obstacles(self, msg):
            # ★2026-11-06 口径修复：障碍发布在 map 系，原来用 math.hypot(ob.x, ob.y)
            # 实际算的是"障碍到地图原点"的距离（实车日志里 nearest_obs_dist 恒 ≈80m），
            # 完全看不出"障碍有多近"，导致无法从日志判断绕障/停车时机。
            # 改为相对当前车位姿的距离；并额外给车体系坐标（正前/偏左/偏右一眼可见）。
            loc = self.cache.get('loc')
            nearest = None
            nearest_id = None
            nearest_type = None
            nbx = nby = None
            n_near4 = 0
            front_min = None
            if loc is not None:
                yaw = float(loc['theta'])
                for ob in msg.obstacles:
                    dx = ob.x - loc['x']
                    dy = ob.y - loc['y']
                    d = math.hypot(dx, dy)
                    bfx, bfy = body_frame(dx, dy, yaw)   # 车体系：x 前向 / y 左向
                    if d < 4.0:
                        n_near4 += 1
                    # 正前窄条带（|横向| ≤ 0.4m≈半车宽）内的最小纵向距离 = "贴障"判据的真实输入
                    if bfx > 0.0 and abs(bfy) <= 0.4:
                        front_min = bfx if front_min is None else min(front_min, bfx)
                    if nearest is None or d < nearest:
                        nearest = d
                        nearest_id = ob.id
                        nearest_type = ob.type
                        nbx, nby = bfx, bfy
            self.cache['obstacles'] = {
                'objects': [dict(id=int(o.id),type=str(o.type),x=float(o.x),y=float(o.y),length=float(o.length),width=float(o.width),heading=float(o.heading),speed=float(o.speed),has_shape_heading=bool(getattr(o,'has_shape_heading',False)),shape_heading=float(getattr(o,'shape_heading',o.heading))) for o in msg.obstacles],
                'source_stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                'n': len(msg.obstacles),
                'nearest_dist': nearest,
                'nearest_id': nearest_id,
                'nearest_type': nearest_type,
                'nbx': nbx,      # 最近障碍：车体系前向 m
                'nby': nby,      # 最近障碍：车体系左向 m（>0 偏左, <0 偏右）
                'n_near4': n_near4,
                'front_min': front_min,
            }

        def _cb_goal(self, msg):
            self.cache['goal'] = {'x': msg.pose.position.x, 'y': msg.pose.position.y}

        def _cb_atomic_plan(self,msg):
            try: payload=json.loads(msg.data)
            except (ValueError,TypeError):return
            self._writer.submit('plans.jsonl',dict(self._envelopes.get('plan',{}),payload=payload))

        def _cb_session_result(self,msg):
            try: payload=json.loads(msg.data)
            except (ValueError,TypeError):return
            self._writer.submit('sessions.jsonl',dict(self._envelopes.get('session',{}),payload=payload))

        def _cb_diag(self, msg):
            try:
                self.cache['diag'] = json.loads(msg.data)
            except (json.JSONDecodeError, ValueError):
                pass

        def _cb_gps(self, msg):
            self.cache['gps'] = {'lat': msg.latitude, 'lon': msg.longitude}

        def _record_message(self,key,topic,typ,msg):
            elapsed=self._mark(key);self._global_seq+=1;self._topic_seq[key]=self._topic_seq.get(key,0)+1
            source,frame=source_header(msg);prior=self._source_history.get(key)
            repeated=source is not None and source==prior;out_of_order=source is not None and prior is not None and source<prior
            if source is not None:self._source_history[key]=source
            envelope=dict(schema_version=SCHEMA_VERSION,t=elapsed,monotonic_ns=time.monotonic_ns(),wall_unix_ns=time.time_ns(),ros_receive_ns=self.get_clock().now().nanoseconds,source_stamp_ns=source,frame_id=frame,topic=topic,message_type=typ.__module__.split('.')[0]+'/msg/'+typ.__name__,global_seq=self._global_seq,topic_seq=self._topic_seq[key],source_repeated=repeated,source_out_of_order=out_of_order)
            self._envelopes[key]=envelope
            self._writer.submit('messages.jsonl',dict(envelope,payload=safe_data(msg)))
            if repeated or out_of_order:self._writer.submit('events.jsonl',dict(envelope,event='source_stamp_fault'),critical=True)

        def _cb_json(self,key,msg):
            try:self.cache[key]=json.loads(msg.data)
            except (ValueError,TypeError):self._writer.submit('events.jsonl',dict(t=time.monotonic()-self._t0,event='invalid_json',topic=key),critical=True)

        def _cb_parameter_event(self,msg):
            self._writer.submit('runtime_parameters.jsonl',dict(t=time.monotonic()-self._t0,kind='parameter_event',payload=safe_data(msg)),critical=True)
            for client in self._runtime_clients.values():client['next']=0.

        def _poll_parameters(self,now):
            for name,c in self._runtime_clients.items():
                try:
                    if c['stage']=='idle' and now>=c['next']:
                        if not c['list'].service_is_ready():c['next']=now+5.;continue
                        req=ListParameters.Request();req.depth=0;c['future']=c['list'].call_async(req);c['stage']='listing';c['deadline']=now+3.
                    if c['stage']!='idle' and now>c['deadline']:c['future'].cancel();c['stage']='idle';c['next']=now+10.;continue
                    if c['stage']=='listing' and c['future'].done():
                        names=c['future'].result().result.names;c['names']=names
                        req=GetParameters.Request();req.names=names;c['future']=c['get'].call_async(req);c['stage']='getting';c['deadline']=now+3.
                    if c['stage']=='getting' and c['future'].done():
                        values=c['future'].result().values
                        self._writer.submit('runtime_parameters.jsonl',dict(t=now-self._t0,kind='actual_parameter_service',node=name,parameters=dict(zip(c['names'],[parameter_value(v) for v in values]))),critical=True)
                        c['stage']='idle';c['next']=now+30.
                except Exception as exc:
                    self._writer.submit('events.jsonl',dict(t=now-self._t0,event='parameter_read_error',node=name,error=type(exc).__name__),critical=True);c['stage']='idle';c['next']=now+10.

        def _health_tick(self):
            now=time.monotonic();status=dict(t=now-self._t0,writer=self._writer.stats(),bag=self._bag.poll(now),received_messages=self._global_seq,topic_counts=dict(self._topic_seq))
            self._writer.submit('recorder_health.jsonl',status,critical=True)
            signature=(status['writer']['dropped'],status['writer']['write_errors'],status['bag']['state'])
            if self._health_before!=signature:
                self._writer.submit('events.jsonl',dict(status,event='recorder_health_change'),critical=True);self._health_before=signature
                if signature[0] or signature[1] or signature[2] in ('failed','unavailable','quota_reached','disk_reserve_reached'):self.get_logger().warning('Recording health: '+str(signature))
            self._poll_parameters(now)
            if now-self._metadata_at>=(5. if now-self._t0<30. else 30.):
                self._metadata_at=now
                endpoints={}
                for topic in self._registered_topics:
                    try:
                        endpoints[topic]=[dict(node_name=e.node_name,node_namespace=e.node_namespace,topic_type=e.topic_type,depth=e.qos_profile.depth,reliability=str(e.qos_profile.reliability),durability=str(e.qos_profile.durability)) for e in self.get_publishers_info_by_topic(topic)]
                    except (AttributeError,RuntimeError):endpoints[topic]=None
                self._writer.submit('topic_graph.jsonl',dict(t=now-self._t0,topics=self.get_topic_names_and_types(),publishers=endpoints),critical=True)

        def destroy_node(self):
            self._writer.submit('recorder_health.jsonl',dict(t=time.monotonic()-self._t0,shutdown=True,bag=self._bag.close()),critical=True)
            self._writer.close()
            return super().destroy_node()

        # ------------------------------------------------------------------
        # 快照定时器（核心：纯函数 build_snapshot_row / diff_events / append_jsonl）
        # ------------------------------------------------------------------
        def _check_drops(self, t: float, wall: str) -> list:
            """连续 drop_timeout 秒无到达 → topic_drop；恢复到达 → topic_recover。仅连续话题。

            从未到达的话题在启动 drop_timeout 秒后报一次 topic_drop（since=0.0，标记"从启动即断流"）。
            ★2026-11-06：启动 start_grace 秒内完全不报（算法栈本身延 12s 才起，否则开头全是假 drop）。
            """
            evs = []
            if t < self._start_grace:
                return evs
            now = time.monotonic()
            for topic in self._drop_topics:
                last = self._last_arrival.get(topic)
                if last is None:
                    silent = t > self._drop_timeout
                    since = 0.0
                else:
                    silent = (now - last) > self._drop_timeout
                    since = now - last
                if silent and not self._drop_state.get(topic):
                    self._drop_state[topic] = True
                    evs.append({'t': t, 'wall': wall, 'event': 'topic_drop',
                                'detail': {'topic': topic, 'silent_s': round(since, 2)}})
                elif not silent and self._drop_state.get(topic):
                    self._drop_state[topic] = False
                    evs.append({'t': t, 'wall': wall, 'event': 'topic_recover',
                                'detail': {'topic': topic}})
            return evs

        def _snapshot_tick(self):
            t = time.monotonic() - self._t0
            wall = datetime.now().isoformat(timespec='seconds')
            rates = self._rates()
            rates.update(self._last_seen_ages())
            row = build_snapshot_row(self.cache, t, wall, rates)
            # ---- P0-5 (U9)：turnmode 错位计数（纯观测取证，不改任何控制） ----
            # ★2026-11-06：cs_turnmode 已归一化到指令侧编号（normalize_state_turnmode），
            #   本计数只反映"指令已下发自转、底盘回显还没到 4"（含 -1 未到位），不再是枚举差异误报。
            cc_tm = row.get('cc_turnmode')
            cs_tm = row.get('cs_turnmode')
            if (cc_tm is not None and cs_tm is not None
                    and int(cc_tm) == 4 and int(cs_tm) != 4):
                self._tm_mismatch_count += 1
                if self._tm_first_mis_t is None:
                    self._tm_first_mis_t = t
                self._tm_last_mis_t = t
            row['cc_tm_mismatch_count'] = self._tm_mismatch_count
            row['cc_tm_first_mis_t'] = self._tm_first_mis_t
            row['cc_tm_last_mis_t'] = self._tm_last_mis_t
            row['schema_version']=SCHEMA_VERSION;row['snapshot_is_synchronized']=False
            row['input_messages']=dict(self._envelopes)
            row['snapshot_dt_s']=None if self._last_snapshot_t is None else t-self._last_snapshot_t
            self._last_snapshot_t=t
            row['source_ages_s']={k:(self.get_clock().now().nanoseconds-v['source_stamp_ns'])/1e9 if v['source_stamp_ns'] is not None else None for k,v in self._envelopes.items()}
            row['recorder_health']=self._writer.stats()
            row['perception']=self.cache.get('perception_diag');row['control']=self.cache.get('control_diag');row['driver']=self.cache.get('driver_diag')
            self._writer.submit('snapshots.jsonl',row)
            for ev in diff_events(self._prev_row, row, t, wall):
                self._writer.submit('events.jsonl',ev,critical=True)
            for ev in self._check_drops(t, wall):
                self._writer.submit('events.jsonl',ev,critical=True)
            self._prev_row = row

    return _DiagRecorderNode


def main(args=None):
    import rclpy
    node_cls = _make_node()
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
