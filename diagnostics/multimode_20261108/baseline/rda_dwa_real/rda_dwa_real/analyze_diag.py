"""Offline evidence summary for recorder JSONL; never imports ROS or controls a car.

Schema 2 message records preserve callback order and exact source epochs. Legacy
snapshot-only runs remain readable, with their sampling limits stated explicitly.
The findings are observations to investigate, not automatic root-cause verdicts.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path


def number(value):
    if isinstance(value, bool) or value is None:
        return None
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError, OverflowError):
        return None


def epoch(value):
    # A float cannot preserve contemporary ROS nanoseconds. Never round it into
    # an apparently exact reference identifier.
    if isinstance(value, int) and not isinstance(value, bool) and value > 0:
        return value
    if isinstance(value, str) and value.isdigit() and int(value) > 0:
        return int(value)
    return None


class Stats:
    def __init__(self):
        self.count, self.total = 0, 0.0
        self.minimum = self.maximum = None
        self.recent = deque(maxlen=10000)

    def add(self, value):
        value = number(value)
        if value is None:
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        self.recent.append(value)

    def result(self):
        values = sorted(self.recent)
        return {'count': self.count, 'min': self.minimum, 'max': self.maximum,
                'mean': self.total/self.count if self.count else None,
                'p95': values[min(len(values)-1, int(.95*len(values)))] if values else None,
                'quantile_scope': 'all samples' if self.count <= 10000 else 'last 10000 samples'}


def segment_files(run_dir, stem):
    paths = []
    for p in Path(run_dir).glob(stem+'*.jsonl'):
        suffix = p.stem[len(stem):]
        if suffix == '':
            paths.append((0, p))
        elif suffix.startswith('_') and suffix[1:].isdigit():
            paths.append((int(suffix[1:])+1, p))
    return [path for _, path in sorted(paths)]


def read_rows(paths, quality):
    for path in paths:
        with path.open(encoding='utf-8') as stream:
            for line_no, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line, parse_constant=lambda v: None)
                    if not isinstance(row, dict):
                        raise ValueError('record is not an object')
                    yield row
                except (ValueError, json.JSONDecodeError):
                    quality['invalid_json_lines'] += 1
                    if len(quality['invalid_line_examples']) < 10:
                        quality['invalid_line_examples'].append({'file': path.name, 'line': line_no})


def status_payload(payload):
    if isinstance(payload.get('parsed'), dict):
        return payload['parsed']
    return payload


class RunAnalysis:
    def __init__(self):
        self.message_counts, self.events, self.reasons = Counter(), Counter(), Counter()
        self.source_counts = {}
        self.stats = {key: Stats() for key in (
            'planning_duration_s', 'planning_cycle_s', 'controller_cycle_s',
            'steering_rate_abs_deg_s', 'command_acceleration_m_s2',
            'command_feedback_speed_error_m_s', 'reference_pair_receive_gap_s',
            'cloud_to_obstacle_observed_receive_gap_s',
            'localization_yaw_rate_rad_s', 'localization_source_dt_s')}
        self.evidence = {}
        self.epochs, self.consumed_epochs = {}, Counter()
        self.cloud_obstacle_pairs = {}
        self.last_cmd = self.ready = None
        self.previous_sign, self.sign_flips = 0, 0
        self.flip_window = deque()
        self.mode_changes = []
        self.heartbeat_gaps = self.heartbeat_duplicates = 0
        self.spin_sessions, self.active_spin = [], None
        self.first_t = self.last_t = None
        self.effective_params = {}
        self.writer_stats = {}
        self.last_sequence = None
        self.sequence_gaps = self.sequence_nonmonotonic = 0

    def observe(self, code, t, detail):
        item = self.evidence.setdefault(code, {'count': 0, 'examples': []})
        item['count'] += 1
        if len(item['examples']) < 20:
            item['examples'].append({'t': t, 'detail': detail})

    def timing(self, t):
        if t is not None:
            self.first_t = t if self.first_t is None else min(self.first_t, t)
            self.last_t = t if self.last_t is None else max(self.last_t, t)

    def planning(self, data, t):
        if not isinstance(data, dict):
            return
        reason = data.get('safety_reason') or data.get('reason')
        if reason:
            self.reasons[str(reason)] += 1
        clocks = data.get('clocks') or {}
        self.stats['planning_duration_s'].add(data.get('planning_duration_s', clocks.get('planning_duration_s')))
        self.stats['planning_cycle_s'].add(data.get('cycle_dt_s', clocks.get('cycle_dt_s')))
        if isinstance(data.get('effective_params'), dict):
            self.effective_params.update(data['effective_params'])

    def controller(self, data, t, clock_epoch=0):
        if not isinstance(data, dict):
            return
        telemetry = data.get('telemetry_json') or {}
        if not isinstance(telemetry, dict):
            telemetry = {}
        ref = epoch(data.get('ref_epoch_ns')) or epoch(telemetry.get('ref_epoch_ns'))
        pairs = telemetry.get('reference_pairs') or telemetry.get('pair') or {}
        ref = ref or epoch(pairs.get('last_consumed_epoch_ns'))
        ref = ref or epoch((telemetry.get('reference') or {}).get('consumed_epoch_ns'))
        if ref:
            self.consumed_epochs[(clock_epoch, ref)] += 1
        clocks = telemetry.get('clocks') or {}
        self.stats['controller_cycle_s'].add(clocks.get('cycle_dt_s', clocks.get('raw_cycle_dt_s', clocks.get('dt'))))
        if isinstance(telemetry.get('effective_params'), dict):
            self.effective_params['controller'] = telemetry['effective_params']
        if data.get('pose_fault_latched') in (1, 1.0, True):
            self.observe('localization_fault_reported', t, {'reason': data.get('reason')})

    def command(self, payload, t):
        mode, speed, angle = payload.get('turnmode'), number(payload.get('speed')), number(payload.get('angle'))
        prior = self.last_cmd
        heartbeat = payload.get('heartbeat')
        if prior and isinstance(heartbeat, int) and isinstance(prior.get('heartbeat'), int):
            increment = (heartbeat-prior['heartbeat']) % 2147483647
            self.heartbeat_duplicates += int(increment == 0)
            if 1 < increment < 1000000:
                self.heartbeat_gaps += increment-1
                self.observe('command_heartbeat_gap', t, {'from': prior['heartbeat'], 'to': heartbeat, 'missing': increment-1})
        if prior and mode != prior['mode']:
            self.mode_changes.append({'t': t, 'from': prior['mode'], 'to': mode})
            self.previous_sign = 0
            self.flip_window.clear()
        if mode == 4 and (not prior or prior['mode'] != 4):
            self.active_spin = {'request_s': t, 'left_censored': prior is None,
                                'ready_s': None, 'first_rotation_command_s': None,
                                'first_observed_yaw_motion_s': None,
                                'ack_request_s': None, 'ack_ready_s': None}
            self.spin_sessions.append(self.active_spin)
        elif mode == 0 and prior and prior['mode'] == 4 and self.active_spin:
            self.active_spin['ack_request_s'] = t
        if mode == 4 and speed is not None and abs(speed) > .06:
            if self.active_spin and self.active_spin['first_rotation_command_s'] is None:
                self.active_spin['first_rotation_command_s'] = t
                if not self.active_spin['left_censored'] and t-self.active_spin['request_s'] < 3.-.05:
                    self.observe('rotation_command_before_3s', t, dict(self.active_spin))
            # DDS receipt order can differ between recorder and controller. A
            # mismatch is evidence to cross-check with controller telemetry.
            if self.ready and t-self.ready['t'] <= .3 and self.ready['mode'] != 4:
                self.observe('rotation_command_ready_mismatch', t, {'ready': self.ready['mode'], 'speed': speed})
        if prior and mode == prior['mode'] == 0 and t is not None:
            dt = t-prior['t']
            if .0001 < dt <= 1.:
                if angle is not None and prior['angle'] is not None:
                    self.stats['steering_rate_abs_deg_s'].add(abs(angle-prior['angle'])/dt)
                if speed is not None and prior['speed'] is not None:
                    self.stats['command_acceleration_m_s2'].add((speed-prior['speed'])/dt)
        if mode == 0 and speed is not None and abs(speed) > .05 and angle is not None:
            sign = 1 if angle > .5 else -1 if angle < -.5 else 0
            if sign and self.previous_sign and sign != self.previous_sign:
                self.sign_flips += 1
                self.flip_window.append(t)
                while self.flip_window and t-self.flip_window[0] > 2.:
                    self.flip_window.popleft()
                if len(self.flip_window) >= 4:
                    self.observe('clustered_steering_sign_changes', t, {'changes_in_2s': len(self.flip_window)})
            if sign:
                self.previous_sign = sign
        self.last_cmd = {'t': t, 'mode': mode, 'speed': speed, 'angle': angle, 'heartbeat': heartbeat}

    def feedback(self, payload, t):
        speeds = payload.get('speeds') or payload.get('speed') or []
        if not isinstance(speeds, list):
            speeds = []
        values = [number(value) for value in speeds]
        if len(values) != 4 or any(value is None for value in values):
            self.observe('invalid_wheel_feedback', t, {'finite_wheel_count': sum(v is not None for v in values)})
            return
        if self.last_cmd and payload.get('turnmode') == self.last_cmd['mode'] == 0:
            # CarState has no source stamp: this is a receipt-time comparison,
            # not an estimate of the physical actuator response delay.
            if t-self.last_cmd['t'] <= .3 and self.last_cmd['speed'] is not None:
                self.stats['command_feedback_speed_error_m_s'].add(self.last_cmd['speed']-sum(values)/4.)
        if any(number(value) not in (None, 0.) for value in payload.get('errcode', [])):
            self.observe('chassis_error_reported', t, {'errcode': payload['errcode']})

    def message(self, row):
        key, payload, t = row.get('cache_key'), row.get('payload') or {}, number(row.get('t'))
        if not isinstance(payload, dict) or t is None:
            return
        self.timing(t)
        self.message_counts[str(key)] += 1
        seq = row.get('sequence')
        if isinstance(seq, int):
            if self.last_sequence is not None:
                self.sequence_gaps += max(0, seq-self.last_sequence-1)
                self.sequence_nonmonotonic += int(seq <= self.last_sequence)
            self.last_sequence = seq
        source = row.get('source') or {}
        clock_epoch = row.get('clock_epoch', 0)
        counts = self.source_counts.setdefault(str(key), Counter())
        counts[str(source.get('progress', 'unavailable'))] += 1
        for flag in ('future', 'stale', 'clock_jump', 'payload_invalid'):
            counts[flag] += int(bool(source.get(flag)))
        if key in ('command', 'traj'):
            stamp = epoch(payload.get('epoch_ns')) or epoch(row.get('source_stamp_ns'))
            if stamp:
                entry = self.epochs.setdefault((clock_epoch, stamp), {})
                first_arrival = key not in entry
                entry.setdefault(key, t)
                if first_arrival and len(entry) == 2:
                    self.stats['reference_pair_receive_gap_s'].add(abs(entry['command']-entry['traj']))
        elif key == 'car_cmd':
            self.command(payload, t)
        elif key == 'car_state':
            self.feedback(payload, t)
        elif key == 'mode_ready':
            mode = payload.get('data')
            self.ready = {'mode': mode, 't': t}
            if self.active_spin:
                if mode == 4 and self.active_spin['ready_s'] is None:
                    self.active_spin['ready_s'] = t
                if mode == 0 and self.active_spin['ack_request_s'] is not None and self.active_spin['ack_ready_s'] is None:
                    self.active_spin['ack_ready_s'] = t
        elif key == 'loc':
            derived = payload.get('derived') or {}
            if derived.get('valid'):
                rate = number(derived.get('yaw_rate'))
                self.stats['localization_yaw_rate_rad_s'].add(rate)
                self.stats['localization_source_dt_s'].add(derived.get('source_dt'))
                if self.active_spin and self.active_spin['ack_request_s'] is None and rate is not None and abs(rate) > .05:
                    if self.active_spin['first_observed_yaw_motion_s'] is None:
                        self.active_spin['first_observed_yaw_motion_s'] = t
        elif key in ('diag', 'rda_status'):
            self.planning(status_payload(payload), t)
        elif key == 'ctrl_status':
            self.controller(status_payload(payload), t, clock_epoch)
        if key in ('cloud', 'obstacles'):
            stamp = epoch(row.get('source_stamp_ns'))
            if stamp:
                entry = self.cloud_obstacle_pairs.setdefault((clock_epoch, stamp), {})
                first_arrival = key not in entry
                entry.setdefault(key, t)
                if first_arrival and len(entry) == 2:
                    self.stats['cloud_to_obstacle_observed_receive_gap_s'].add(entry['obstacles']-entry['cloud'])

    def snapshot(self, row, legacy=False):
        t = number(row.get('t'))
        self.timing(t)
        if isinstance(row.get('writer_stats'), dict):
            self.writer_stats = row['writer_stats']
        if legacy and t is not None:
            self.command({'turnmode': row.get('cc_turnmode'), 'speed': row.get('cc_speed'), 'angle': row.get('cc_angle')}, t)
            self.planning({'reason': row.get('reason'), 'planning_duration_s': row.get('diag_planning_duration_s'), 'cycle_dt_s': row.get('diag_cycle_dt_s')}, t)
        for key in ('loc_source_age', 'obstacle_source_age', 'cloud_source_age'):
            age = number(row.get(key))
            if age is not None and age > .5:
                self.observe('snapshot_source_age_over_0_5s', t, {'key': key, 'age_s': age})

    def result(self, message_stream_present):
        unpaired = []
        for (clock_epoch, stamp), pair in self.epochs.items():
            if len(pair) < 2 and self.last_t-max(pair.values()) > .5:
                if len(unpaired) < 20:
                    unpaired.append({'clock_epoch': clock_epoch, 'epoch_ns': stamp, 'arrivals': pair})
        missing_pairs = [stamp for stamp in self.consumed_epochs if set(self.epochs.get(stamp, {})) != {'command', 'traj'}]
        return {
            'duration_s': self.last_t-self.first_t if self.last_t is not None else 0.,
            'measurement_scope': 'message callback arrivals' if message_stream_present else 'legacy periodic snapshots; 50Hz commands may be aliased',
            'message_counts': dict(self.message_counts),
            'source_progress_counts': {key: dict(counts) for key, counts in self.source_counts.items()},
            'sequence_gaps': self.sequence_gaps, 'nonmonotonic_sequences': self.sequence_nonmonotonic,
            'command_heartbeat_missing_count': self.heartbeat_gaps,
            'command_heartbeat_duplicate_count': self.heartbeat_duplicates,
            'metrics': {key: stat.result() for key, stat in self.stats.items()},
            'stop_reason_counts': dict(self.reasons), 'event_counts': dict(self.events),
            'command_steering_sign_changes': self.sign_flips, 'command_mode_changes': self.mode_changes,
            'spin_sessions': self.spin_sessions,
            'reference_epochs': {'seen': len(self.epochs), 'paired': sum(len(v) == 2 for v in self.epochs.values()),
                                 'unpaired_older_than_0_5s_examples': unpaired,
                                 'controller_consumed_epochs_missing_recorder_pair': len(missing_pairs),
                                 'missing_consumed_pair_examples': [{'clock_epoch': ce, 'epoch_ns': ns} for ce, ns in missing_pairs[:20]]},
            'observations': self.evidence, 'runtime_telemetry_parameters': self.effective_params,
            'last_snapshot_writer_stats': self.writer_stats,
            'limits': [
                'No measured steering angle exists in current CarState; command angle is not actual steering.',
                'CarState has no source timestamp; wheel averaging and receipt-time errors do not identify physical response latency.',
                'DDS arrival order and recorder drops can explain unmatched references; controller consumption and writer health must be checked together.',
                'Command heartbeat gaps show missing observations, not where a message was lost or whether firmware applied it; heartbeat resets are not automatically loss.',
                'Cloud-to-obstacle matching uses equal source stamps; the receipt gap includes processing and transport, may be negative from DDS ordering, and is not pure perception CPU time.',
                'Sign changes may be a normal S-bend. Clusters are investigation clues, not proof of instability.',
                'Threshold flags use the current low-speed 3s/.3s/.5s profile; compare runtime/config provenance before treating them as violations.',
                'Point-cloud metadata does not reproduce segmentation, missed detections or collision candidates; use rosbag/raw cloud for those questions.',
            ],
        }


def load_object(path):
    if not path.exists():
        return None
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def analyze_run(run_dir):
    run_dir = Path(run_dir)
    if not run_dir.is_dir():
        raise ValueError(f'Log directory does not exist: {run_dir}')
    quality = {'invalid_json_lines': 0, 'invalid_line_examples': []}
    messages, snapshots, events = [segment_files(run_dir, stem) for stem in ('messages', 'snapshots', 'events')]
    if not messages and not snapshots:
        raise ValueError('No messages or snapshots JSONL found')
    analysis = RunAnalysis()
    message_rows = 0
    for row in read_rows(messages, quality):
        analysis.message(row)
        message_rows += 1
    for row in read_rows(snapshots, quality):
        analysis.snapshot(row, legacy=not message_rows)
    for row in read_rows(events, quality):
        analysis.events[str(row.get('event', 'unknown'))] += 1
    result = analysis.result(bool(message_rows))
    result.update(report_version=1, run_dir=str(run_dir.resolve()),
                  log_files={stem: [path.name for path in paths] for stem, paths in
                             (('messages', messages), ('snapshots', snapshots), ('events', events))},
                  log_quality=quality, parameters=load_object(run_dir/'params.json'),
                  writer_summary=load_object(run_dir/'writer_summary.json'))
    return result


def report_markdown(result):
    writer = result.get('writer_summary') or result.get('last_snapshot_writer_stats') or {}
    rows = ['# 实车日志取证摘要', '', f'日志目录：`{result["run_dir"]}`', '',
            f'观察时长：{result["duration_s"]:.3f} 秒。统计口径：{result["measurement_scope"]}。', '',
            '这些结果用于筛选问题时段；需要结合轨迹、障碍、状态机和底层反馈判断原因。', '',
            '| 记录完整性 | 值 |', '|---|---:|',
            f'| 无效JSON行 | {result["log_quality"]["invalid_json_lines"]} |',
            f'| 消息序号缺口 | {result["sequence_gaps"]} |',
            f'| 消息序号倒序 | {result["nonmonotonic_sequences"]} |', '',
            '## 时序与运动指标', '', '| 指标 | 样本数 | 均值 | p95 | 最大 |', '|---|---:|---:|---:|---:|']
    def fmt(value):
        return '缺失' if value is None else f'{value:.6g}'
    insertion = rows.index('## 时序与运动指标')
    rows[insertion:insertion] = [
        f'写入完整性：clean_close=`{writer.get("clean_close", "未知")}`，丢弃记录=`{writer.get("dropped", "未知")}`，写入错误=`{writer.get("write_errors", "未知")}`，待写队列=`{writer.get("queue_depth", "未知")}`。',
        '',
        '缺少正常关闭摘要、存在丢弃/写入错误或无效JSON时，先核对日志完整性，再解释车辆和输入故障。',
        '',
    ]
    for key, stat in result['metrics'].items():
        rows.append(f'| {key} | {stat["count"]} | {fmt(stat["mean"])} | {fmt(stat["p95"])} | {fmt(stat["max"])} |')
    rows += ['', 'p95超过10000个样本时采用最后10000个样本；均值与最大值覆盖全部有效样本。', '',
             f'命令转角反号次数：{result["command_steering_sign_changes"]}；模式请求变化次数：{len(result["command_mode_changes"])}。', '',
             '## 明确上报的停止原因', '']
    rows += [f'- `{key}`：{count} 条诊断/状态消息。' for key, count in result['stop_reason_counts'].items()] or ['未记录到原因字段。']
    rows += ['', '## 自转与机械切换时序', '', '| 请求自转 | ready=4 | 首次旋转指令 | 首次观测yaw运动 | 请求ACK | ready=0 |', '|---:|---:|---:|---:|---:|---:|']
    for session in result['spin_sessions']:
        rows.append('| '+' | '.join(fmt(session[key]) for key in ('request_s', 'ready_s', 'first_rotation_command_s', 'first_observed_yaw_motion_s', 'ack_request_s', 'ack_ready_s'))+' |')
    rows += ['', '录制开始时已在自转的会话为left_censored，不能用首条消息代替实际模式请求时间。', '', '## 待核对观察', '']
    for code, item in result['observations'].items():
        rows.append(f'- `{code}`：{item["count"]} 次；示例时刻 '+', '.join(str(ex['t']) for ex in item['examples'][:5])+' 秒。')
    if not result['observations']:
        rows.append('未触发当前阈值观察；这不表示全部信号完整或车辆避障已可靠。')
    refs = result['reference_epochs']
    rows += ['', f'参考epoch共{refs["seen"]}个，完整配对{refs["paired"]}个；控制器消费而记录端缺少完整配对的epoch数：{refs["controller_consumed_epochs_missing_recorder_pair"]}。', '', '## 分析边界', '']
    rows += ['- '+limit for limit in result['limits']]
    return '\n'.join(rows)+'\n'


def main(args=None):
    parser = argparse.ArgumentParser(description='Summarize vehicle diagnostic JSONL without ROS')
    parser.add_argument('run_dir', type=Path, help='Recorder run_* directory')
    parser.add_argument('--out-dir', type=Path, help='Default: <run_dir>/analysis')
    parsed = parser.parse_args(args)
    try:
        result = analyze_run(parsed.run_dir)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    output = parsed.out_dir or parsed.run_dir/'analysis'
    output.mkdir(parents=True, exist_ok=True)
    (output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (output/'summary.md').write_text(report_markdown(result), encoding='utf-8')
    print(str((output/'summary.md').resolve()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
