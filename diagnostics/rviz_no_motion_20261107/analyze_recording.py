"""Read-only JSONL + SQLite/CDR evidence check; no ROS or vehicle output.

TF replay composes the recorded 3D transforms at their latest common time.
Interpolation is an offline approximation, not a tf2 middleware test.
"""
import argparse
import bisect
import collections
import json
import math
from pathlib import Path
import sqlite3
import struct


class CDR:
    def __init__(self, data):
        if data[:4] != b'\x00\x01\x00\x00':
            raise ValueError('Expected little-endian CDR encapsulation')
        self.data, self.offset = data, 4

    def number(self, fmt, alignment):
        self.offset = 4 + ((self.offset-4+alignment-1)//alignment)*alignment
        values = struct.unpack_from('<'+fmt, self.data, self.offset)
        self.offset += struct.calcsize(fmt)
        return values[0] if len(values) == 1 else values

    def string(self):
        length = self.number('I', 4)
        text = self.data[self.offset:self.offset+length-1].decode('utf-8')
        self.offset += length
        return text

    def transforms(self):
        result = []
        for _ in range(self.number('I', 4)):
            stamp = self.number('i', 4)+self.number('I', 4)*1e-9
            parent, child = self.string(), self.string()
            result.append((parent, child, stamp, self.number('3d', 8), self.number('4d', 8)))
        if self.offset != len(self.data):
            raise ValueError('TF CDR size mismatch')
        return result


def quantiles(values):
    values = sorted(values)
    return {str(q): round(values[round((len(values)-1)*q)], 6)
            for q in (0, .5, .9, .99, 1)} if values else {}


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def rotate(q, v):
    return multiply(multiply(q, (*v, 0.)), (-q[0], -q[1], -q[2], q[3]))[:3]


def at(history, stamp):
    i = bisect.bisect_right([r[0] for r in history], stamp)-1
    if i < 0:
        return None
    _, p, q = history[i]
    if i+1 == len(history) or history[i][0] == stamp:
        return p, q
    after, pp, qq = history[i+1]
    f = (stamp-history[i][0])/(after-history[i][0])
    # Use the shortest quaternion arc (SLERP; normalized linear near zero).
    dot = sum(x*y for x, y in zip(q, qq))
    if dot < 0:
        qq = tuple(-v for v in qq)
        dot = -dot
    if dot > .9995:
        weights = (1-f, f)
    else:
        theta = math.acos(max(-1., min(1., dot)))
        weights = (math.sin((1-f)*theta)/math.sin(theta), math.sin(f*theta)/math.sin(theta))
    q = tuple(weights[0]*x+weights[1]*y for x, y in zip(q, qq))
    norm = math.sqrt(sum(v*v for v in q))
    return tuple((1-f)*x+f*y for x, y in zip(p, pp)), tuple(v/norm for v in q)


def latest_pose(histories):
    a = histories.get(('map', 'odom'), [])
    b = histories.get(('odom', 'body'), [])
    if not a or not b:
        return None
    stamp = min(a[-1][0], b[-1][0])
    first, second = at(a, stamp), at(b, stamp)
    if first is None or second is None:
        return None
    p, q = first
    pp, qq = second
    delta = rotate(q, pp)
    position = tuple(x+y for x, y in zip(p, delta))
    x, y, z, w = multiply(q, qq)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return stamp, position[0], position[1], yaw


def analyze(root):
    root = Path(root)
    topics = collections.defaultdict(list)
    for path in sorted(root.glob('messages_*.jsonl')):
        for line in path.open():
            row = json.loads(line)
            payload = row['payload']
            if isinstance(payload, dict) and isinstance(payload.get('data'), str):
                try:
                    payload = json.loads(payload['data'])
                except ValueError:
                    pass
            row['decoded'] = payload
            topics[row['topic']].append(row)
    localization = topics['/localization']
    groups = collections.defaultdict(list)
    for row in localization:
        groups[row['source_stamp_ns']].append(row)
    changing_groups = sum(len({tuple(r['decoded']['pose']['pose']['position'].values()) for r in rows}) > 1
                          for rows in groups.values())
    goals = topics['/move_base_simple/goal']
    first_goal = goals[0]['t']
    report = dict(
        topic_counts={k: len(v) for k, v in topics.items()},
        goals=[dict(t=r['t'], pose=r['decoded']['pose']) for r in goals],
        localization=dict(messages=len(localization), unique_source_stamps=len(groups),
            repeated_stamp_groups_with_changed_position=changing_groups,
            callback_rate_hz=(len(localization)-1)/(localization[-1]['t']-localization[0]['t']),
            receive_gap_s=quantiles([b['t']-a['t'] for a, b in zip(localization, localization[1:])]),
            source_age_s=quantiles([(r['ros_receive_ns']-r['source_stamp_ns'])*1e-9 for r in localization])),
        perception_reasons=dict(collections.Counter(r['decoded']['reason'] for r in topics['/rda/perception_diag'])),
        planning_reasons_after_first_goal=dict(collections.Counter(r['decoded']['reason'] for r in topics['/rda/diag'] if r['t'] >= first_goal)),
        control_stop_reasons=dict(collections.Counter(r['decoded']['safety_reason'] for r in topics['/rda/control_diag'])),
        nonzero_car_commands=sum(abs(r['decoded']['speed']) > 1e-6 for r in topics['/car_cmd']),
        nonempty_references=sum(bool(r['decoded']['reference']) for r in topics['/rda/plan']),
        budget_events=[dict(t=r['t'], telemetry=r['decoded'].get('planner')) for r in topics['/rda/diag'] if r['decoded']['reason']=='planning_budget'],
        writer_summary=json.loads((root/'writer_summary.json').read_text()))
    raw_counts = collections.Counter()
    tf_ages = collections.defaultdict(list)
    histories = collections.defaultdict(list)
    perception_ages, control_ages, replay_poses = [], [], []
    query_failures = collections.Counter()
    for path in sorted((root/'raw_bag').glob('*.db3')):
        with sqlite3.connect('file:'+str(path)+'?mode=ro', uri=True) as db:
            names = dict(db.execute('SELECT id,name FROM topics'))
            for ident, count in db.execute('SELECT topic_id,count(*) FROM messages GROUP BY topic_id'):
                raw_counts[names[ident]] += count
            wanted = [i for i, n in names.items() if n in ('/tf','/rda/perception_diag','/rda/control_diag')]
            query = 'SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ('+','.join('?' for _ in wanted)+') ORDER BY timestamp'
            for ident, timestamp, data in db.execute(query, wanted):
                name, now = names[ident], timestamp*1e-9
                if name == '/tf':
                    for parent, child, stamp, p, q in CDR(data).transforms():
                        key = (parent, child)
                        tf_ages[key].append(now-stamp)
                        h = histories[key]
                        if h and stamp <= h[-1][0]:
                            continue
                        h.append((stamp, p, q))
                        if len(h)>150:
                            del h[:-150]
                else:
                    pose = latest_pose(histories)
                    if pose is None:
                        query_failures[name] += 1
                        continue
                    age = now-pose[0]
                    (perception_ages if name == '/rda/perception_diag' else control_ages).append(age)
                    if name == '/rda/control_diag':
                        replay_poses.append((now, *pose))
    report['raw_bag_counts'] = dict(raw_counts)
    report['tf_source_age_s'] = {a+'->'+b: dict(count=len(v), quantiles=quantiles(v)) for (a,b),v in tf_ages.items()}
    report['tf_replay'] = dict(
        note='Offline 3D chain composition at latest common source time; not a tf2/DDS test or vehicle test.',
        query_failures=dict(query_failures),
        perception_queries=len(perception_ages),
        perception_pose_age_s=quantiles(perception_ages),
        perception_age_gate_passes=sum(0<=v<=.5 for v in perception_ages),
        control_queries=len(control_ages),
        control_pose_age_s=quantiles(control_ages),
        control_age_gate_passes=sum(0<=v<=.35 for v in control_ages))
    return report, replay_poses


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('recording')
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    report, _ = analyze(args.recording)
    result = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    if args.output:
        args.output.write_text(result)
    print(json.dumps({k: v for k, v in report.items() if k not in ('topic_counts','raw_bag_counts','goals','writer_summary')}, ensure_ascii=False, indent=2))
