"""Detections JSONL -> tracking JSONL. Synthetic smoke is explicitly labelled."""
import argparse
import json
from pathlib import Path

from .types import Detection
from .tracker import MultiObjectTracker


def run(args):
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.smoke:
        frames = [{'stamp': 1.+i*.1, 'detections': [dict(x=2.+.05*i, y=1., label='PEDESTRIAN')]}
                  for i in range(30)]
    else:
        if args.input is None:
            raise ValueError('--input is required without --smoke')
        frames = (json.loads(line) for line in args.input.open() if line.strip())
    tracker = MultiObjectTracker()
    with args.output.open('w') as stream:
        for frame in frames:
            detections = [Detection(**d).validate() for d in frame['detections']]
            tracks = tracker.step(detections, float(frame['stamp']))
            stream.write(json.dumps(dict(stamp=frame['stamp'], frame_id='map',
                                         tracks=tracks, synthetic_fixture=args.smoke), allow_nan=False) + '\n')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--smoke', action='store_true')
    return run(parser.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
