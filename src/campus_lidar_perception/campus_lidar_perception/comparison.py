"""Summarise recorded throughput/latency/grid coverage without ground-truth claims."""
import argparse
import json
from pathlib import Path
import numpy as np
from collections import Counter


def summarise(path):
    path=Path(path)
    rows=[json.loads(line) for line in (path/'events.jsonl').open() if line.strip()]
    tracks=[r for r in rows if r['kind']=='tracks']
    grids=[r['message'] for r in rows if r['kind']=='grid']
    semantic=[r['message'] for r in rows if r['kind']=='diagnostic' and r['message'].get('reason')=='semantic_ok']
    metadata=json.loads((path/'metadata.json').read_text())
    out=dict(profile=metadata['profile'],tracking_filter=metadata.get('tracking_filter','cv'),frames=len(tracks),accuracy_evaluated=False)
    out['latency_population']='Received successful tracks messages, including planner-suppressed frames; failures are excluded.'
    out['diagnostic_reason_counts']=dict(Counter(r['message'].get('reason','unknown') for r in rows if r['kind']=='diagnostic'))
    out['semantic_frames']=len(semantic)
    for key in ('semantic_prediction_s','semantic_processing_s'):
        values=[r[key]*1000 for r in semantic if key in r]
        if values:out[key.replace('_s','_ms')]=dict(zip(('p50','p95','p99'),map(float,np.percentile(values,[50,95,99]))))
    if not tracks:return out
    duration=tracks[-1]['received_monotonic']-tracks[0]['received_monotonic']
    out['received_output_hz']=(len(tracks)-1)/duration if duration>0 else None
    out['received_tracks_hz']=out['received_output_hz']
    published=[r for r in tracks if r['message'].get('obstacle_published',False)]
    out['planner_publish_attempt_frames']=len(published)
    for key in ('duration_s','inference_s','geometry_s','tracking_s','source_age_s'):
        values=[r['message'][key]*1000 for r in tracks if key in r['message']]
        if values:out[key.replace('_s','_ms')]=dict(zip(('p50','p95','p99'),map(float,np.percentile(values,[50,95,99]))))
    out['mean_objects']=float(np.mean([len(r['message']['tracks']) for r in tracks]))
    out['semantic_available_fraction']=float(np.mean([r['message'].get('semantic_available',False) for r in tracks]))
    out['mean_semantic_matched_fraction']=float(np.mean([r['message'].get('semantic_matched_fraction',0.) for r in tracks]))
    if grids:
        out['mean_grid_cells']={k:float(np.mean([g[k] for g in grids])) for k in ('free','occupied','unknown')}
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs',nargs='+',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    report=dict(runs=[summarise(path) for path in args.runs],
                interpretation='Coverage is not accuracy. Use labelled campus replay to assess recall, false positives and false-free drivable cells.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(json.dumps(report,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
