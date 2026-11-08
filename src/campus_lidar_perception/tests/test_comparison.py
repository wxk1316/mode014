import json
from campus_lidar_perception.comparison import summarise


def test_summary_does_not_claim_accuracy(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'profile':'centerpoint_geometry'}))
    frames=[dict(kind='tracks',received_monotonic=i*.1,message=dict(duration_s=.02,
                 inference_s=.01,source_age_s=.05,tracks=[])) for i in range(10)]
    (tmp_path/'events.jsonl').write_text('\n'.join(json.dumps(r) for r in frames))
    result=summarise(tmp_path)
    assert result['received_output_hz']==10. and not result['accuracy_evaluated']
    assert result['duration_ms']['p95']==20.


def test_successful_semantic_latency_is_recorded(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'profile':'centerpoint_harpnext'}))
    event=dict(kind='diagnostic',message=dict(reason='semantic_ok',semantic_prediction_s=.071,semantic_processing_s=.075))
    (tmp_path/'events.jsonl').write_text(json.dumps(event))
    result=summarise(tmp_path)
    assert result['semantic_frames']==1
    assert result['semantic_prediction_ms']['p50']==71.
    assert result['semantic_processing_ms']['p95']==75.
