"""Read recorder schema2 without ROS, report missing data and recorder loss."""
import pathlib,json,argparse,collections

def inspect(root):
 root=pathlib.Path(root);counts=collections.Counter();bad=0;repeated=out_of_order=0;last_global=0;sequence_gaps=0;topics={}
 for path in sorted(root.glob('messages_*.jsonl')):
  for line in path.open():
   try:row=json.loads(line)
   except ValueError:bad+=1;continue
   counts[row['topic']]+=1
   if last_global and row['global_seq']>last_global+1:sequence_gaps+=row['global_seq']-last_global-1
   last_global=row['global_seq'];repeated+=bool(row.get('source_repeated'));out_of_order+=bool(row.get('source_out_of_order'))
   topics.setdefault(row['topic'],[]).append(row['t'])
 rates={k:((len(v)-1)/(v[-1]-v[0]) if len(v)>1 and v[-1]>v[0] else None) for k,v in topics.items()}
 def final_jsonl(name):
  path=root/name
  if not path.exists():return None
  last=None
  for line in path.open():
   try:last=json.loads(line)
   except ValueError:pass
  return last
 health=final_jsonl('recorder_health.jsonl');summary=None
 if (root/'writer_summary.json').exists():summary=json.loads((root/'writer_summary.json').read_text())
 return dict(run=str(root),topic_counts=dict(counts),observed_callback_rate_hz=rates,malformed_lines=bad,global_sequence_gaps=sequence_gaps,repeated_source_stamps=repeated,out_of_order_source_stamps=out_of_order,writer_summary=summary,last_health=health,raw_bag_metadata_exists=(root/'raw_bag/metadata.yaml').exists(),parameters_service_file_exists=(root/'runtime_parameters.jsonl').exists())
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('run');args=ap.parse_args();print(json.dumps(inspect(args.run),ensure_ascii=False,indent=2))
