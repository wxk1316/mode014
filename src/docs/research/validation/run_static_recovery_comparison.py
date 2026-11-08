"""Matched-horizon baseline and mature static recovery, using the root harness.

Preserves earlier 35/60-second failures. No production source is edited and no
ROS/DDS/UDP is started. Baseline is extracted from the saved source zip.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

ROOT=Path(__file__).resolve().parents[3]
CASES=['multi_blocks','u_shape','thick_u','narrow_u']


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--worker',action='store_true')
    parser.add_argument('--source',type=Path);parser.add_argument('--output',type=Path)
    parser.add_argument('--duration',type=float,default=90.);parser.add_argument('--label',default='integrated')
    args=parser.parse_args()
    if args.worker:
        spec=importlib.util.spec_from_file_location('root_comparison',ROOT/'docs/research/validation/run_comparison.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        for case in CASES:module.SCENARIOS[case]['duration']=args.duration
        module.worker(args.source,args.output,args.label,CASES)
        return
    output=ROOT/f'docs/research/validation/static_recovery_{int(args.duration)}s'
    output.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rda-static-baseline-') as directory:
        baseline=Path(directory)
        with zipfile.ZipFile(ROOT/'docs/research/baseline_sources.zip') as archive:
            for name in archive.namelist():
                if name.startswith(('rda_dwa/','rda_dwa_real/','docs/validation/')) and not name.endswith(('.json','.jsonl')):
                    archive.extract(name,baseline)
        for label,source in [('integrated',ROOT),('baseline',baseline)]:
            command=[sys.executable,str(Path(__file__).resolve()),'--worker','--source',str(source),
                     '--output',str(output/label),'--duration',str(args.duration),'--label',label]
            subprocess.run(command,check=True)
    report={'scope':'same root bounded actuator and synthetic observation oracle; same duration per case; no ROS/DDS/UDP',
            'duration_s':args.duration,'cases':CASES,
            'variants':{label:json.loads((output/label/'results.json').read_text())for label in('baseline','integrated')}}
    (output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
