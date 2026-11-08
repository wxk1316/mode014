#!/usr/bin/env python3
"""Real GPU startup/inference and CUDA feature parity, without accuracy claims."""
import argparse
import json
from pathlib import Path
import time
import numpy as np

from campus_lidar_perception.centerpoint_backend import CenterPointBackend
from campus_lidar_perception.centerpoint_reference import pillar_features
from campus_lidar_perception.harp_backend import HarpNeXtBackend
from campus_lidar_perception.geometry import PatchworkGround


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir',type=Path,required=True)
    parser.add_argument('--engine-dir',type=Path,default=Path.home()/'.cache/campus_lidar_perception/engines')
    parser.add_argument('--library',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=dict(test_kind='synthetic_gpu_contract_smoke',accuracy_evaluated=False)
    start=time.monotonic()
    cp=CenterPointBackend(args.model_dir/'centerpoint_base',args.engine_dir,args.library)
    report['engine_startup_seconds']=time.monotonic()-start
    xyzt=np.array([[.01,.02,-.3,0.],[.04,.03,-.2,.1],[5.,2.,-.4,0.]],np.float32)
    try:
        boxes,diag=cp.predict(xyzt)
        actual,actual_cells=cp.debug_features()
        reference,cells=pillar_features(xyzt)
        order=np.argsort(actual_cells)
        np.testing.assert_array_equal(actual_cells[order],cells)
        np.testing.assert_allclose(actual[order],reference,rtol=1e-4,atol=2e-5)
        report.update(cuda_feature_parity=True,centerpoint=diag,synthetic_boxes=len(boxes))
    finally:cp.close()
    harp=HarpNeXtBackend(args.model_dir/'harpnext_nuscenes')
    labels,scores=harp.predict(np.array([[5.,0.,0.,60.],[5.,.2,-.2,80.],[-5.,1.,-.3,30.]],np.float32))
    if not np.isfinite(scores).all() or not np.all((labels>=0)&(labels<16)):
        raise RuntimeError('HARP-NeXt GPU output invalid')
    report['harp_gpu_forward']=True
    report['scatter_backend']=harp.scatter_backend
    PatchworkGround(.36)
    report['patchwork_binding']=True
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
