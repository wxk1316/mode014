#!/usr/bin/env python3
"""Check ONNX and extract pure tensor HARP weights, without training/quantization."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import onnx
import torch


def prepare(root):
    report = {}
    for path in (root/'models/centerpoint_base').glob('*.onnx'):
        model = onnx.load(path)
        onnx.checker.check_model(model)
        def spec(v):
            return dict(name=v.name,shape=[d.dim_value or d.dim_param for d in v.type.tensor_type.shape.dim])
        report[path.name] = dict(inputs=[spec(v) for v in model.graph.input],
                                outputs=[spec(v) for v in model.graph.output])
    if len(report)!=2:raise ValueError('Both CenterPoint base ONNX files are required')
    path = root/'models/harpnext_nuscenes'
    # The official checkpoint also contains NumPy training statistics. Permit
    # precisely the built-in NumPy scalar/dtype constructs for this extraction.
    core = np._core if hasattr(np,'_core') else np.core
    with torch.serialization.safe_globals([core.multiarray.scalar,np.dtype,type(np.dtype(np.float64))]):
        original = torch.load(path/'ckpt_best.pth',map_location='cpu',weights_only=True)
    state = original['net']
    if not all(isinstance(v,torch.Tensor) for v in state.values()):
        raise ValueError('Expected a tensor-only network state')
    output = path/'weights.pth'
    torch.save(state,output)
    loaded = torch.load(output,map_location='cpu',weights_only=True)
    if not all(torch.equal(state[k],loaded[k]) for k in state):
        raise ValueError('Tensor extraction changed pretrained parameters')
    report['harpnext'] = dict(tensor_count=len(state),parameters_unchanged=True,
                              bytes=output.stat().st_size,sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    (root/'models/input_contracts.json').write_text(json.dumps(report,indent=2))
    manifest_path = root/'models/manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['normalized_harp_weights'] = report['harpnext']
    manifest_path.write_text(json.dumps(manifest,indent=2))
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-root',type=Path,default=Path(__file__).resolve().parents[1])
    prepare(parser.parse_args().package_root)
