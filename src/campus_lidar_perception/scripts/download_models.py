#!/usr/bin/env python3
"""Fetch the selected, versioned pretrained artifacts; never run training."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time
import urllib.request

AW_REV = '494c8171def40bd36cc2feb323e0a5acbfab132b'
HARP_WEIGHT_REV = 'c2cd3ce5940be23bfbc2d1cf22131fd2fd3dc149'
HARP_CODE_REV = '69c52ddc54e1fefaa7e64ac8e3fffea33e814eda'
HARP_CODE = ['core/network.py', 'core/harpnext_core/encoder/features_encoder.py',
             'core/harpnext_core/backbone/harpnext_backbone.py',
             'core/harpnext_core/decode_heads/harpnext_head.py',
             'core/harpnext_core/decode_heads/aux_head.py',
             'core/harpnext_core/segmentor/harpnext.py']


def artifacts(root):
    cp = 'https://huggingface.co/AutowareFoundation/lidar_centerpoint/resolve/'+AW_REV+'/base/'
    for name in ('pts_voxel_encoder.onnx', 'pts_backbone_neck_head.onnx',
                 'ml_package.param.yaml', 'detection_class_remapper.param.yaml'):
        yield root/'models/centerpoint_base'/name, cp+name, False
    yield (root/'models/harpnext_nuscenes/ckpt_best.pth',
           'https://huggingface.co/samir-abh/HARP-NeXt/resolve/'+HARP_WEIGHT_REV+
           '/harpnext-nuscenes-32x480/ckpt_best.pth', False)
    raw = 'https://raw.githubusercontent.com/SamirAbouHaidar/HARP-NeXt/'+HARP_CODE_REV+'/'
    yield root/'models/harpnext_nuscenes/net.yaml', raw+'configs/net/harpnext-nuscenes.yaml', False
    yield root/'models/harpnext_nuscenes/classes.yaml', raw+'datasets/nuscenes/nuscenes.yaml', False
    for name in HARP_CODE+['LICENSE']:
        yield root/'campus_lidar_perception/vendor/harpnext'/name, raw+name, name.endswith('.py')


def fetch(entry):
    path, url, adapt = entry
    path.parent.mkdir(parents=True, exist_ok=True)
    raw_path = path.with_suffix(path.suffix+'.upstream') if adapt else path
    if not raw_path.exists():
        partial = raw_path.with_suffix(raw_path.suffix+'.part')
        for attempt in range(4):
            try:
                # Resume only when the server actually returns a matching range.
                start = partial.stat().st_size if partial.exists() else 0
                request = urllib.request.Request(url, headers={'Range':f'bytes={start}-'} if start else {})
                with urllib.request.urlopen(request, timeout=45) as response:
                    append = start and response.status == 206
                    length = int(response.headers.get('Content-Length', '0'))
                    expected = (start if append else 0)+length
                    with partial.open('ab' if append else 'wb') as stream:
                        while chunk := response.read(1024*256):
                            stream.write(chunk)
                    if length and partial.stat().st_size != expected:
                        raise IOError('incomplete download '+url)
                partial.replace(raw_path)
                break
            except Exception:
                if attempt == 3:
                    raise
                time.sleep(1.)
    if adapt:
        source = raw_path.read_text()
        source = source.replace('from core.', 'from campus_lidar_perception.vendor.harpnext.core.')
        source = source.replace('import torch_scatter', 'from campus_lidar_perception import scatter_ops as torch_scatter')
        path.write_text(source)
        # Keep the original alongside the adaptation for review and provenance.
        for parent in path.parents:
            if parent.name == 'campus_lidar_perception':
                break
            (parent/'__init__.py').touch(exist_ok=True)
    record = dict(path=str(path), url=url, bytes=path.stat().st_size,
                  sha256=hashlib.sha256(path.read_bytes()).hexdigest(), import_adaptation=adapt)
    print('Fetched', path.name, record['bytes'], flush=True)
    return record


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package-root', type=Path, default=Path(__file__).resolve().parents[1])
    args = p.parse_args()
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(fetch, artifacts(args.package_root)))
    for r in results:
        r['path'] = str(Path(r['path']).relative_to(args.package_root))
    manifest = dict(centerpoint_revision=AW_REV, harp_weight_revision=HARP_WEIGHT_REV,
                    harp_code_revision=HARP_CODE_REV, files=results)
    (args.package_root/'models/manifest.json').write_text(json.dumps(manifest, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
