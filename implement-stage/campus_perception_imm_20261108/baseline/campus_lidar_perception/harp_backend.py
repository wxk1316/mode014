"""Load the real 32x480 HARP-NeXt checkpoint and infer row-aligned labels."""
from pathlib import Path
import numpy as np
import yaml


CLASS_NAMES = ['barrier', 'bicycle', 'bus', 'car', 'construction_vehicle', 'motorcycle',
               'pedestrian', 'traffic_cone', 'trailer', 'truck', 'driveable_surface',
               'other_flat', 'sidewalk', 'terrain', 'manmade', 'vegetation']


def project(points, height=32, width=480, fov_up=10., fov_down=-30.):
    depth = np.linalg.norm(points[:, :3], axis=1)
    pitch = np.arcsin(np.clip(points[:, 2]/np.maximum(depth, 1e-6), -1., 1.))
    up, down = np.deg2rad(fov_up), np.deg2rad(fov_down)
    valid = (depth > .05) & (pitch >= down) & (pitch <= up)
    x = np.floor(.5*(-np.arctan2(points[:, 1], points[:, 0])/np.pi+1.)*width).astype(np.int64)
    y = np.floor((1.-(pitch+abs(down))/(abs(down)+abs(up)))*height).astype(np.int64)
    return np.column_stack((np.zeros(len(points), np.int64), np.clip(y,0,height-1),
                            np.clip(x,0,width-1))), valid


def interpolate(points):
    """Author's one-pixel horizontal interpolation at W1920, vectorised.

    Appends synthetic points after measured points so only measured-point labels
    are returned. Unlike upstream proj_idx>0, index zero is a valid measurement.
    No interpolation spans the azimuth seam or the field-of-view boundary.
    """
    coors, valid = project(points, width=1920)
    if not valid.all():
        raise ValueError('interpolate expects field-of-view filtered points')
    depth = np.linalg.norm(points[:, :3], axis=1)
    # Stable nearest-return selection for duplicate pixels.
    pixel = coors[:, 1]*1920+coors[:, 2]
    order = np.lexsort((depth, pixel))
    first = np.r_[True, pixel[order][1:] != pixel[order][:-1]]
    selected = order[first]
    image = np.full((32,1920,4), -1., np.float32)
    mask = np.zeros((32,1920), bool)
    image[coors[selected,1], coors[selected,2]] = points[selected]
    mask[coors[selected,1], coors[selected,2]] = True
    fill = ~mask[:,1:-1] & mask[:,:-2] & mask[:,2:]
    extra = ((image[:,:-2]+image[:,2:])*.5)[fill]
    return np.ascontiguousarray(np.concatenate((points, extra), axis=0))


class HarpNeXtBackend:
    def __init__(self, model_dir, device='cuda', fp16=True):
        import torch
        from .vendor.harpnext.core.network import Network
        from .scatter_ops import BACKEND
        self.torch, self.device = torch, torch.device(device)
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('HARP-NeXt requires the Jetson CUDA PyTorch installation')
        path = Path(model_dir)
        config = yaml.safe_load((path/'net.yaml').read_text())
        if config['range_proj'] != dict(range_H=32,range_W=480,fov_up=10.,fov_down=-30.):
            raise ValueError('checkpoint requires original nuScenes 32x480 projection')
        self.model = Network('harpnext', config).build_network()
        if not (path/'weights.pth').is_file():
            raise RuntimeError('Run scripts/prepare_models.py to extract the pure tensor weights')
        state = torch.load(path/'weights.pth', map_location='cpu', weights_only=True)
        self.model.load_state_dict(state, strict=True)
        self.model.to(self.device).eval()
        self.fp16 = fp16 and self.device.type == 'cuda'
        self.scatter_backend = BACKEND
        self.parameter_count = sum(p.numel() for p in self.model.parameters())

    def predict(self, points):
        torch = self.torch
        _, valid = project(points)
        indices = np.flatnonzero(valid)
        labels = np.full(len(points), -1, np.int16)
        confidence = np.zeros(len(points), np.float32)
        if len(indices) < 2:
            return labels, confidence
        measured = np.ascontiguousarray(points[indices], dtype=np.float32)
        augmented = interpolate(measured)
        coors, _ = project(augmented)
        inputs = dict(voxels=dict(voxels=torch.as_tensor(augmented, device=self.device),
                                  coors=torch.as_tensor(coors, device=self.device)))
        with torch.inference_mode(), torch.autocast(device_type=self.device.type,
                                                   dtype=torch.float16, enabled=self.fp16):
            logits = self.model(inputs, training=False)['seg_logits'][:len(measured)]
            probs = logits.float().softmax(dim=1)
            scores, classes = probs.max(dim=1)
        labels[indices] = classes.cpu().numpy().astype(np.int16)
        confidence[indices] = scores.cpu().numpy()
        return labels, confidence
