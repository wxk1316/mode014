"""Inference-only scatter adapter; prefer torch-scatter when installed.

The native Torch fallback implements exactly the dim=0 reductions used by the
vendored network. Its performance must be measured separately on the Orin.
"""
import torch
try:
    import torch_scatter as _accelerated
except ImportError:
    _accelerated = None

BACKEND = 'torch_scatter' if _accelerated is not None else 'torch_native'


def scatter_mean(src, index, dim=0):
    if _accelerated is not None:
        return _accelerated.scatter_mean(src, index, dim=dim)
    if dim != 0 or src.ndim != 2 or index.ndim != 1:
        raise ValueError('unsupported HARP scatter_mean input')
    size = int(index.max())+1 if index.numel() else 0
    out = src.new_zeros((size, src.shape[1]))
    out.index_add_(0, index, src)
    count = src.new_zeros(size)
    count.index_add_(0, index, src.new_ones(len(index)))
    return out/count.clamp_min(1)[:, None]


def scatter_max(src, index, dim=0, out=None):
    if _accelerated is not None:
        return _accelerated.scatter_max(src, index, dim=dim, out=out)
    if dim != 0 or src.ndim != 2 or index.ndim != 1:
        raise ValueError('unsupported HARP scatter_max input')
    if out is None:
        size = int(index.max())+1 if index.numel() else 0
        out = src.new_full((size, src.shape[1]), -torch.inf)
    out.scatter_reduce_(0, index[:, None].expand_as(src), src, reduce='amax', include_self=True)
    out.masked_fill_(torch.isinf(out), 0.)
    # HARP only consumes values ([0]); argmax indices are unused in inference.
    return out, None
