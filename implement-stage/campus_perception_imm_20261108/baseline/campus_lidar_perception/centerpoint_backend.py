"""ctypes facade for the standalone native TensorRT10/CUDA base detector."""
import ctypes as ct
from pathlib import Path
import numpy as np
import math

from .centerpoint_reference import CLASSES, nms
from .types import Detection
from .coordinates import wrap


class CenterPointBackend:
    def __init__(self, model_dir, engine_dir, library=None, thresholds=None, max_range=30.):
        path = Path(library) if library else Path(__file__).parent/'lib/libcampus_centerpoint.so'
        if not path.is_file():
            raise RuntimeError('Native CenterPoint library missing; run scripts/build_orin.sh: '+str(path))
        self.lib = ct.CDLL(str(path))
        float_ptr = ct.POINTER(ct.c_float)
        self.lib.campus_cp_api_version.restype = ct.c_int
        self.lib.campus_cp_last_error.restype = ct.c_char_p
        self.lib.campus_cp_create.argtypes = [ct.c_char_p,ct.c_char_p]
        self.lib.campus_cp_create.restype = ct.c_void_p
        self.lib.campus_cp_run.argtypes = [ct.c_void_p,float_ptr,ct.c_int,float_ptr,float_ptr,ct.c_int]
        self.lib.campus_cp_run.restype = ct.c_int
        self.lib.campus_cp_pillar_count.argtypes = [ct.c_void_p]
        self.lib.campus_cp_pillar_count.restype = ct.c_int
        self.lib.campus_cp_destroy.argtypes = [ct.c_void_p]
        self.lib.campus_cp_export_features.argtypes = [ct.c_void_p,float_ptr,ct.POINTER(ct.c_int),ct.c_int]
        self.lib.campus_cp_export_features.restype = ct.c_int
        if self.lib.campus_cp_api_version()!=1:
            raise RuntimeError('Unsupported native CenterPoint ABI')
        self.handle = self.lib.campus_cp_create(str(model_dir).encode(),str(engine_dir).encode())
        if not self.handle:raise RuntimeError(self.lib.campus_cp_last_error().decode())
        self.thresholds = np.asarray(thresholds or [.35]*5,dtype=np.float32)
        if self.thresholds.shape != (5,):raise ValueError('five class thresholds required')
        self.output = np.empty((32768,9),np.float32)
        self.max_range = max_range

    def predict(self, xyzt):
        points = np.ascontiguousarray(xyzt,dtype=np.float32)
        if points.ndim!=2 or points.shape[1]!=4:
            raise ValueError('CenterPoint base input must be XYZT, not XYZI')
        ptr = lambda a:a.ctypes.data_as(ct.POINTER(ct.c_float))
        count = self.lib.campus_cp_run(self.handle,ptr(points),len(points),ptr(self.thresholds),ptr(self.output),len(self.output))
        if count<0:raise RuntimeError(self.lib.campus_cp_last_error().decode())
        boxes = []
        for x,y,z,l,w,h,yaw,score,label in self.output[:count]:
            if np.hypot(x,y)>self.max_range:continue
            d = Detection(float(x),float(y),float(z),float(l),float(w),float(h),
                          wrap(-float(yaw)-math.pi/2),CLASSES[int(label)],float(score),'centerpoint')
            boxes.append(d.validate())
        return nms(boxes),dict(num_pillars=self.lib.campus_cp_pillar_count(self.handle),raw_candidates=count)

    def debug_features(self):
        count=self.lib.campus_cp_pillar_count(self.handle)
        features=np.empty((count,32,9),np.float32)
        cells=np.empty(count,np.int32)
        status=self.lib.campus_cp_export_features(self.handle,features.ctypes.data_as(ct.POINTER(ct.c_float)),
                                                cells.ctypes.data_as(ct.POINTER(ct.c_int)),count)
        if status<0:raise RuntimeError(self.lib.campus_cp_last_error().decode())
        return features,cells

    def close(self):
        if getattr(self,'handle',None):
            self.lib.campus_cp_destroy(self.handle)
            self.handle = None
