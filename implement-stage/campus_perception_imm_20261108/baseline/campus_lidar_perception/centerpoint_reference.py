"""CPU reference for the base model's XYZT -> N,32,9 feature contract and decoding."""
import math
import numpy as np

from .types import Detection
from .coordinates import wrap

CLASSES = ['CAR','TRUCK','BUS','BICYCLE','PEDESTRIAN']


def pillar_features(points, max_voxels=40000):
    points = np.asarray(points, np.float32)
    keep = np.isfinite(points).all(axis=1) & np.all(points[:,:3] >= [-76.8,-76.8,-4.],axis=1)
    keep &= np.all(points[:,:3] < [76.8,76.8,6.],axis=1)
    p = points[keep]
    cell = np.floor((p[:,:2]+76.8)/.32).astype(np.int64)
    ids = cell[:,1]*480+cell[:,0]
    unique, inverse = np.unique(ids, return_inverse=True)
    if len(unique) > max_voxels:raise ValueError('too many pillars')
    out = np.zeros((len(unique),32,9),np.float32)
    for i,c in enumerate(unique):
        g = p[inverse == i][:32]
        features = np.column_stack((g, g[:,:3]-g[:,:3].mean(axis=0),
                                     g[:,:2]-[c%480*.32-76.8+.16,c//480*.32-76.8+.16]))
        out[i,:len(g)] = features
    return out, unique.astype(np.int32)


def decode_head(outputs, thresholds=(.35,)*5, stride=1):
    hm = outputs['heatmap'][0]
    scores = 1/(1+np.exp(-np.clip(hm, -80, 80)))
    label = scores.argmax(axis=0)
    best = scores.max(axis=0)
    ys,xs = np.nonzero(best >= np.asarray(thresholds)[label])
    detections = []
    for y,x in zip(ys,xs):
        cls = int(label[y,x])
        sin,cos = outputs['rot'][0,:,y,x]
        if cls != 4 and math.hypot(sin,cos)<.3:continue
        # Published dimension head is width, length, height in log space.
        w,l,h = np.exp(outputs['dim'][0,:,y,x])
        dx,dy = outputs['reg'][0,:,y,x]
        detections.append(Detection(x=float((x+dx)*.32*stride-76.8),
                                   y=float((y+dy)*.32*stride-76.8),
                                   z=float(outputs['height'][0,0,y,x]),
                                   length=float(l),width=float(w),height=float(h),
                                   yaw=wrap(-math.atan2(sin,cos)-math.pi/2),label=CLASSES[cls],
                                   score=float(best[y,x]),source='centerpoint'))
    return detections


def corners(d):
    c,s = math.cos(d.yaw),math.sin(d.yaw)
    local = np.array([[-d.length/2,-d.width/2],[d.length/2,-d.width/2],
                      [d.length/2,d.width/2],[-d.length/2,d.width/2]])
    return local@np.array([[c,s],[-s,c]])+[d.x,d.y]


def rotated_iou(a,b):
    subject = list(corners(a))
    clip = corners(b)
    for i in range(4):
        p,q = clip[i],clip[(i+1)%4]
        edge = q-p
        def side(x):return edge[0]*(x[1]-p[1])-edge[1]*(x[0]-p[0])
        result = []
        if not subject:return 0.
        previous = subject[-1]
        for current in subject:
            sp,sc = side(previous),side(current)
            if (sp >= 0) != (sc >= 0):
                result.append(previous+(current-previous)*sp/(sp-sc))
            if sc >= 0:result.append(current)
            previous = current
        subject = result
    if len(subject)<3:return 0.
    polygon = np.array(subject)
    area = .5*abs(np.sum(polygon[:,0]*np.roll(polygon[:,1],-1)-polygon[:,1]*np.roll(polygon[:,0],-1)))
    return float(area/max(1e-9,a.length*a.width+b.length*b.width-area))


def nms(detections, circle_distance=.5, iou_threshold=.1, maximum=500):
    kept = []
    for d in sorted(detections,key=lambda v:v.score,reverse=True):
        duplicate = False
        for old in kept:
            distance = math.hypot(d.x-old.x,d.y-old.y)
            if d.label == old.label and (distance < circle_distance or
                                        (distance < 10. and rotated_iou(d,old)>iou_threshold)):
                duplicate = True;break
        if not duplicate:
            kept.append(d)
        if len(kept)>=maximum:break
    return kept
