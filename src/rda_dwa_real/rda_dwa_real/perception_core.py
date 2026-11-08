"""Campus road surfaces, exact XY connectivity, and occupied envelopes.

Coordinates are sensor -> base_link, then heights above the observed ground.
Shape labels remain geometric hints, not semantic pedestrian classifications.
"""
from dataclasses import dataclass
import math
import time
import numpy as np
from .obstacle_fusion import infer_type


def point_cloud_xyz(msg):
    """Read XYZ float fields with byte order, point stride and row padding."""
    height, width = int(msg.height), int(msg.width)
    step = int(msg.point_step)
    row_step = int(getattr(msg, 'row_step', width*step))
    if height<=0 or width<=0 or step<=0 or row_step<width*step:
        raise ValueError('invalid cloud dimensions')
    fields={f.name:f for f in msg.fields}
    formats=[];offsets=[]
    for name in ('x','y','z'):
        f=fields.get(name)
        if f is None or int(getattr(f,'count',1))<1 or f.datatype not in (7,8):
            raise ValueError('XYZ float fields required')
        size=4 if f.datatype==7 else 8
        if f.offset<0 or f.offset+size>step:raise ValueError('invalid field offset')
        formats.append(('>' if getattr(msg,'is_bigendian',False) else '<')+f'f{size}')
        offsets.append(f.offset)
    if len(msg.data)<(height-1)*row_step+width*step:raise ValueError('truncated cloud')
    dtype=np.dtype(dict(names=['x','y','z'],formats=formats,offsets=offsets,itemsize=step))
    rows=np.ndarray((height,width),dtype=dtype,buffer=msg.data,strides=(row_step,step))
    return np.column_stack([rows[n].ravel() for n in ('x','y','z')]).astype(float,copy=False)


@dataclass
class PerceptionParams:
    vehicle_length: float=1.2
    vehicle_width: float=.8
    vehicle_outside: float=.05
    sensor_x: float=0.
    sensor_y: float=0.
    sensor_yaw: float=0.
    lidar_height: float=.45
    max_range: float=6.
    x_min: float=-10.
    x_max: float=10.
    y_min: float=-6.
    y_max: float=6.
    z_min: float=-.5
    z_max: float=.5
    ground_enabled: bool=True
    ground_roi_min: float=.8
    ground_roi_max: float=4.
    ground_min_points: int=50
    ground_max_tilt_deg: float=15.
    ground_reuse_s: float=1.
    ground_cell_size: float=.5
    ground_min_cells: int=12
    ground_min_span: float=1.
    ground_reference_z: float=-.36
    ground_reference_tolerance: float=.05
    ground_max_jump: float=.06
    ground_max_residual: float=.025
    height_min: float=.06
    height_max: float=2.2
    dropoff_height: float=.06
    voxel_size: float=.10
    cluster_gate: float=.30
    cluster_min_points: int=10
    cluster_min_extent: float=.15
    boundary_min_points: int=3
    boundary_min_extent: float=.30
    boundary_max_height: float=.20
    max_cluster_points: int=8000
    max_distance_tests: int=2000000
    structural_length: float=3.5
    structure_tile_size: float=1.2
    cavity_tile_size: float=.4
    cavity_min_extent: float=.8
    cavity_fill_ratio: float=.35
    drivable_resolution: float=.2
    drivable_max_height: float=.035
    drivable_min_points: int=2


class LidarPerceptionCore:
    def __init__(self,params=None):
        self.p=params or PerceptionParams()
        p=self.p
        positive=(p.voxel_size,p.cluster_gate,p.max_range,p.structure_tile_size,
                  p.structural_length,p.cavity_tile_size,p.cavity_min_extent,
                  p.ground_cell_size,p.ground_min_span,p.ground_reference_tolerance,
                  p.ground_max_jump,p.ground_max_residual,p.ground_reuse_s,
                  p.drivable_resolution,p.drivable_max_height,p.dropoff_height,
                  p.boundary_min_extent,p.boundary_max_height)
        if (not all(math.isfinite(v) and v>0 for v in positive)
                or math.sqrt(2.)*p.voxel_size>p.cluster_gate
                or not 0<p.cavity_fill_ratio<=1 or p.cavity_tile_size>p.structure_tile_size
                or not math.isfinite(p.ground_reference_z)
                or not 0<p.drivable_max_height<p.height_min<p.height_max
                or p.cluster_min_points<1 or p.boundary_min_points<3
                or p.ground_min_points<3 or p.ground_min_cells<3
                or p.max_cluster_points<1 or p.max_distance_tests<1 or p.drivable_min_points<1
                or math.ceil(2*p.max_range/p.drivable_resolution)>512
                or not 0<=p.ground_roi_min<p.ground_roi_max
                or not 0<p.ground_max_tilt_deg<90
                or not all(math.isfinite(v) for v in (p.sensor_x,p.sensor_y,p.sensor_yaw,p.lidar_height,
                    p.vehicle_length,p.vehicle_width,p.vehicle_outside,p.x_min,p.x_max,p.y_min,p.y_max))
                or p.vehicle_length<=0 or p.vehicle_width<=0 or p.vehicle_outside<0
                or p.x_min>=p.x_max or p.y_min>=p.y_max):
            raise ValueError('invalid perception geometry or support parameters')
        self.plane=None;self.plane_at=-math.inf;self.plane_yaw=0.


    def _ground(self,pts,stamp,yaw=0.):
        """Fit spatially balanced road returns with a vehicle-height anchor.

        The reference is the road height at the vehicle origin in base_link.
        It prevents raised/depressed sidewalks and platform tops becoming the
        driving plane merely because their returns are denser than the road.
        """
        p=self.p;r=np.linalg.norm(pts[:,:2],axis=1)
        roi=pts[(r>=p.ground_roi_min)&(r<=p.ground_roi_max)&(pts[:,2]>=-1.6)&(pts[:,2]<=.2)]
        if stamp<self.plane_at:
            self.plane=None;self.plane_at=-math.inf
        prior=None
        if self.plane is not None:
            angle=yaw-self.plane_yaw;c,s=math.cos(angle),math.sin(angle)
            prior=np.array([c*self.plane[0]+s*self.plane[1],-s*self.plane[0]+c*self.plane[1],self.plane[2]])
        fit=None;count=0;residual=None;cells=0;reason='insufficient_support'
        if len(roi)>=p.ground_min_points:
            seed=balanced_ground_returns(roi,p.ground_cell_size)
            fit,mask=fit_road_plane(seed,p,prior)
            if fit is None:
                reason='ground_geometry_or_reference'
            else:
                errors=roi[:,2]-(roi[:,:2]@fit[:2]+fit[2])
                count=int(np.count_nonzero(np.abs(errors)<=.035))
                cells=int(np.count_nonzero(mask))
                residual=float(np.median(np.abs(errors[np.abs(errors)<=.035]))) if count else None
                if count<p.ground_min_points or residual is None or residual>p.ground_max_residual:
                    fit=None;reason='ground_residual_or_support'
                elif prior is not None and stamp-self.plane_at<=p.ground_reuse_s and abs(fit[2]-prior[2])>p.ground_max_jump:
                    fit=None;reason='ground_jump_rejected'
                else:
                    reason='observed_road_surface'
        if fit is not None:
            self.plane=fit;self.plane_at=stamp
        elif prior is not None:
            self.plane=prior
        self.plane_yaw=yaw
        valid=self.plane is not None and 0<=stamp-self.plane_at<=p.ground_reuse_s
        return valid,dict(ground_plane=self.plane.tolist() if self.plane is not None else None,
                          ground_inliers=count,ground_cells=cells,ground_fit_reason=reason,
                          ground_reused=fit is None and valid,ground_residual_m=residual,
                          ground_age_s=stamp-self.plane_at if self.plane is not None else None)

    def process(self,xyz,stamp,pose=(0.,0.,0.)):
        begin=time.monotonic();p=self.p;points=np.asarray(xyz,dtype=float)
        if points.ndim!=2 or points.shape[1]!=3:raise ValueError('XYZ array required')
        if not math.isfinite(stamp) or len(pose)!=3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('finite source time and planar pose required')
        finite=np.isfinite(points).all(axis=1);points=points[finite].copy()
        c,s=math.cos(p.sensor_yaw),math.sin(p.sensor_yaw)
        x=points[:,0].copy();y=points[:,1].copy()
        points[:,0]=c*x-s*y+p.sensor_x;points[:,1]=s*x+c*y+p.sensor_y
        points[:,2]+=p.lidar_height  # TF base_link -> rslidar has +height
        radius=np.linalg.norm(points[:,:2],axis=1)
        keep=(radius<=p.max_range)&(points[:,0]>=p.x_min)&(points[:,0]<=p.x_max)&(points[:,1]>=p.y_min)&(points[:,1]<=p.y_max)
        self_box=(np.abs(points[:,0])<=p.vehicle_length/2+p.vehicle_outside)&(np.abs(points[:,1])<=p.vehicle_width/2+p.vehicle_outside)
        points=points[keep&~self_box]
        metrics=dict(finite_points=int(np.count_nonzero(finite)),roi_points=len(points))
        if p.ground_enabled:
            phase=time.monotonic();valid,ground=self._ground(points,stamp,pose[2]);metrics.update(ground,ground_duration_s=time.monotonic()-phase)
            if not valid:return [],dict(metrics,valid=False,reason='ground_unavailable')
            heights=points[:,2]-(points[:,0]*self.plane[0]+points[:,1]*self.plane[1]+self.plane[2])
            keep=((heights>=p.height_min)&(heights<=p.height_max))|(heights<=-p.dropoff_height)
            metrics['ground_removed_points']=int(np.count_nonzero(~keep))
            metrics['raised_boundary_points']=int(np.count_nonzero((heights>=p.height_min)&(heights<=p.boundary_max_height)))
            metrics['dropoff_points']=int(np.count_nonzero(heights<=-p.dropoff_height))
            metrics['drivable_grid']=observed_drivable_grid(points[:,:2],heights,p)
        else:
            heights=points[:,2];keep=(heights>=p.z_min)&(heights<=p.z_max)
        points=points[keep];heights=heights[keep];metrics['filtered_points']=len(points)
        if not len(points):return [],dict(metrics,valid=True,reason='empty_observations',cluster_count=0)
        phase=time.monotonic()
        keys=np.floor(points[:,:2]/p.voxel_size).astype(np.int32)
        cells,inverse,counts=cell_index(keys)
        starts=np.r_[0,np.cumsum(counts)[:-1]];ends=np.cumsum(counts)-1
        extrema=[]
        for axis in (0,1):
            order=np.lexsort((points[:,axis],inverse));extrema.extend((order[starts],order[ends]))
        representatives=np.stack([points[idx,:2] for idx in extrema],axis=1)
        selected_count=len(np.unique(np.concatenate(extrema)))
        metrics.update(cluster_input_points=selected_count,occupied_voxels=len(cells),voxel_duration_s=time.monotonic()-phase)
        if selected_count>p.max_cluster_points:return [],dict(metrics,valid=False,reason='cluster_capacity')
        phase=time.monotonic();distance_metrics={}
        try:
            labels,edges=voxel_components(cells,representatives,p.voxel_size,p.cluster_gate,
                                         points=points[:,:2],point_cells=inverse,
                                         max_distance_tests=p.max_distance_tests,metrics=distance_metrics)
        except DistanceBudgetExceeded:
            return [],dict(metrics,**distance_metrics,valid=False,reason='cluster_distance_capacity')
        raw_labels=labels[inverse];order=np.argsort(raw_labels,kind='stable')
        cuts=np.flatnonzero(np.diff(raw_labels[order]))+1;groups=np.split(order,cuts)
        metrics.update(distance_metrics,cluster_count=len(groups),component_edges=edges,component_duration_s=time.monotonic()-phase)
        phase=time.monotonic();detections=[];rejected=tiles=0;supports=[]
        pc,ps=math.cos(pose[2]),math.sin(pose[2])
        for indices in groups:
            raw_xy=points[indices,:2];support=len(indices);height=float(max(heights[indices]))
            cx,cy,L,W,heading=bounding_box_xy(raw_xy)
            boundary=bool(np.max(np.abs(heights[indices]))<=p.boundary_max_height)
            ordinary=support>=p.cluster_min_points and max(L,W)>=p.cluster_min_extent-1e-8
            supported_boundary=boundary and support>=p.boundary_min_points and max(L,W)>=p.boundary_min_extent-1e-8
            if not (ordinary or supported_boundary):
                rejected+=1;continue
            box_groups=[(raw_xy,(cx,cy,L,W,heading))]
            cavity=has_empty_box_area(raw_xy,(cx,cy,L,W,heading),p)
            if max(L,W)>p.structural_length or cavity:
                # Group raw supports by fixed world cells in one native sort.
                world=np.column_stack((pose[0]+pc*raw_xy[:,0]-ps*raw_xy[:,1],pose[1]+ps*raw_xy[:,0]+pc*raw_xy[:,1]))
                # Use finer cells around the car. Away from the car, ordinary
                # structural cells already avoid filling the entire cavity and
                # keep scene/tracker cost bounded for long campus boundaries.
                tile_size=p.cavity_tile_size if box_intersects_car((cx,cy,L,W,heading),p) else p.structure_tile_size
                _,ids,_=cell_index(np.floor(world/tile_size).astype(np.int32))
                tile_order=np.argsort(ids,kind='stable');splits=np.flatnonzero(np.diff(ids[tile_order]))+1
                box_groups=[(raw_xy[idx],bounding_box_xy(raw_xy[idx])) for idx in np.split(tile_order,splits)];tiles+=len(box_groups)
                # Reliability belongs to the accepted parent. Even a one-point
                # terminal tile must preserve that parent's observed envelope.
            for group,box in box_groups:
                cx,cy,L,W,heading=box if box is not None else bounding_box_xy(group)
                detections.append((cx,cy,infer_type(L,W),max(L,.10),max(W,.10),heading))
                supports.append(dict(points=len(group),parent_points=support,height_m=height,
                                     structural=len(box_groups)>1,cavity=cavity,boundary=boundary))
        metrics.update(box_duration_s=time.monotonic()-phase,processing_duration_s=time.monotonic()-begin)
        metrics.update(weak_clusters_removed=rejected,structural_tiles=tiles,detection_support=supports,detection_count=len(detections))
        return detections,dict(metrics,valid=True,reason='published')


def bounding_box_xy(points):
    """Fit an OBB to every raw support point, not only voxel representatives."""
    xy=np.asarray(points,dtype=float)
    centred=xy-xy.mean(axis=0)
    xx=float(np.mean(centred[:,0]**2));yy=float(np.mean(centred[:,1]**2));cross=float(np.mean(centred[:,0]*centred[:,1]))
    theta=.5*math.atan2(2.*cross,xx-yy);c,s=math.cos(theta),math.sin(theta)
    u=c*xy[:,0]+s*xy[:,1];v=-s*xy[:,0]+c*xy[:,1]
    ulo,uhi=float(u.min()),float(u.max());vlo,vhi=float(v.min()),float(v.max())
    um,vm=(ulo+uhi)/2,(vlo+vhi)/2
    return c*um-s*vm,s*um+c*vm,max(.05,uhi-ulo),max(.05,vhi-vlo),theta


class DistanceBudgetExceeded(RuntimeError):
    """An exact graph could not be completed within its pair-test budget."""


def voxel_components(cells,representatives,voxel_size,gate,*,points=None,
                     point_cells=None,max_distance_tests=2000000,metrics=None):
    """Connect occupied voxels using extrema first and raw points when needed.

    Within a voxel all extrema are connected (diagonal <= gate). Pairwise
    distances across nearby occupied voxels run in bounded native arrays,
    instead of Python loops over every point and every dense neighbour.
    A failed extrema test is not evidence that the raw voxels are disconnected.
    For ambiguous voxel pairs, check all distinct XY supports in small bounded
    blocks. Exhausting the work budget invalidates the observation; no partial
    graph is returned as an empty or fragmented free scene.
    """
    n=len(cells);parent=np.arange(n);left=[];right=[];edge_count=0
    stats=metrics if metrics is not None else {}
    stats.update(exact_distance_tests=0,exact_voxel_queries=0)
    if not n:return parent,0
    raw_cache={};raw_order=raw_starts=raw_counts=None
    if points is not None:
        points=np.asarray(points);point_cells=np.asarray(point_cells)
        if points.ndim!=2 or points.shape[1]!=2 or point_cells.shape!=(len(points),):
            raise ValueError('raw XY points and their voxel ids required')
        raw_order=np.argsort(point_cells,kind='stable')
        raw_counts=np.bincount(point_cells,minlength=n);raw_starts=np.r_[0,np.cumsum(raw_counts)[:-1]]
    bounds_min=representatives.min(axis=1);bounds_max=representatives.max(axis=1)

    def supports(cell):
        if cell not in raw_cache:
            start=raw_starts[cell]
            raw_cache[cell]=np.unique(points[raw_order[start:start+raw_counts[cell]]],axis=0)
        return raw_cache[cell]

    def raw_link(i,j,gate2):
        a,b=supports(i),supports(j);stats['exact_voxel_queries']+=1
        for first in range(0,len(a),128):
            aa=a[first:first+128]
            for second in range(0,len(b),128):
                bb=b[second:second+128];tests=len(aa)*len(bb)
                if stats['exact_distance_tests']+tests>max_distance_tests:
                    raise DistanceBudgetExceeded('exact XY connectivity budget exceeded')
                stats['exact_distance_tests']+=tests
                distance=(aa[:,None,0]-bb[None,:,0])**2+(aa[:,None,1]-bb[None,:,1])**2
                if np.any(distance<=gate2+1e-12):return True
        return False
    origin=cells.min(axis=0);span=cells.max(axis=0)-origin+1
    code=(cells[:,0].astype(np.int64)-origin[0])*int(span[1])+cells[:,1]-origin[1]
    extent=int(math.ceil(gate/voxel_size))+1;gate2=gate*gate
    for dx in range(extent+1):
        for dy in range(-extent,extent+1):
            if dx==0 and dy<=0:continue
            if (max(abs(dx)-1,0)*voxel_size)**2+(max(abs(dy)-1,0)*voxel_size)**2>gate2+1e-12:continue
            target=cells+np.array([dx,dy]);inside=(target[:,1]>=origin[1])&(target[:,1]<origin[1]+span[1])
            wanted=(target[:,0].astype(np.int64)-origin[0])*int(span[1])+target[:,1]-origin[1]
            j=np.searchsorted(code,wanted);i=np.flatnonzero(inside&(j<n))
            if not len(i):continue
            j=j[i];keep=code[j]==wanted[i];i=i[keep];j=j[keep]
            if not len(i):continue
            a=representatives[i];b=representatives[j]
            distance=(a[:,:,None,0]-b[:,None,:,0])**2+(a[:,:,None,1]-b[:,None,:,1])**2
            linked=np.min(distance,axis=(1,2))<=gate2+1e-12
            if points is not None:
                gap=np.maximum(np.maximum(bounds_min[i]-bounds_max[j],bounds_min[j]-bounds_max[i]),0.)
                ambiguous=np.flatnonzero(~linked&(np.sum(gap*gap,axis=1)<=gate2+1e-12))
                for k in ambiguous:linked[k]=raw_link(int(i[k]),int(j[k]),gate2)
            i=i[linked];j=j[linked];edge_count+=len(i)
            left.append(i);right.append(j)
    if edge_count:
        left=np.concatenate(left);right=np.concatenate(right)
        while True:
            a=parent[left];b=parent[right];updated=parent.copy()
            np.minimum.at(updated,np.maximum(a,b),np.minimum(a,b))
            while np.any(updated!=updated[updated]):updated=updated[updated]
            if np.array_equal(updated,parent):break
            parent=updated
    return parent,edge_count


def cell_index(keys):
    """Native integer sort instead of structured two-column unique sorting."""
    origin=keys.min(axis=0);stride=int(keys[:,1].max()-origin[1]+1)
    code=(keys[:,0].astype(np.int64)-origin[0])*stride+keys[:,1]-origin[1]
    unique,inverse,counts=np.unique(code,return_inverse=True,return_counts=True)
    cells=np.column_stack((unique//stride+origin[0],unique%stride+origin[1])).astype(np.int32)
    return cells,inverse,counts


def balanced_ground_returns(points,cell_size):
    """One low and one median return per cell, independent of return density."""
    _,ids,counts=cell_index(np.floor(points[:,:2]/cell_size).astype(np.int32))
    starts=np.r_[0,np.cumsum(counts)[:-1]]
    order=np.lexsort((points[:,1],points[:,0],points[:,2],ids))
    chosen=np.unique(np.r_[order[starts+counts//10],order[starts+counts//2]])
    seed=points[chosen]
    return seed[np.lexsort((seed[:,2],seed[:,1],seed[:,0]))]


def least_squares_plane(points,weights=None):
    weights=np.ones(len(points)) if weights is None else np.asarray(weights)
    weights=weights/weights.sum();mean=np.sum(points*weights[:,None],axis=0)
    centred=points-mean
    xx=np.sum(weights*centred[:,0]**2);yy=np.sum(weights*centred[:,1]**2);xy=np.sum(weights*centred[:,0]*centred[:,1])
    xz=np.sum(weights*centred[:,0]*centred[:,2]);yz=np.sum(weights*centred[:,1]*centred[:,2]);det=xx*yy-xy*xy
    if det<=max(1e-10,1e-5*(xx+yy)**2):return None
    a=(xz*yy-yz*xy)/det;b=(yz*xx-xz*xy)/det
    return np.array([a,b,mean[2]-a*mean[0]-b*mean[1]])


def fit_road_plane(seed,p,prior=None):
    """Bounded deterministic plane hypotheses with an origin-height constraint.

    Local supports have more weight than remote sidewalks. Models are scored
    on spatially balanced samples; accepted inliers must cover two dimensions.
    """
    if len(seed)<p.ground_min_cells:return None,None
    candidates=[np.array([0.,0.,p.ground_reference_z])]
    if prior is not None:candidates.append(prior)
    whole=least_squares_plane(seed)
    if whole is not None:candidates.append(whole)
    triples=np.random.default_rng(0).integers(0,len(seed),size=(192,3))
    a=seed[triples[:,0]];u=seed[triples[:,1]]-a;v=seed[triples[:,2]]-a
    det=u[:,0]*v[:,1]-u[:,1]*v[:,0];usable=np.abs(det)>.05
    a,u,v,det=a[usable],u[usable],v[usable],det[usable]
    ax=(u[:,2]*v[:,1]-v[:,2]*u[:,1])/det
    ay=(u[:,0]*v[:,2]-v[:,0]*u[:,2])/det
    candidates.extend(np.column_stack((ax,ay,a[:,2]-ax*a[:,0]-ay*a[:,1])))
    candidates=np.asarray(candidates)
    allowed=(np.abs(candidates[:,2]-p.ground_reference_z)<=p.ground_reference_tolerance)&(np.linalg.norm(candidates[:,:2],axis=1)<=math.tan(math.radians(p.ground_max_tilt_deg)))
    candidates=candidates[allowed]
    if not len(candidates):return None,None
    error=seed[:,2,None]-(seed[:,:2]@candidates[:,:2].T+candidates[None,:,2])
    inliers=np.abs(error)<=.035
    anchors=(np.linalg.norm(seed[:,:2],axis=1)<=1.8)&(np.abs(seed[:,1])<=p.vehicle_width/2+p.vehicle_outside)
    # The surface continuing immediately ahead/behind the car defines the
    # road level. A much larger sidewalk must not tilt the fit toward itself.
    weights=np.where(anchors,20.,1.)
    score=np.sum(weights[:,None]*inliers*(1.-np.minimum(np.abs(error),.035)/.035),axis=0)
    best=int(np.argmax(score));fit=candidates[best];mask=inliers[:,best]
    for _ in range(3):
        values=seed[mask]
        if len(values)<p.ground_min_cells or np.any(np.ptp(values[:,:2],axis=0)<p.ground_min_span):return None,None
        candidate=least_squares_plane(values,weights[mask])
        if candidate is None:return None,None
        if abs(candidate[2]-p.ground_reference_z)>p.ground_reference_tolerance or math.hypot(*candidate[:2])>math.tan(math.radians(p.ground_max_tilt_deg)):
            return None,None
        fit=candidate;mask=np.abs(seed[:,2]-(seed[:,:2]@fit[:2]+fit[2]))<=.035
    values=seed[mask]
    if len(values)<p.ground_min_cells or np.any(np.ptp(values[:,:2],axis=0)<p.ground_min_span):return None,None
    cells,_,_=cell_index(np.floor(values[:,:2]/p.ground_cell_size).astype(np.int32))
    if len(cells)<p.ground_min_cells:return None,None
    return fit,mask


def box_intersects_car(box,p):
    cx,cy,length,width,angle=box;c,s=math.cos(angle),math.sin(angle)
    # SAT intersection against the actual vehicle, including its outside pad.
    hx=p.vehicle_length/2+p.vehicle_outside;hy=p.vehicle_width/2+p.vehicle_outside
    return (abs(cx)<=hx+abs(c)*length/2+abs(s)*width/2
                and abs(cy)<=hy+abs(s)*length/2+abs(c)*width/2
                and abs(c*cx+s*cy)<=length/2+abs(c)*hx+abs(s)*hy
                and abs(-s*cx+c*cy)<=width/2+abs(s)*hx+abs(c)*hy)


def has_empty_box_area(points,box,p):
    cx,cy,length,width,angle=box;c,s=math.cos(angle),math.sin(angle)
    if box_intersects_car(box,p):return True
    if min(length,width)<p.cavity_min_extent:return False
    local=np.column_stack((c*(points[:,0]-cx)+s*(points[:,1]-cy),
                           -s*(points[:,0]-cx)+c*(points[:,1]-cy)))
    cells,_,_=cell_index(np.floor((local-local.min(axis=0))/p.voxel_size).astype(np.int32))
    span=np.ptp(cells,axis=0)+1;fill=len(cells)/float(span[0]*span[1])
    return fill<p.cavity_fill_ratio


def observed_drivable_grid(xy,heights,p):
    """Observed road=0, elevated/depressed supports=100, unseen cells=-1.

    No ray casting, extrapolation, or fill through occluded/unsupported space.
    These values describe surface evidence, not vehicle-footprint clearance.
    """
    size=int(math.ceil(2*p.max_range/p.drivable_resolution));origin=-size*p.drivable_resolution/2
    keys=np.floor((xy-origin)/p.drivable_resolution).astype(np.int32)
    inside=(keys[:,0]>=0)&(keys[:,0]<size)&(keys[:,1]>=0)&(keys[:,1]<size)
    keys=keys[inside];heights=heights[inside];ids=keys[:,1]*size+keys[:,0]
    road=np.abs(heights)<=p.drivable_max_height
    blocked=((heights>=p.height_min)&(heights<=p.height_max))|(heights<=-p.dropoff_height)
    counts=np.bincount(ids[road],minlength=size*size)
    grid=np.full(size*size,-1,dtype=np.int8);grid[counts>=p.drivable_min_points]=0
    grid[ids[blocked]]=100
    return dict(resolution=p.drivable_resolution,width=size,height=size,
                origin_x=origin,origin_y=origin,data=grid.tolist(),
                free_cells=int(np.count_nonzero(grid==0)),blocked_cells=int(np.count_nonzero(grid==100)),
                unknown_cells=int(np.count_nonzero(grid==-1)))
