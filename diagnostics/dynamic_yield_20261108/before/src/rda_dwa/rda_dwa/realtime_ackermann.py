"""Bounded Ackermann RDA-DWA execution for the real vehicle.

Samples executable steering targets, checks ramped OBB trajectories at all
prediction times, and uses a persistent static visibility guide. No ROS.
"""
import math,time,heapq
from dataclasses import replace
import numpy as np
from .enhanced_dwa import EnhancedDWA
from .motion_safety import VehicleGeometry,rollout_rear,to_centres,minimum_gaps,obstacle_from_object,rollout_stop

class RealtimeAckermannPlanner(EnhancedDWA):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.geometry=VehicleGeometry(self.cc.ack.length,self.cc.ack.width,self.cc.ack.wheelbase)
        self.reference=[];self.telemetry={};self._guide=[];self._guide_goal=None
        self.current_steer=0.;self.obstacle_age=0.
    def _guide_target(self,start,goal,obstacles,deadline):
        # Guide boxes are a separate geometric heuristic; the final OBB rollout
        # ALWAYS validates physical length, width, orientation and safety margin.
        # Build the heuristic in a goal-aligned frame fixed for this goal.
        # Map yaw must not change the required manoeuvre for an identical scene.
        if self._guide_goal!=tuple(goal):
            self._guide=[];self._guide_goal=tuple(goal);self._guide_origin=start
            self._guide_axis=math.atan2(goal[1]-start[1],goal[0]-start[0])
        ox,oy=self._guide_origin;basis_c,basis_s=math.cos(self._guide_axis),math.sin(self._guide_axis)
        def inward(p):
            dx,dy=p[0]-ox,p[1]-oy;return (round(basis_c*dx+basis_s*dy,12),round(-basis_s*dx+basis_c*dy,12))
        def outward(p):return (ox+basis_c*p[0]-basis_s*p[1],oy+basis_s*p[0]+basis_c*p[1])
        start,goal=inward(start),inward(goal)
        obstacles=[replace(o,x=inward((o.x,o.y))[0],y=inward((o.x,o.y))[1],heading=o.heading-self._guide_axis,
                           vx=basis_c*o.vx+basis_s*o.vy,vy=-basis_s*o.vx+basis_c*o.vy) for o in obstacles]
        boxes=[];pad=self.geometry.width/2+getattr(self.params,'safety_margin',.12)+getattr(self.params,'guide_extra',.08)
        for o in obstacles:
            if o.speed>.05 or o.uncertain:continue
            c,s=abs(math.cos(o.heading)),abs(math.sin(o.heading))
            hx=c*o.length/2+s*o.width/2+pad;hy=s*o.length/2+c*o.width/2+pad
            if math.hypot(o.x-start[0],o.y-start[1])<8:boxes.append((o.x-hx,o.y-hy,o.x+hx,o.y+hy))
        if len(boxes)>8:boxes=sorted(boxes,key=lambda b:math.hypot((b[0]+b[2])/2-start[0],(b[1]+b[3])/2-start[1]))[:8]
        def visible(a,b):
            for x0,y0,x1,y1 in boxes:
                lo,hi=0.,1.
                for p,d,mn,mx in [(a[0],b[0]-a[0],x0,x1),(a[1],b[1]-a[1],y0,y1)]:
                    if abs(d)<1e-9:
                        if p<=mn+1e-5 or p>=mx-1e-5:lo=2.;break
                    else:
                        aa,bb=sorted(((mn+1e-5-p)/d,(mx-1e-5-p)/d));lo=max(lo,aa);hi=min(hi,bb)
                if lo<hi and hi>0 and lo<1:return False
            return True
        while self._guide and (math.dist(start,self._guide[0])<.65 or (len(self._guide)>1 and visible(start,self._guide[1]))):self._guide.pop(0)
        if self._guide:return outward(self._guide[0])
        if visible(start,goal):self._guide=[];return outward(goal)
        nodes=[start,goal]
        for x0,y0,x1,y1 in boxes:nodes.extend([(x0-.05,y0-.05),(x0-.05,y1+.05),(x1+.05,y0-.05),(x1+.05,y1+.05)])
        dist=[math.inf]*len(nodes);dist[0]=0.;prev={};queue=[(0.,0)]
        while queue and time.perf_counter()<deadline:
            value,i=heapq.heappop(queue)
            if value!=dist[i]:continue
            if i==1:break
            for j,b in enumerate(nodes):
                if i==j or not visible(nodes[i],b):continue
                vv=round(value+math.dist(nodes[i],b),10)
                if vv<dist[j]:dist[j]=vv;prev[j]=i;heapq.heappush(queue,(vv,j))
        if 1 in prev:
            route=[];i=1
            while i:route.append(nodes[i]);i=prev[i]
            self._guide=list(reversed(route));return outward(self._guide[0])
        return outward(goal)
    def plan(self,robot,goal,obstacles,trap=None):
        begin=time.perf_counter();deadline=begin+getattr(self.params,'planning_budget_s',.08)
        self.reference=[]
        obs=[o if hasattr(o,'vx') else obstacle_from_object(o) for o in obstacles]
        if len(obs)>64:self.telemetry={'reason':'obstacle_capacity'};return False,0.,0.
        centre=(robot.x+self.geometry.rear_offset*math.cos(robot.theta),robot.y+self.geometry.rear_offset*math.sin(robot.theta))
        local=self._guide_target(centre,goal,obs,min(deadline,begin+.015))
        dt=max(.02,min(.2,self.params.dt));acc=getattr(self.params,'command_acc',.7)
        goal_cap=math.dist(centre,goal)*1.2
        forecast=getattr(self.params,'velocity_horizon_targets',False)
        vmax=min(self.params.max_linear_velocity,getattr(self.params,'cruise_velocity',self.params.max_linear_velocity),goal_cap) if forecast else min(self.params.max_linear_velocity,max(0.,robot.v)+acc*dt,goal_cap)
        # No commanded acceleration beyond the dynamic window. Full steering
        # targets are rolled out through the measured actuator rate, not snapped.
        velocities=np.unique(np.r_[0.,np.linspace(min(vmax,max(0.,robot.v-1.2*dt)),vmax,5)])
        deltas=np.unique(np.r_[np.linspace(-self.cc.ack.delta_max,self.cc.ack.delta_max,21),self.current_steer,0.])
        vs,ds=np.meshgrid(velocities,deltas,indexing='ij');vs=vs.ravel();ds=ds.ravel()
        if forecast:
            curve_limit=np.sqrt(getattr(self.params,'lateral_acc_max',.6)*self.geometry.wheelbase/np.maximum(np.abs(np.tan(ds)),.001))
            vs=np.minimum(vs,curve_limit)
        rear,speeds,times=rollout_rear(robot.x,robot.y,robot.theta,robot.v,self.current_steer,vs,ds,self.geometry,
                                    self.params.sim_time,.1,acc,1.2,getattr(self.params,'command_steer_rate',math.radians(35)))
        centres=to_centres(rear,self.geometry)
        sweep=(max(vmax,robot.v)+max(vmax,robot.v)*math.tan(self.cc.ack.delta_max)/self.geometry.wheelbase*self.geometry.radius)*.05
        margin=getattr(self.params,'safety_margin',.12)
        gaps=minimum_gaps(centres,times,obs,self.geometry,self.obstacle_age,sweep)
        if time.perf_counter()>deadline:self.telemetry={'reason':'planning_budget','duration_s':time.perf_counter()-begin};return False,0.,0.
        dynamic=any(o.speed>.1 or o.uncertain for o in obs)
        if dynamic and getattr(self.params,'velocity_horizon_targets',False):
            stop,st=rollout_stop(robot.x,robot.y,robot.theta,robot.v,self.current_steer,vs,ds,self.geometry,self.params.sim_time)
            stop_gaps=minimum_gaps(to_centres(stop,self.geometry),st,obs,self.geometry,self.obstacle_age,sweep)
            gaps=np.minimum(gaps,stop_gaps)
        feasible=gaps>=margin
        static_motion=bool(np.any(feasible&(vs>.025)))
        if any(o.speed>.1 or o.uncertain for o in obs):
            static_obs=[o for o in obs if o.speed<=.1 and not o.uncertain]
            static_gaps=minimum_gaps(centres,times,static_obs,self.geometry,self.obstacle_age,sweep)
            static_motion=bool(np.any((static_gaps>=margin)&(vs>.025)))
        if time.perf_counter()>deadline:self.telemetry={'reason':'planning_budget','duration_s':time.perf_counter()-begin};return False,0.,0.
        end=centres[:,-1,:];d0=math.dist(centre,local);dist=np.hypot(end[:,0]-local[0],end[:,1]-local[1])
        desired=math.atan2(local[1]-centre[1],local[0]-centre[0]);err=(desired-end[:,2]+math.pi)%(2*math.pi)-math.pi
        score=3*(d0-dist)-getattr(self.params,'heading_weight',.3)*np.abs(err)+.15*vs
        score+=.15*np.minimum(gaps,.4)
        score-=getattr(self.params,'steer_weight',.12)*(ds/self.cc.ack.delta_max)**2
        score-=getattr(self.params,'steer_change_weight',.08)*((ds-self.current_steer)/self.cc.ack.delta_max)**2
        score[~feasible]=-np.inf
        if not np.any(feasible):
            reason='dynamic_yield' if static_motion and any(o.speed>.1 or o.uncertain for o in obs) else 'blocked'
            self.telemetry={'reason':reason,'duration_s':time.perf_counter()-begin,'guide_target':local,'static_motion_feasible':static_motion}
            return False,0.,0.
        k=int(np.flatnonzero(score>=np.max(score)-1e-10)[0]);cruise=float(vs[k]);v=min(cruise,max(0.,robot.v)+acc*dt) if forecast else cruise
        delta=float(ds[k]);w=v*math.tan(delta)/self.geometry.wheelbase
        # Emit the VALIDATED ramped path, rather than extrapolating the first
        # command to an unvalidated constant-curvature 2.5 m path.
        if v>=.005:
            self.reference=[(float(p[0]),float(p[1]),float(p[2]),float(max(v,pv)) if forecast else v,float(t)) for p,pv,t in zip(rear[k],speeds[k],times)]
        reason='motion' if v>.005 else ('dynamic_yield' if static_motion and any(o.speed>.1 or o.uncertain for o in obs) else ('orientation_stall' if np.any(feasible&(vs>.025)) else 'blocked'))
        self.telemetry={'reason':reason,'duration_s':time.perf_counter()-begin,
                        'candidates':len(vs),'feasible':int(sum(feasible)),'min_gap':float(gaps[k]) if math.isfinite(gaps[k]) else None,'guide_target':local,'selected_delta':delta,'static_motion_feasible':static_motion,'forecast_progress':float(d0-dist[k]),'cruise':cruise}
        return True,v,w
