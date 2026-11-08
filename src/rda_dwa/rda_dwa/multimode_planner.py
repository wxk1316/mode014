"""ACK/CRAB mode-local planning with explicit stationary switch occupancy.
ACK remains the calibrated planner. Mode candidates compare time to the SAME
local waypoint. Mode selection is committed by the supervisor, never a tracker
error threshold. Models and returned references carry body yaw, not path yaw.
"""
import math,time
import numpy as np
from .crab_model import CrabParams,rollout_crab
from .motion_safety import to_centres,minimum_gaps

class MultiModePlanner:
    supports_multimode=True
    def __init__(self,ack):
        self.ack=ack;self.params=ack.params;self.geometry=ack.geometry
        self.reference=[];self.telemetry={};self.current_mode=0;self.mode_epoch=0
        self._guide_cache=None;self._guide_at=-1e9;self._guide_key=None;self._reverse_schedule=None;self.current_steer=0.;self.obstacle_age=0.;self.current_acc=0.;self.current_steer_rate=0.
    def __getattr__(self,name):
        ack=self.__dict__.get('ack')
        if ack is None:raise AttributeError(name)
        return getattr(ack,name)
    def crab_params(self):
        return CrabParams(v_max=getattr(self.params,'crab_v_max',.5),beta_rate=math.radians(getattr(self.params,'crab_beta_rate_deg',30.)),acc=getattr(self.params,'command_acc',.7),dec=getattr(self.params,'command_dec',1.2),jerk=getattr(self.params,'command_jerk',2.))
    def guide(self,robot,goal,obs):
        from .local_route import route_waypoint
        centre=(robot.x+.345*math.cos(robot.theta),robot.y+.345*math.sin(robot.theta));now=getattr(robot,'planning_stamp',0.)
        key=(tuple(goal),round(robot.theta/.2))
        small=[o for o in obs if o.speed<=.1 and o.length<4. and o.width<2.]
        if len(small)==1 and self.current_mode==0:
            point=self.ack._guide_target(centre,goal,obs,time.perf_counter()+.015)
            if math.dist(centre,point)<8.:self._guide_found=True;return point
        if self._guide_cache is None or key!=self._guide_key or now-self._guide_at>.5 or math.dist(centre,self._guide_cache)<.25:
            point,found=route_waypoint(centre,robot.theta,goal,obs,self.geometry,getattr(self.params,'safety_margin',.12))
            self._guide_cache=point;self._guide_at=now;self._guide_key=key;self._guide_found=found
        return self._guide_cache
    def crab_reorientation(self,robot,goal,obs):
        from .local_route import route_waypoint
        from .motion_safety import spin_is_safe
        centre=(robot.x+.345*math.cos(robot.theta),robot.y+.345*math.sin(robot.theta))
        desired=math.atan2(goal[1]-centre[1],goal[0]-centre[0])
        for yaw in (getattr(self,'_task_yaw',desired),getattr(self,'_task_yaw',desired)+math.pi/2,getattr(self,'_task_yaw',desired)-math.pi/2,desired,desired+math.pi/2,desired-math.pi/2):
            if abs((yaw-robot.theta+math.pi)%(2*math.pi)-math.pi)<math.radians(1.5):continue
            point,found=route_waypoint(centre,yaw,goal,obs,self.geometry,getattr(self.params,'safety_margin',.12))
            forward=(point[0]-centre[0])*math.cos(yaw)+(point[1]-centre[1])*math.sin(yaw)
            if found and spin_is_safe(*centre,robot.theta,yaw,obs,self.geometry,self.obstacle_age):return yaw
        return None
    def plan_crab(self,robot,goal,obs,local=None):
        cp=self.crab_params();centre=np.array([robot.x+.345*math.cos(robot.theta),robot.y+.345*math.sin(robot.theta)])
        target=np.array(local if local is not None else self.guide(robot,goal,obs));offset=target-centre
        if not getattr(self,'_guide_found',True):
            from .local_route import route_waypoint
            from .motion_safety import spin_is_safe
            desired=math.atan2(goal[1]-centre[1],goal[0]-centre[0])
            for yaw in (getattr(self,'_task_yaw',desired),getattr(self,'_task_yaw',desired)+math.pi/2,getattr(self,'_task_yaw',desired)-math.pi/2,desired,desired+math.pi/2,desired-math.pi/2):
                if abs((yaw-robot.theta+math.pi)%(2*math.pi)-math.pi)<math.radians(1.5):continue
                _,found=route_waypoint(centre,yaw,goal,obs,self.geometry,getattr(self.params,'safety_margin',.12))
                if found and spin_is_safe(*centre,robot.theta,yaw,obs,self.geometry,self.obstacle_age):
                    self.reference=[];self.telemetry=dict(reason='blocked',route_yaw=yaw,guide_target=goal,motion_mode='CRAB',forecast_progress=0.)
                    return False,0.,0.
        direction=(math.atan2(offset[1],offset[0])-robot.theta+math.pi)%(2*math.pi)-math.pi
        motion_sign=1.
        if abs(direction)>math.pi/2+.02:
            motion_sign=-1.;direction=(direction+math.pi+math.pi)%(2*math.pi)-math.pi
        if abs(direction)>math.pi/2+.02:
            yaw=self.crab_reorientation(robot,goal,obs)
            if yaw is not None:
                self.reference=[];self.telemetry=dict(reason='blocked',route_yaw=yaw,guide_target=goal,motion_mode='CRAB',forecast_progress=0.);return False,0.,0.
            self.reference=[];self.telemetry={'reason':'crab_heading_unreachable','guide_target':tuple(target)};return False,0.,0.
        beta=float(np.clip(direction,-math.pi/2,math.pi/2));dist=math.dist(centre,goal)
        brake_delay=getattr(self.params,'actuator_delay_s',.1)+.25+cp.acc/cp.jerk
        cap=min(cp.v_max,max(0.,math.sqrt((cp.dec*brake_delay)**2+2*cp.dec*max(0.,dist-.18))-cp.dec*brake_delay))
        waypoint_speed=min(cap,max(.025,math.dist(centre,target)/3.5))
        speeds=np.unique(np.r_[0.,np.linspace(0.,cap,6),min(cap,robot.v),waypoint_speed])
        betas=np.unique(np.clip(np.r_[beta,self.current_steer,beta+np.radians([-15,-7,0,7,15]),0.],-math.pi/2,math.pi/2))
        vv,bb=np.meshgrid(speeds*motion_sign,betas,indexing='ij')
        poses,vel,ts,angles=rollout_crab(robot.x,robot.y,robot.theta,robot.v,self.current_steer,vv.ravel(),bb.ravel(),max(3.,getattr(self.params,'sim_time',3.)),params=cp,initial_acc=self.current_acc*(-1 if getattr(robot,"command_speed",0.)<0 else 1),initial_beta_rate=self.current_steer_rate,initial_sign=-1 if getattr(robot,"command_speed",0.)<0 else 1)
        centres=to_centres(poses,self.geometry);gaps=minimum_gaps(centres,ts,obs,self.geometry,self.obstacle_age,np.maximum(vel,np.c_[vel[:,1:],vel[:,-1]])*.025+.002)
        # Short-horizon command must retain a independently checked braking tail.
        margin=getattr(self.params,'safety_margin',.12);progress=np.linalg.norm(offset)-np.linalg.norm(centres[:,-1,:2]-target,axis=1)
        score=3*progress-.12*np.abs(bb.ravel()-self.current_steer)+.08*vv.ravel()+.1*np.minimum(gaps,.4)
        score[gaps<margin]=-np.inf
        current_sign=-1. if getattr(robot,"command_speed",0.)<-.001 else 1.
        if robot.v>.035 and motion_sign!=current_sign:score[np.abs(vv.ravel())>.001]=-np.inf
        if not np.any(np.isfinite(score)):
            yaw=self.crab_reorientation(robot,goal,obs)
            if yaw is not None:
                self.reference=[];self.telemetry=dict(reason='blocked',route_yaw=yaw,guide_target=goal,motion_mode='CRAB',forecast_progress=0.);return False,0.,0.
            self.reference=[];self.telemetry={'reason':'blocked','guide_target':tuple(target),'motion_mode':'CRAB'};return False,0.,0.
        k=int(np.argmax(score));v=float(vv.ravel()[k]);b=float(bb.ravel()[k]);self.reference=[]
        if abs(v)<.005 and robot.v<.035 and abs(beta-self.current_steer)>math.radians(1):
            wait=abs(beta-self.current_steer)/cp.beta_rate+.3
            st=np.linspace(0.,wait,max(2,int(math.ceil(wait/.05))+1));park=np.tile([robot.x,robot.y,robot.theta],(len(st),1))
            pg=float(minimum_gaps(to_centres(park,self.geometry),st,obs,self.geometry,self.obstacle_age,.002))
            if pg>=margin:
                self.telemetry=dict(reason='crab_prepare',selected_beta=beta,crab_reverse=motion_sign<0,guide_target=tuple(target),forecast_progress=0.,motion_mode='CRAB')
                return True,0.,beta
        if abs(v)>.005 or robot.v>.025:self.reference=[(float(q[0]),float(q[1]),float(q[2]),v,float(t),b) for q,t in zip(poses[k],ts)]
        reason='crab_motion' if abs(v)>.005 else ('goal_braking' if robot.v>.025 else ('crab_prepare' if abs(b-self.current_steer)>math.radians(1) else 'crab_hold'))
        self.telemetry=dict(reason=reason,motion_mode='CRAB',crab_reverse=(v<0 or (v==0 and current_sign<0)),guide_target=tuple(target),selected_beta=b,forecast_progress=float(progress[k]),min_gap=float(gaps[k]) if math.isfinite(float(gaps[k])) else None,candidates=len(score))
        return True,v,b
    def switch_candidate(self,robot,goal,obs,local=None):
        """Zero-speed preparation inside >=3s switching plus 4s translation.
        Only queried near standstill: stopping from current speed is not omitted.
        """
        if robot.v>.035:return None
        cp=self.crab_params();centre=np.array([robot.x+.345*math.cos(robot.theta),robot.y+.345*math.sin(robot.theta)])
        target=np.array(local if local is not None else self.guide(robot,goal,obs));diff=target-centre
        beta=(math.atan2(diff[1],diff[0])-robot.theta+math.pi)%(2*math.pi)-math.pi
        forward_error=diff[0]*math.cos(robot.theta)+diff[1]*math.sin(robot.theta)
        if abs(beta)>math.pi/2+1e-5:
            if forward_error<-.03 or abs(beta)>math.pi/2+.03:
                beta=(beta+math.pi+math.pi)%(2*math.pi)-math.pi;candidate_sign=-1.
            else:beta=float(np.clip(beta,-math.pi/2,math.pi/2));candidate_sign=1.
        else:candidate_sign=1.
        wait=3.+.6+abs(self.current_steer)/cp.beta_rate
        candidate_speed=min(cp.v_max,max(.1,math.dist(centre,target)/.8))
        motion=min(4.,max(1.,math.dist(centre,target)/candidate_speed))
        poses,vel,t,angles=rollout_crab(robot.x,robot.y,robot.theta,0.,beta,[candidate_sign*candidate_speed],[beta],motion,params=cp,delay=.1)
        tail,tv,tt,tb=rollout_crab(*poses[0,-1],float(vel[0,-1]),beta,[0.],[beta],1.,params=cp,delay=0.,initial_sign=candidate_sign)
        poses=np.concatenate((poses,tail[:,1:]),axis=1);t=np.r_[t,motion+tt[1:]]
        wt=np.arange(0.,wait,.05);stationary=np.tile([robot.x,robot.y,robot.theta],(len(wt),1))
        entire=np.concatenate((stationary,poses[0]));times=np.r_[wt,wait+t]
        gap=float(minimum_gaps(to_centres(entire,self.geometry),times,obs,self.geometry,self.obstacle_age,.025*cp.v_max+.002))
        if gap<getattr(self.params,'safety_margin',.12):return None
        progress=math.dist(centre,target)-math.dist(to_centres(poses,self.geometry)[0,-1,:2],target)
        if progress<min(.15,.4*math.dist(centre,target)):return None
        return dict(mode=1,beta=beta,direction=candidate_sign,time_s=wait+math.dist(centre,target)/cp.v_max,gap=gap,progress=progress,guide_target=tuple(target))
    def continue_reverse(self,robot,obs):
        reference=getattr(self,'_reverse_reference',None)
        if not reference:return None
        best=(math.inf,0.)
        elapsed=getattr(robot,'planning_stamp',0.)-self._reverse_started
        for a,b in zip(reference,reference[1:]):
            if a[4]>elapsed+.5:break
            dx,dy=b[0]-a[0],b[1]-a[1];l2=dx*dx+dy*dy
            u=max(0.,min(1.,((robot.x-a[0])*dx+(robot.y-a[1])*dy)/max(l2,1e-12)))
            dist=math.hypot(robot.x-a[0]-u*dx,robot.y-a[1]-u*dy)
            t=a[4]+u*(b[4]-a[4])
            if t>=getattr(self,'_reverse_progress',0.)-.15 and dist<best[0]:best=(dist,t)
        progress=max(getattr(self,'_reverse_progress',0.),best[1]);self._reverse_progress=progress
        end=reference[-1]
        if robot.v<.035 and math.hypot(robot.x-end[0],robot.y-end[1])<.1 and abs((robot.theta-end[2]+math.pi)%(2*math.pi)-math.pi)<math.radians(5):
            self._reverse_reference=None;self._reverse_schedule=None;return None
        from .reverse_tracking_forecast import forecast
        predicted,vel,times=forecast(reference,robot.x,robot.y,robot.theta,robot.v,self.current_steer,
            getattr(robot,'command_speed',-robot.v),-self.current_acc,self.current_steer_rate,self.geometry,
            phase=max(0.,elapsed),rate=getattr(self.params,'command_steer_rate',math.radians(45)))
        centres=to_centres(predicted,self.geometry)
        gap=float(minimum_gaps(centres,times,obs,self.geometry,self.obstacle_age,.04)[0])
        if gap<getattr(self.params,'safety_margin',.12):return None
        # Keep the ORIGINAL phase geometry; a noisy replan must not restart the
        # turn-out phase and postpone countersteering indefinitely.
        self.reference=reference;self.telemetry=dict(reason='motion',motion_mode='ACK',reverse=True,committed_refuge=True,latched_macro=True,macro_start=self._reverse_started,min_gap=gap,forecast_progress=0.)
        return True,reference[0][3],reference[0][3]*math.tan(reference[0][5])/self.geometry.wheelbase
    def reverse_candidate(self,robot,obs):
        from .signed_ack import rollout_reverse
        ts=np.linspace(0.,7.,141);vs=[];ds=[];continuation=False
        stamp=getattr(robot,'planning_stamp',0.)
        if self._reverse_schedule is not None:
            schedule=self._reverse_schedule;elapsed=stamp-schedule['stamp']
            if elapsed<7.:
                vs.append(np.interp(ts+elapsed,ts,schedule['v'],right=0.));ds.append(np.interp(ts+elapsed,ts,schedule['d'],right=0.));continuation=True
            else:self._reverse_schedule=None
        if robot.v>.035 and not continuation:return None
        for amplitude in (math.radians(20),math.radians(25)):
            for cruise in (.6,.8,1.):
                for turn in (1.8,2.1,2.4,2.7):
                    for brake in (3.2,3.6,4.,4.4):
                        for side in (-1,1):
                            vs.append(np.where(ts<brake,cruise,0.));ds.append(np.where(ts<turn,side*amplitude,np.where(ts<2*turn,-side*amplitude,0.)))
        poses,vel,tt,angles=rollout_reverse(robot.x,robot.y,robot.theta,robot.v,self.current_steer,vs,ds,self.geometry,initial_acc=-self.current_acc,initial_rate=self.current_steer_rate,delay=getattr(self.params,"actuator_delay_s",.1),rate=getattr(self.params,'command_steer_rate',math.radians(45)))
        centres=to_centres(poses,self.geometry);sweep=np.maximum(vel,np.c_[vel[:,1:],vel[:,-1]])*(1.+math.tan(math.pi/6)/.69*self.geometry.radius)*.025+.002
        gaps=minimum_gaps(centres,tt,obs,self.geometry,self.obstacle_age,sweep)
        refuge=self.ack._future_refuge(centres,tt,obs,.22)
        heading=np.abs((centres[:,-1,2]-robot.theta+math.pi)%(2*math.pi)-math.pi)<math.radians(15)
        safe=(gaps>=.22)&refuge&(vel[:,-1]<.035)&heading
        if not np.any(safe):return None
        scores=np.where(safe,gaps-.02*np.linalg.norm(centres[:,-1,:2]-centres[:,0,:2],axis=1),-np.inf)
        if continuation and safe[0]:scores[0]+=100.
        from .reverse_tracking_forecast import forecast
        k=None;reference=None
        for index in np.argsort(scores)[-8:][::-1]:
            if not math.isfinite(float(scores[index])):continue
            candidate=[(float(q[0]),float(q[1]),float(q[2]),-float(vt),float(t),float(bt)) for q,vt,t,bt in zip(poses[index],vs[index],tt,ds[index])]
            predicted,pv,pt=forecast(candidate,robot.x,robot.y,robot.theta,robot.v,self.current_steer,
                getattr(robot,'command_speed',-robot.v),-self.current_acc,self.current_steer_rate,self.geometry,
                rate=getattr(self.params,'command_steer_rate',math.radians(45)))
            pg=float(minimum_gaps(to_centres(predicted,self.geometry),pt,obs,self.geometry,self.obstacle_age,.04)[0])
            if pg>=.18 and self.ack._future_refuge(to_centres(predicted,self.geometry),pt,obs,.18)[0]:
                k=int(index);reference=candidate;break
        if k is None:return None
        v=float(vs[k][0]);b=float(ds[k][0])
        if not(continuation and k==0):self._reverse_schedule=dict(v=vs[k],d=ds[k],stamp=stamp)
        self._reverse_reference=reference;self._reverse_started=stamp;self._reverse_progress=0.
        return -v,b,reference,float(gaps[k])
    def plan(self,robot,goal,obs,trap=None):
        if not hasattr(self,'_task_yaws'):self._task_yaws={}
        if tuple(goal) not in self._task_yaws:self._task_yaws[tuple(goal)]=robot.theta
        self._task_goal=tuple(goal);self._task_yaw=self._task_yaws[tuple(goal)]
        if self.current_mode==1:return self.plan_crab(robot,goal,obs)
        if getattr(self,'_reverse_reference',None) is not None:
            continued=self.continue_reverse(robot,obs)
            if continued is not None:return continued
        a=self.ack;a.current_steer=self.current_steer;a.obstacle_age=self.obstacle_age;a.current_acc=self.current_acc;a.current_steer_rate=self.current_steer_rate
        centre=np.array([robot.x+.345*math.cos(robot.theta),robot.y+.345*math.sin(robot.theta)])
        forward=np.array([math.cos(robot.theta),math.sin(robot.theta)])
        approaching=[o for o in obs if not o.uncertain and o.speed>.1 and (np.array([o.x,o.y])-centre)@forward>.3 and np.array([o.vx,o.vy])@forward<-.15 and abs((np.array([o.x,o.y])-centre)@np.array([-forward[1],forward[0]]))<1.4+o.width/2]
        if self._reverse_schedule is None and approaching:
            answer=a.plan(robot,goal,obs,trap)
            if answer[0] and a.telemetry.get('min_gap',0.)>=getattr(self.params,'dynamic_forward_margin',.4):
                self.reference=a.reference;self.telemetry=dict(a.telemetry,motion_mode='ACK');return answer
        if self._reverse_schedule is not None or approaching:
            reverse=self.reverse_candidate(robot,obs)
            if reverse is not None:
                v,b,reference,gap=reverse
                self.reference=reference;self.telemetry=dict(reason='motion',motion_mode='ACK',reverse=True,committed_refuge=True,macro_start=self._reverse_started,selected_delta=b,min_gap=gap,forecast_progress=0.,guide_target=goal)
                return True,v,v*math.tan(b)/self.geometry.wheelbase
            if self._reverse_schedule is not None:
                self.reference=[];self.telemetry=dict(reason='dynamic_wait',motion_mode='ACK',reverse=True,forecast_progress=0.,guide_target=goal)
                return True,0.,0.
        local=self.guide(robot,goal,obs)
        # Feed the same local waypoint to ACK and CRAB; the timed physical
        # collision model remains authoritative, not this geometric guide.
        if not getattr(self,'_guide_found',True):
            from .local_route import route_waypoint
            from .motion_safety import spin_is_safe
            desired=math.atan2(goal[1]-centre[1],goal[0]-centre[0])
            yaw_choice=None
            for yaw in (self._task_yaw,self._task_yaw+math.pi/2,self._task_yaw-math.pi/2,desired,desired+math.pi/2,desired-math.pi/2):
                error=abs((yaw-robot.theta+math.pi)%(2*math.pi)-math.pi)
                if error<math.radians(8):continue
                _,found=route_waypoint(centre,yaw,goal,obs,self.geometry,getattr(self.params,'safety_margin',.12))
                if found and spin_is_safe(*centre,robot.theta,yaw,obs,self.geometry,self.obstacle_age):
                    yaw_choice=yaw;break
            self.reference=[];
            if yaw_choice is not None:
                self.telemetry=dict(reason='blocked',route_yaw=yaw_choice,guide_target=goal,motion_mode='ACK',forecast_progress=0.)
                return False,0.,0.
            self.telemetry=dict(reason='blocked',guide_no_route=True,guide_target=goal,motion_mode='ACK',forecast_progress=0.)
            return False,0.,0.
        a.external_guide_target=local
        answer=a.plan(robot,goal,obs,trap);self.reference=a.reference;self.telemetry=dict(a.telemetry,motion_mode='ACK');return answer
    def set_last_cmd(self,v,w):self.ack.set_last_cmd(v,w)
