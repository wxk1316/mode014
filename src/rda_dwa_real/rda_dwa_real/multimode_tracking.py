"""Version-2 tracker/session execution, reusing ACK LQR and spin gains.
Driver READY is the only mechanical completion signal. The tracker separately
confirms body stop and accounts for zero-speed angle preparation.
"""
import math
import numpy as np
from rda_dwa.crab_model import CrabParams,rollout_crab
from rda_dwa.motion_safety import speed_step,steering_step,to_centres,minimum_gaps

MODES={'ACK':0,'CRAB':1,'SPIN':4}

class MultimodeTrackerMixin:
    def init_multimode(self):
        self._motion_sign=1.;self._v2=False;self._v2_mode=0;self._v2_desired=0;self._v2_phase='RUN'
        self._v2_pending=None;self._v2_at=0.;self._v2_stop_at=None;self._v2_prepare=0.
        self._spin_stop_since=None;self._driver_state=None;self._driver_at=-1e9;self._v2_epoch=-1;self._mode_echo_rx=-1e9
    def set_driver_state(self,state,now):
        actual=int(state.get('actual_mode',-1))
        if actual not in (0,1,4):return
        self._driver_state=dict(state);self._driver_at=now
        self._v2_mode=actual
    def set_plan(self,plan,now,seq):
        mode=str(plan.get('motion_mode','ACK'))
        if mode not in MODES:raise ValueError('unknown motion_mode')
        epoch=int(plan.get('mode_epoch',0))
        if epoch<self._v2_epoch:return
        if epoch>self._v2_epoch:self._spin_result=None;self._spin_stop_since=None
        self._last_v2_reason=plan.get('reason','');self._v2=True;self._v2_epoch=epoch;self._session=int(plan.get('spin_session',epoch))
        self._committed_refuge=bool(plan.get('planner',{}).get('committed_refuge'));self._v2_desired=MODES[mode];self._v2_prepare=float(plan.get('prepare_angle_rad',0.))
        self._spin_precision=float(plan.get('spin_tolerance_deg',self.p.rotate_precision_deg))
        if not math.isfinite(self._spin_precision) or not 0.5<=self._spin_precision<=8.:raise ValueError('spin precision')
        self._crab_yaw_ref=float(plan.get('body_yaw_ref',self._pyaw))
        if not math.isfinite(self._crab_yaw_ref):raise ValueError('body yaw reference')
        self._spin_target=float(plan.get('spin_target',self._pyaw))
        if self._v2_phase in ('BRAKE','RESET') and self._v2_desired==self._v2_mode:
            self._v2_pending=self._v2_mode
        if not math.isfinite(self._v2_prepare) or abs(self._v2_prepare)>math.pi/2+1e-8:raise ValueError('invalid angle')
        if self._v2_phase=='RUN' and self._v2_desired==self._v2_mode:
            self.mode=mode
            reference=plan.get('road_base',[])
            if reference:
                previous=self.mode;self.mode=self.MODE_ACK
                self.set_reference(reference,now,seq);self.mode=previous
                self._reverse_ref=int(plan.get('direction',-1 if math.copysign(1.,reference[0][3])<0 else 1))<0
                if self._reverse_ref:
                    self._reverse_raw_reference=[tuple(q) for q in reference]
                    self._reverse_macro_start=float(plan.get('planner',{}).get('macro_start',now))
                if self._v2_mode in (0,1) and abs(self._last_v_out)>.02 and self._reverse_ref!=(self._last_v_out<0):
                    self._v2_pending=0;self._v2_phase='BRAKE';self._v2_at=now;self._v2_stop_at=None;self.clear_path('direction_change_stop')

            else:
                self.clear_path(plan.get('reason','empty_plan'))
    def _mode_output(self,mode,v,angle):
        if abs(v)>.001 and mode!=4:self._motion_sign=math.copysign(1.,v)
        self._last_v_out=float(v);self._last_steer=float(angle)
        return dict(enable=1,turnmode=int(mode),speed=float(v),angle=math.degrees(angle) if mode!=4 else 0.)
    def _v2_stop(self,dt,angle=None,mode=None):
        source=self._v2_mode if mode is None else mode
        if source==4:
            self._spin_w+=max(-self.p.spin_acc_max*dt,min(self.p.spin_acc_max*dt,-self._spin_w))
            return self._mode_output(4,self._spin_w,0.)
        sign=self._motion_sign
        v,a=speed_step(abs(self._last_v_out),self._acc,0.,dt,self.p.acc_max,self.p.dec_max,self.p.jerk_max)
        self._acc=float(a)
        return self._mode_output(source,sign*float(v),self._last_steer if angle is None else angle)
    def _v2_stopped(self,now):
        return self._velocity_estimator.valid and now-self._pose_at<=self.p.localization_timeout and self._stopped(now)
    def _step_v2(self,now,dt):
        if self._safety_reason:
            self._acc=0.;self._spin_w=0.;self._v2_stop_at=None;self._spin_stop_since=None
            if self._v2_phase=='RESET':self._v2_phase='BRAKE'
            self._diag={'branch':'MULTIMODE_SAFETY','reason':self._safety_reason}
            return self._mode_output(self._v2_mode,0.,self._last_steer)
        if self._driver_state is None or now-self._driver_at>self.p.driver_feedback_timeout:
            self._acc=0.;self._spin_w=0.;self._v2_stop_at=None;self._spin_stop_since=None
            if self._v2_phase=='RESET':self._v2_phase='BRAKE'
            self._diag={'branch':'DRIVER_STALE'}
            return self._mode_output(self._v2_mode,0.,self._last_steer)
        if self._v2_phase=='RUN' and self._v2_desired!=self._v2_mode:
            self._v2_pending=self._v2_desired;self._v2_phase='BRAKE';self._v2_at=now;self._v2_stop_at=None
            self.clear_path('mode_transition');self._spin_result=None
        if self._v2_phase!='RUN':
            self.mode='SWITCH';self._diag={'branch':'MODE_'+self._v2_phase,'target_mode':self._v2_pending}
            if now-self._v2_at>self.p.mode_switch_timeout:
                self._safety_reason='mode_switch_timeout';return self._mode_output(self._v2_mode,0.,self._last_steer)
            if self._v2_phase=='BRAKE':
                stopped=self._v2_stopped(now)
                self._v2_stop_at=(now if self._v2_stop_at is None else self._v2_stop_at) if stopped else None
                if self._v2_stop_at is not None and now-self._v2_stop_at>=.3:
                    self._v2_phase='RESET';self._zero_at=now;self._zero_duration=abs(self._last_steer)/math.radians(self.p.crab_beta_rate_deg)+.2
                return self._v2_stop(dt)
            if self._v2_phase=='RESET':
                if not self._v2_stopped(now):
                    self._v2_phase='BRAKE';self._v2_stop_at=None;return self._v2_stop(dt)
                angle,rate=steering_step(self._last_steer,self._steer_rate,0.,dt,math.radians(self.p.crab_beta_rate_deg),math.radians(self.p.max_steer_accel_deg))
                self._steer_rate=float(rate)
                if now-self._zero_at>=self._zero_duration and abs(angle)<math.radians(.5):
                    self._v2_phase='WAIT_DRIVER';self._prepare_at=now;self._prepared_target=self._v2_prepare if self._v2_pending==1 else 0.
                    self._last_steer=0.;self._steer_rate=0.
                return self._mode_output(self._v2_mode,0.,float(angle))
            target=self._v2_pending
            angle,rate=steering_step(self._last_steer,self._steer_rate,self._prepared_target,dt,math.radians(self.p.crab_beta_rate_deg),math.radians(self.p.max_steer_accel_deg))
            self._steer_rate=float(rate)
            state=self._driver_state
            stopped=self._v2_stopped(now)
            self._v2_stop_at=(now if self._v2_stop_at is None else self._v2_stop_at) if stopped else None
            if self._v2_stop_at is not None and now-self._v2_stop_at>=.3 and state.get('ready') and int(state.get('actual_mode',-1))==target and now-self._prepare_at>=abs(self._prepared_target)/math.radians(self.p.crab_beta_rate_deg)+.25 and self._v2_stopped(now):
                self._v2_phase='RUN';self._v2_pending=None;self.mode={0:'ACK',1:'CRAB',4:'SPIN'}[target]
                self._last_v_out=self._acc=self._spin_w=0.;self._spin_result=None;self.clear_path('mode_ready_replan')
            return self._mode_output(target,0.,float(angle))
        self.mode={0:'ACK',1:'CRAB',4:'SPIN'}[self._v2_mode]
        if not self._driver_state.get('ready'):
            self._spin_stop_since=None;self._v2_stop_at=None
            self._diag={'branch':'MODE_NOT_READY'};return self._mode_output(self._v2_mode,0.,self._last_steer)
        if self._v2_mode==4:
            err=(self._spin_target-self._pyaw+math.pi)%(2*math.pi)-math.pi
            wanted=math.copysign(min(self.p.spin_w_max,self.p.spin_w_max*abs(err)/math.radians(self.p.rotate_decel_deg)),err) if abs(err)>math.radians(getattr(self,'_spin_precision',self.p.rotate_precision_deg)) else 0.
            self._spin_w+=max(-self.p.spin_acc_max*dt,min(self.p.spin_acc_max*dt,wanted-self._spin_w))
            horizon=max(.8,abs(self._yaw_rate)/self.p.spin_acc_max+.3)
            times=np.arange(0.,horizon+.025,.05);yaw=self._pyaw;rate=self._yaw_rate;poses=[]
            for t in times:
                poses.append([self._centre[0],self._centre[1],yaw])
                rate+=max(-self.p.spin_acc_max*.05,min(self.p.spin_acc_max*.05,self._spin_w-rate));yaw+=rate*.05
            gap=float(minimum_gaps(np.asarray(poses),times,self._obstacles,self.geometry,now-self._obstacle_at,max(abs(self._spin_w),abs(self._yaw_rate))*self.geometry.radius*.025+.002))
            if gap<self.p.safety_margin:
                self._diag={'branch':'SPIN_EXECUTION_GUARD','min_predicted_gap':gap};return self._mode_output(4,0.,0.)
            settled=abs(err)<=math.radians(getattr(self,'_spin_precision',self.p.rotate_precision_deg)) and abs(self._spin_w)<.03 and self._v2_stopped(now)
            self._spin_stop_since=(now if self._spin_stop_since is None else self._spin_stop_since) if settled else None
            if self._spin_stop_since is not None and now-self._spin_stop_since>=self.p.spin_stop_hold:self._spin_result='reached'
            self._diag={'branch':'SPIN_RUN','spin_err':err};return self._mode_output(4,self._spin_w,0.)
        if self._v2_mode==1 and self._last_v2_reason=='crab_prepare' and self._v2_stopped(now):
            angle,rate=steering_step(self._last_steer,self._steer_rate,self._v2_prepare,dt,math.radians(self.p.crab_beta_rate_deg),math.radians(self.p.max_steer_accel_deg))
            self._steer_rate=float(rate);self._diag={'branch':'CRAB_PREPARE'};return self._mode_output(1,0.,float(angle))
        if len(self._ref)<2 or now-self._last_traj_ts>self.p.traj_timeout:
            self._diag={'branch':'MODE_NO_REF'};return self._v2_stop(dt)
        if self._v2_mode==0 and getattr(self,'_reverse_ref',False):return self._step_reverse(now,dt)
        if self._v2_mode==0:
            self._v2_bypass=True
            try:return self.step(now,dt)
            finally:self._v2_bypass=False
        return self._step_crab(now,dt)
    def _step_crab(self,now,dt):
        s=self._project();xr,yr,theta,vr,_=self._eval_at_s(s)
        i=min(len(self._ref)-1,max(1,self._proj_idx+1));a,b=self._ref[i-1],self._ref[i]
        chi=math.atan2(b['y']-a['y'],b['x']-a['x'])
        # Endpoint derivative comes from commanded beta, not body yaw.
        beta=self._target_steer_at_s(s,chi-self._pyaw)
        ref_sign=-1. if getattr(self,'_reverse_ref',False) else 1.
        world=ref_sign*np.array([vr*math.cos(theta+beta),vr*math.sin(theta+beta)])+self.p.crab_position_gain*np.array([xr-self._px,yr-self._py])
        c,ss=math.cos(self._pyaw),math.sin(self._pyaw)
        bx=c*world[0]+ss*world[1];by=-ss*world[0]+c*world[1]
        sign=self._motion_sign if abs(bx)<.025 else math.copysign(1.,bx)
        wanted=min(self.p.crab_v_max,math.hypot(bx,by))
        target=math.atan2(sign*by,max(0.,sign*bx)) if wanted>.01 else self._last_steer
        if abs(self._last_v_out)>.02 and sign!=self._motion_sign:
            self._diag={'branch':'CRAB_DIRECTION_BRAKE'};return self._v2_stop(dt)
        yaw_error=abs((getattr(self,'_crab_yaw_ref',theta)-self._pyaw+math.pi)%(2*math.pi)-math.pi)
        if yaw_error>math.radians(self.p.crab_yaw_tolerance_deg):
            self._diag={'branch':'CRAB_YAW_FAULT'};return self._v2_stop(dt)
        angle,rate=steering_step(self._last_steer,self._steer_rate,target,dt,math.radians(self.p.crab_beta_rate_deg),math.radians(self.p.max_steer_accel_deg));angle=float(np.clip(angle,-math.pi/2,math.pi/2));self._steer_rate=float(rate)
        if abs(target-angle)>math.radians(12):wanted=min(wanted,.12)
        v,a=speed_step(abs(self._last_v_out),self._acc,wanted,dt,self.p.acc_max,self.p.dec_max,self.p.jerk_max);self._acc=float(a)
        cp=CrabParams(v_max=self.p.crab_v_max,beta_rate=math.radians(self.p.crab_beta_rate_deg))
        poses,_,tt,_=rollout_crab(self._px,self._py,self._pyaw,self._v_meas,self._last_steer,[sign*float(v)],[angle],max(.8,self._v_meas/self.p.dec_max+.6),params=cp,initial_acc=self._acc,initial_beta_rate=self._steer_rate,initial_sign=self._motion_sign)
        gap=float(minimum_gaps(to_centres(poses,self.geometry),tt,self._obstacles,self.geometry,now-self._obstacle_at,max(self._v_meas,float(v))*.025+.002)[0])
        stop,_,st,_=rollout_crab(self._px,self._py,self._pyaw,self._v_meas,self._last_steer,[0.],[self._last_steer],max(1.5,self._v_meas/self.p.dec_max+.9),params=cp,initial_acc=self._acc,initial_beta_rate=self._steer_rate,initial_sign=self._motion_sign)
        gap=min(gap,float(minimum_gaps(to_centres(stop,self.geometry),st,self._obstacles,self.geometry,now-self._obstacle_at,max(self._v_meas,float(v))*.025+.002)[0]))
        if gap<self.p.safety_margin:
            self._diag={'branch':'CRAB_EXECUTION_GUARD','min_predicted_gap':gap};return self._mode_output(1,0.,self._last_steer)
        self._diag={'branch':'CRAB','e_y':math.hypot(xr-self._px,yr-self._py),'e_th':yaw_error,'min_predicted_gap':gap,'v_ref':vr,'beta':angle}
        return self._mode_output(1,sign*float(v),angle)

    def _step_reverse(self,now,dt):
        from rda_dwa.signed_ack import rollout_reverse
        s=self._project();xr,yr,theta,vr,_=self._eval_at_s(s)
        raw=np.asarray(self._reverse_raw_reference,float);phase=max(0.,now-self._reverse_macro_start)
        ff=float(np.interp(phase,raw[:,4],raw[:,5]));vr=abs(float(np.interp(phase,raw[:,4],raw[:,3])))
        remaining=self._s_arr[-1]-s
        if vr<.01 and remaining>.08:
            vr=min(.4,math.sqrt(2.*self.p.dec_max*max(0.,remaining-.05)))
            ff=self._target_steer_at_s(s,0.)
        err=(self._pyaw-theta+math.pi)%(2*math.pi)-math.pi
        ey=-(self._px-xr)*math.sin(theta)+(self._py-yr)*math.cos(theta)
        correction=-self.p.reverse_position_gain*ey+self.p.reverse_heading_gain*err
        correction=float(np.clip(correction,-math.radians(self.p.feedback_limit_deg),math.radians(self.p.feedback_limit_deg)))
        target=float(np.clip(ff+correction,-math.pi/6,math.pi/6))
        angle=self._steer(target,dt)
        v,a=speed_step(abs(self._last_v_out),self._acc,min(vr,self.p.v_max),dt,self.p.acc_max,self.p.dec_max,self.p.jerk_max);self._acc=float(a)
        from rda_dwa.reverse_tracking_forecast import forecast
        reference=self._reverse_raw_reference
        poses,_,tt=forecast(reference,self._px,self._py,self._pyaw,self._v_meas,self._last_steer,
            self._last_v_out,self._acc,self._steer_rate,self.geometry,horizon=max(1.5,self._v_meas/1.2+.9),phase=phase,
            rate=math.radians(self.p.max_steer_rate_deg),ky=self.p.reverse_position_gain,kth=self.p.reverse_heading_gain)
        gap=float(minimum_gaps(to_centres(poses,self.geometry),tt,self._obstacles,self.geometry,now-self._obstacle_at,max(self._v_meas,float(v))*.05+.002)[0])
        stop,_,st,_=rollout_reverse(self._px,self._py,self._pyaw,self._v_meas,self._last_steer,[0.],[self._last_steer],self.geometry,horizon=max(1.5,self._v_meas/1.2+.9),initial_acc=self._acc,initial_rate=self._steer_rate,rate=math.radians(self.p.max_steer_rate_deg))
        brake_gap=float(minimum_gaps(to_centres(stop,self.geometry),st,self._obstacles,self.geometry,now-self._obstacle_at,max(self._v_meas,float(v))*.05+.002)[0])
        if not getattr(self,'_committed_refuge',False):gap=min(gap,brake_gap)
        if gap<self.p.safety_margin:
            self._diag={'branch':'REVERSE_GUARD','min_predicted_gap':gap};return self._mode_output(0,0.,self._last_steer)
        self._diag={'branch':'ACK_REVERSE','braking_backup_gap':brake_gap,'validated_refuge_continuation':getattr(self,'_committed_refuge',False),'e_th':err,'min_predicted_gap':gap,'v_ref':-vr}
        return self._mode_output(0,-float(v),angle)
