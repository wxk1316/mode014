"""Single spawned planning process; main ROS callbacks never wait for a plan."""
import copy,time,multiprocessing,queue,math

def _work(input_queue,output_queue,planner):
    output_queue.put(('ready',None))
    while True:
        item=input_queue.get()
        if item is None:return
        seq,stamp,robot,goal,obs,trap,steer,age=item
        try:
            started=time.monotonic();planner.current_steer=steer;planner.obstacle_age=age
            result=planner.plan(robot,goal,obs,trap)
            packet=(seq,stamp,robot,goal,result,planner.reference,dict(planner.telemetry,worker_duration_s=time.monotonic()-started,sampled_pose=[robot.x,robot.y,robot.theta],sampled_velocity=robot.v,sampled_steer=steer,sampled_obstacle_age_s=age))
        except Exception as exc:packet=(seq,stamp,robot,goal,(False,0.,0.),[],{'reason':'planning_exception','error':type(exc).__name__})
        output_queue.put(('result',packet))

class AsyncPlanner:
    def __init__(self,planner,max_age=.3):
        self.base=planner;self.max_age=max_age;self.reference=[];self.telemetry={}
        self.current_steer=0.;self.obstacle_age=0.;self._pending=False;self._ready=False;self._seq=0;self._result=None
        ctx=multiprocessing.get_context('spawn');self._input=ctx.Queue(1);self._output=ctx.Queue(2)
        self._process=ctx.Process(target=_work,args=(self._input,self._output,planner),daemon=True);self._process.start()
    def __getattr__(self,name):return getattr(self.base,name)
    def plan(self,robot,goal,obs,trap=None):
        now=time.monotonic()
        while True:
            try:kind,payload=self._output.get_nowait()
            except queue.Empty:break
            if kind=='ready':self._ready=True
            else:self._pending=False;self._result=payload
        if self._ready and not self._pending and self._process.is_alive():
            self._seq+=1;item=(self._seq,now,copy.copy(robot),tuple(goal),copy.deepcopy(obs),copy.deepcopy(trap),self.current_steer,self.obstacle_age)
            try:self._input.put_nowait(item);self._pending=True
            except queue.Full:pass
        self.reference=[]
        if not self._process.is_alive():self.telemetry={'reason':'planning_worker_failed'};return False,0.,0.
        if self._result is None:self.telemetry={'reason':'planning_pending','worker_ready':self._ready,'request_sequence':self._seq,'pending':self._pending};return False,0.,0.
        seq,stamp,pose,old_goal,result,reference,telemetry=self._result
        err=(robot.theta-pose.theta+math.pi)%(2*math.pi)-math.pi
        age=now-stamp
        if old_goal!=tuple(goal) or age>self.max_age+1e-6 or math.hypot(robot.x-pose.x,robot.y-pose.y)>.18 or abs(err)>math.radians(12):
            self.telemetry=dict(telemetry,reason='planning_result_stale',result_age_s=age,sequence=seq,goal_changed=old_goal!=tuple(goal),pose_displacement_m=math.hypot(robot.x-pose.x,robot.y-pose.y),yaw_change_rad=err,pending=self._pending);return False,0.,0.
        self.reference=reference;self.telemetry=dict(telemetry,result_age_s=age,sequence=seq)
        return result
    def close(self):
        if self._process.is_alive():
            try:self._input.put_nowait(None)
            except queue.Full:pass
            self._process.join(timeout=.2)
            if self._process.is_alive():self._process.terminate();self._process.join(timeout=.2)
        self._input.close();self._output.close()
