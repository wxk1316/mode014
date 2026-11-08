"""Bounded, asynchronous diagnostics. No ROS imports or control publications."""
from __future__ import annotations
import collections,hashlib,json,math,os,pathlib,platform,queue,shutil,signal,subprocess,sys,threading,time

SCHEMA_VERSION=2
BAG_TOPICS=['/rslidar_points','/imu/data','/localization','/tf','/tf_static','/move_base_simple/goal','/rda/obstacles','/rda/plan','/rda/diag','/rda/perception_diag','/rda/control_diag','/rda/session_state','/car_cmd','/car_state','/car_driver/diag','/car_driver/io','/parameter_events','/cmd_vel','/rda/traj','/rda/traj_mode','/rda/traj_clear','/rda/spin_target','/car_bridge_state','/rda/ctrl_status','/rda/status','/rda/test_note']

def safe_data(value):
    """ROS fields -> strict JSON primitives, preserving all small-message fields."""
    if value is None or isinstance(value,(str,bool,int)):return value
    if isinstance(value,float):return value if math.isfinite(value) else None
    if isinstance(value,dict):return {str(k):safe_data(v) for k,v in value.items()}
    if isinstance(value,(list,tuple,bytes,bytearray)):return [safe_data(v) for v in value]
    if hasattr(value,'get_fields_and_field_types'):
        return {k:safe_data(getattr(value,k)) for k in value.get_fields_and_field_types()}
    if hasattr(value,'tolist'):return safe_data(value.tolist())
    if hasattr(value,'__iter__'):return [safe_data(v) for v in value]
    if hasattr(value,'__dict__'):return {k:safe_data(v) for k,v in vars(value).items() if not k.startswith('_')}
    return str(value)

def source_header(message):
    h=getattr(message,'header',None);s=getattr(h,'stamp',None)
    ns=int(s.sec)*10**9+int(s.nanosec) if s is not None else None
    return ns if ns else None,getattr(h,'frame_id',None)

def parameter_value(value):
    fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
    return safe_data(getattr(value,fields[value.type])) if value.type in fields else None

class AsyncJsonlWriter:
    """Callback only serializes and enqueues; one thread owns all open files.

    Limits apply to record count AND serialized bytes. Critical events have a
    separate bounded reserve. Each accepted record retains its arrival sequence,
    because writing across files is not a synchronization guarantee.
    """
    def __init__(self,root,max_records=4096,max_bytes=32*1024**2,rotate_bytes=64*1024**2,min_free_bytes=512*1024**2,flush_interval=1.):
        self.root=pathlib.Path(root);self.root.mkdir(parents=True,exist_ok=True)
        self.max_records=max_records;self.max_bytes=max_bytes;self.rotate_bytes=rotate_bytes;self.min_free_bytes=min_free_bytes;self.flush_interval=flush_interval
        self._normal=collections.deque();self._critical=collections.deque();self._bytes=0;self._critical_bytes=0;self._condition=threading.Condition();self._closing=False;self._files={};self._sizes={};self._segments={};self._last_flush=time.monotonic();self._last_disk=0.;self._disk_ok=True
        self._stats=dict(accepted=0,written=0,dropped=0,queue_high_water=0,bytes_high_water=0,bytes_written=0,write_errors=0,last_error=None,free_bytes=None)
        self._thread=threading.Thread(target=self._run,name='diag-file-writer',daemon=True);self._thread.start()
    def submit(self,name,row,critical=False):
        if pathlib.Path(name).name!=name:raise ValueError('invalid output name')
        data=(json.dumps(safe_data(row),ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n').encode()
        with self._condition:
            full=len(self._normal)>=self.max_records or self._bytes+len(data)>self.max_bytes
            if self._closing or (critical and (len(self._critical)>=256 or self._critical_bytes+len(data)>1024**2)) or (not critical and full):self._stats['dropped']+=1;return False
            (self._critical if critical else self._normal).append((name,data));self._bytes+=len(data);self._stats['accepted']+=1
            if critical:self._critical_bytes+=len(data)
            self._stats['queue_high_water']=max(self._stats['queue_high_water'],len(self._normal)+len(self._critical));self._stats['bytes_high_water']=max(self._stats['bytes_high_water'],self._bytes);self._condition.notify();return True
    def stats(self):
        with self._condition:return dict(self._stats,queue_records=len(self._normal)+len(self._critical),queue_bytes=self._bytes,alive=self._thread.is_alive())
    def _handle(self,name):
        # Legacy snapshots/events remain single files. Full-message files rotate.
        if name=='messages.jsonl':
            segment=self._segments.setdefault(name,1)
            if self._sizes.get(name,0)>=self.rotate_bytes:
                self._files.pop(name).close();segment+=1;self._segments[name]=segment;self._sizes[name]=0
            path=self.root/f'messages_{segment:06d}.jsonl'
        else:path=self.root/name
        if name not in self._files:self._files[name]=open(path,'ab',buffering=256*1024)
        return self._files[name]
    def _flush(self):
        for handle in self._files.values():handle.flush()
        self._last_flush=time.monotonic()
    def _run(self):
        try:
            while True:
                with self._condition:
                    if not self._normal and not self._critical and not self._closing:self._condition.wait(timeout=self.flush_interval)
                    if self._critical:item=self._critical.popleft();self._critical_bytes-=len(item[1])
                    elif self._normal:item=self._normal.popleft()
                    elif self._closing:break
                    else:item=None
                    if item:self._bytes-=len(item[1])
                now=time.monotonic()
                try:
                    if now-self._last_disk>5:
                        free=shutil.disk_usage(self.root).free;self._stats['free_bytes']=free;self._disk_ok=free>=self.min_free_bytes;self._last_disk=now
                    if item:
                        if not self._disk_ok:raise OSError('diagnostic disk reserve reached')
                        name,data=item;self._handle(name).write(data);self._sizes[name]=self._sizes.get(name,0)+len(data);self._stats['written']+=1;self._stats['bytes_written']+=len(data)
                    if now-self._last_flush>=self.flush_interval:self._flush()
                except OSError as exc:
                    self._stats['write_errors']+=1;self._stats['last_error']=str(exc)
                    if item:self._stats['dropped']+=1
        finally:
            try:self._flush()
            except OSError:pass
            for h in self._files.values():h.close()
    def close(self,timeout=3.):
        with self._condition:self._closing=True;self._condition.notify_all()
        self._thread.join(timeout);status=dict(self.stats(),drained=not self._thread.is_alive())
        # Shutdown only: this path never executes inside a message callback.
        if status['drained']:
            try:(self.root/'writer_summary.json').write_text(json.dumps(status,indent=2))
            except OSError:pass
        return status

class BagCapture:
    """Optional child rosbag process, independent from control. No shell.

    Probe the installed CLI before selecting options. Quotas stop only the bag;
    JSON diagnostics continue. A missing rosbag is explicitly recorded.
    """
    def __init__(self,root,enabled=True,topics=None,total_bytes=8*1024**3,min_free_bytes=2*1024**3,segment_bytes=512*1024**2):
        self.root=pathlib.Path(root);self.enabled=enabled;self.topics=topics or BAG_TOPICS;self.total_bytes=total_bytes;self.min_free_bytes=min_free_bytes;self.segment_bytes=segment_bytes;self.process=None;self.console=None;self.state='disabled';self.error=None;self._stop_reason=None;self._stop_at=None;self._last_poll=0.;self.bytes=0
    def start(self):
        if not self.enabled:return
        ros2=shutil.which('ros2')
        if not ros2:self.state='unavailable';self.error='ros2 CLI not installed';return
        try:
            help_result=subprocess.run([ros2,'bag','record','--help'],capture_output=True,text=True,timeout=5)
            if help_result.returncode:raise RuntimeError('rosbag2 record CLI unavailable')
            text=help_result.stdout
            if '--qos-profile-overrides-path' not in text:raise RuntimeError('installed rosbag2 lacks QoS overrides')
            if shutil.disk_usage(self.root).free<self.min_free_bytes:raise RuntimeError('insufficient disk reserve for raw bag')
            qos={topic:{'history':'keep_last','depth':5,'reliability':'best_effort','durability':'volatile'} for topic in self.topics}
            if '/tf_static' in qos:qos['/tf_static'].update(reliability='reliable',durability='transient_local',depth=100)
            if '/parameter_events' in qos:qos['/parameter_events'].update(reliability='reliable',depth=100)
            qpath=self.root/'bag_qos.yaml';qpath.write_text(json.dumps(qos,indent=2)) # JSON is valid YAML
            command=[ros2,'bag','record','-o',str(self.root/'raw_bag'),'--qos-profile-overrides-path',str(qpath)]
            if '--max-bag-size' in text:command+=['--max-bag-size',str(self.segment_bytes)]
            if '--max-cache-size' in text:command+=['--max-cache-size',str(64*1024**2)]
            command+=self.topics;self.console=open(self.root/'bag_console.log','ab')
            self.process=subprocess.Popen(command,stdout=self.console,stderr=subprocess.STDOUT,start_new_session=True);self.state='recording';self.command=command
        except (OSError,RuntimeError,subprocess.TimeoutExpired) as exc:self.state='unavailable';self.error=str(exc)
    def poll(self,now=None):
        now=time.monotonic() if now is None else now
        if self.process and self.process.poll() is not None:
            self.state=self._stop_reason or ('failed' if self.process.returncode else 'finished');self.console.close();self.process=None
        if self.process and now-self._last_poll>=5:
            self._last_poll=now;folder=self.root/'raw_bag';self.bytes=sum(p.stat().st_size for p in folder.rglob('*') if p.is_file()) if folder.exists() else 0
            if self.bytes>=self.total_bytes:self.stop('quota_reached')
            elif shutil.disk_usage(self.root).free<self.min_free_bytes:self.stop('disk_reserve_reached')
        if self.process and self._stop_at is not None and now-self._stop_at>5:
            self.process.terminate()
        return dict(state=self.state,error=self.error,bytes=self.bytes,topics=self.topics,command=getattr(self,'command',None))
    def stop(self,reason='stopped'):
        if self.process and self._stop_at is None:
            self._stop_reason=reason;self._stop_at=time.monotonic();self.state='stopping';self.process.send_signal(signal.SIGINT)
    def close(self):
        self.stop()
        if self.process:
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2)
            self.poll()
        return self.poll()

def code_provenance(paths):
    files=[]
    for path in paths:
        p=pathlib.Path(path)
        try:files.append(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
        except OSError:files.append(dict(path=str(p),missing=True))
    return dict(schema_version=SCHEMA_VERSION,python=sys.version,platform=platform.platform(),ros_distro=os.environ.get('ROS_DISTRO'),rmw=os.environ.get('RMW_IMPLEMENTATION'),files=files)
