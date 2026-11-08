"""Inspect raw cloud geometry and point field layout without ROS."""
import json,sys,sqlite3,types,collections
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'diagnostics/rviz_no_motion_20261107'))
from analyze_recording import CDR

def decode_cloud(data):
    c=CDR(data);sec=c.number('i',4);ns=c.number('I',4);frame=c.string();height=c.number('I',4);width=c.number('I',4)
    fields=[]
    for _ in range(c.number('I',4)):
        name=c.string();offset=c.number('I',4);datatype=c.number('B',1);count=c.number('I',4)
        fields.append(types.SimpleNamespace(name=name,offset=offset,datatype=datatype,count=count))
    big=c.number('B',1);step=c.number('I',4);row_step=c.number('I',4);size=c.number('I',4)
    raw=data[c.offset:c.offset+size];c.offset+=size;dense=c.number('B',1);assert c.offset==len(data)
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec,nanosec=ns),frame_id=frame),width=width,height=height,fields=fields,point_step=step,row_step=row_step,is_bigendian=bool(big),data=raw)

def xyz(msg):
    dtype=np.dtype({'names':['x','y','z'],'formats':['>f4' if msg.is_bigendian else '<f4']*3,'offsets':[next(f.offset for f in msg.fields if f.name==n) for n in ('x','y','z')],'itemsize':msg.point_step})
    values=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    out=np.column_stack([values[n].ravel() for n in ('x','y','z')]);return out[np.isfinite(out).all(axis=1)]

def run():
    stats=[]
    p=ROOT/'record_data/run_19700109_151544_181276302/raw_bag/raw_bag_1.db3'
    with sqlite3.connect('file:'+str(p)+'?mode=ro',uri=True) as db:
        ident=db.execute('select id from topics where name=?',('/rslidar_points',)).fetchone()[0]
        rows=db.execute('select timestamp,data from messages where topic_id=? order by timestamp desc limit 5',(ident,)).fetchall()
    for ts,data in rows:
        msg=decode_cloud(data);points=xyz(msg);radius=np.linalg.norm(points[:,:2],axis=1)
        near=points[(radius>.8)&(radius<4.)];z=near[:,2]
        hist,edges=np.histogram(z,bins=np.arange(-2.5,2.51,.02));peaks=np.argsort(hist)[-8:][::-1]
        stats.append({'stamp':ts*1e-9,'fields':[vars(f) for f in msg.fields],'point_step':msg.point_step,'count':len(points),'near_count':len(near),'near_z_quantiles':np.percentile(z,[0,5,10,20,50,80,95,100]).tolist(),'near_z_peaks':[{'z':float((edges[k]+edges[k+1])/2),'count':int(hist[k])} for k in peaks]})
    (Path(__file__).parent/'cloud_geometry.json').write_text(json.dumps(stats,indent=2)+'\n');print(json.dumps([{k:v for k,v in s.items() if k!='fields'} for s in stats[:2]],indent=2))
if __name__=='__main__':run()
