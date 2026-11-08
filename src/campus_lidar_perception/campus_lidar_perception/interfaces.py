"""Populate the existing planner message without extending its ROS schema."""
import math

PLANNER_FIELDS = ('id','type','x','y','length','width','heading','speed',
                  'has_shape_heading','shape_heading')


def obstacle_array(result, array_factory, obstacle_factory):
    msg = array_factory()
    msg.header.frame_id = 'map'
    total_ns = result.get('stamp_ns',round(result['stamp']*1e9))
    msg.header.stamp.sec,msg.header.stamp.nanosec = divmod(total_ns,1000000000)
    for row in result['tracks']:
        if not all(math.isfinite(float(row[k])) for k in ('x','y','length','width','heading','speed','shape_heading')):
            raise ValueError('invalid track for planner')
        ob = obstacle_factory()
        for field in PLANNER_FIELDS:
            setattr(ob,field,row[field])
        msg.obstacles.append(ob)
    return msg
