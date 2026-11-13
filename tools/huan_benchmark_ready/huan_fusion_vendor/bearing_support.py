"""Anonymous 2-D bearing / LiDAR consistency, never a depth or identity repair.

Planar scope: the camera optical vertical axis must be vertical in odom.
The horizontal bbox-center ray must intersect the measured circle at IMAGE
time, using two real scans bracketing that time. No motion extrapolation.
"""
import math
import numpy as np
from .vision_contract import number,vector

def horizontal_match(track,stamp,frame,history,radius,model):
    if not (isinstance(track,dict) and isinstance(track.get('id'),int) and not isinstance(track['id'],bool) and track['id']>=0 and number(stamp) and isinstance(model,dict)
        and model.get('frame_id')==frame and isinstance(model.get('width'),int) and isinstance(model.get('height'),int)
        and model['width']>0 and model['height']>0
        and all(number(model.get(k)) for k in ['fx','fy','cx','cy']) and model['fx']>0 and model['fy']>0
        and 0<=model['cx']<model['width'] and 0<=model['cy']<model['height']
        and vector(track.get('bbox'),4) and number(track.get('confidence')) and .4<=track['confidence']<=1.
        and 0<=track['bbox'][0]<track['bbox'][2]<=model['width']
        and 0<=track['bbox'][1]<track['bbox'][3]<=model['height']
        and track.get('reference_valid') is True and track.get('odom_frame_id')=='odom'
        and track.get('odom_source')=='tf2_ekf'
        and all(number(track.get(k)) and abs(track[k]-stamp)<=.001 for k in ['timestamp','reference_query_stamp','reference_transform_stamp'])
        and number(radius) and .20<=radius<=.40):return None
    try:matrix=np.asarray(track.get('odom_from_camera'),dtype=float)
    except (TypeError,ValueError):return None
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all():return None
    rotation=matrix[:3,:3]
    if (np.max(np.abs(rotation.T@rotation-np.eye(3)))>1e-5 or abs(np.linalg.det(rotation)-1.)>1e-5
        or not np.allclose(matrix[3],[0.,0.,0.,1.],atol=1e-9,rtol=0)
        or abs(rotation[2,0])>.001 or abs(rotation[2,2])>.001 or abs(abs(rotation[2,1])-1.)>.001):return None
    if not isinstance(history,list) or len(history)<4:return None
    if not all(number(r.get('stamp')) and vector(r.get('position'),2) for r in history):return None
    if not all(0<b['stamp']-a['stamp']<=.35 and math.dist(a['position'],b['position'])<=1.2*(b['stamp']-a['stamp'])+.08 for a,b in zip(history,history[1:])):return None
    bracket=next(((a,b) for a,b in zip(history,history[1:]) if a['stamp']<=stamp<=b['stamp']),None)
    if bracket is None:return None
    a,b=bracket;f=(stamp-a['stamp'])/(b['stamp']-a['stamp'])
    position=np.array(a['position'])+f*(np.array(b['position'])-np.array(a['position']))
    u=(track['bbox'][0]+track['bbox'][2])/2.
    direction=(rotation@np.array([(u-model['cx'])/model['fx'],0.,1.]))[:2]
    norm=float(np.linalg.norm(direction))
    if norm<1e-9:return None
    direction/=norm;relative=position-matrix[:2,3];along=float(relative@direction)
    perpendicular=abs(float(direction[0]*relative[1]-direction[1]*relative[0]))
    if along<=0 or perpendicular>radius:return None
    return dict(image_stamp=stamp,track_2d_id=track.get('id'),bbox_center_u_px=u,
        scan_bracket_stamps=[a['stamp'],b['stamp']],circle_at_image_xy=position.tolist(),
        ray_origin_xy=matrix[:2,3].tolist(),ray_direction_xy=direction.tolist(),
        ray_perpendicular_m=perpendicular,ray_forward_m=along,physical_radius_m=radius,
        method='single_bbox_horizontal_ray_intersects_interpolated_measured_circle')

def camera_bearing_proof(packet,history,radius,now):
    if not (isinstance(packet,dict) and packet.get('source')=='gazebo' and number(packet.get('timestamp'))
        and 0<=now-packet['timestamp']<=.5 and isinstance(packet.get('tracks'),list) and len(packet['tracks'])==1
        and isinstance(packet.get('camera_model'),dict) and packet['camera_model'].get('source')=='camera_info'
        and isinstance(packet.get('vision_quality'),dict)
        and packet['camera_model'].get('width')==packet['vision_quality'].get('image_width')
        and packet['camera_model'].get('height')==packet['vision_quality'].get('image_height')
        and isinstance(packet['camera_model'].get('distortion'),list)
        and all(number(x) and abs(x)<=1e-9 for x in packet['camera_model']['distortion'])):
        return None
    return horizontal_match(packet['tracks'][0],packet['timestamp'],packet.get('frame_id'),history,radius,packet['camera_model'])
