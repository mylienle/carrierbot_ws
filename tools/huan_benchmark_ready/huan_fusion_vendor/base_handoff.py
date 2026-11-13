"""Bounded semantic camera -> continuously observed laser handoff, odom frame.

Never invent a new camera timestamp/depth/TF. A laser track is a separate source.
Camera process heartbeat, fresh scan + TF, circle fit, association and bounded
semantic age are all required. No scan support means stop, not blind coasting.
"""
import math
import numpy as np
from .vision_contract import assess as vision_assess, number, vector

def fresh_scan(scan,now,wall):
    return (isinstance(scan,dict) and number(scan.get('stamp')) and 0<=now-scan['stamp']<=.30
            and number(scan.get('wall')) and 0<=wall-scan['wall']<=.35)


def clusters(ranges, angle_min, angle_increment, range_min, range_max, pose):
    groups=[];group=[];previous=None
    for i,r in enumerate(ranges):
        if not number(r) or not range_min<r<min(range_max,10.):
            if group:groups.append(group)
            group=[];previous=None;continue
        a=angle_min+i*angle_increment+pose[2]
        p=[pose[0]+r*math.cos(a),pose[1]+r*math.sin(a)]
        if previous is not None and math.dist(previous,p)>.16:
            if group:groups.append(group)
            group=[]
        group.append(p);previous=p
    if group:groups.append(group)
    # The robot laser faces backwards: a person ahead straddles +/-pi.
    # Merge the two boundary arcs before fitting, rather than inventing two people.
    if (len(groups)>1 and abs(angle_increment)*len(ranges)>6.
        and math.dist(groups[-1][-1],groups[0][0])<.16):
        groups=[groups[-1]+groups[0]]+groups[1:-1]
    result=[]
    for group in groups:
        if len(group)<4 or math.dist(group[0],group[-1])<.12:continue
        a=np.asarray(group);xy=a-a.mean(axis=0)
        # Reject straight wall segments and poorly conditioned arcs.
        if np.linalg.svd(xy,compute_uv=False)[-1]<.012:continue
        matrix=np.column_stack((2*xy[:,0],2*xy[:,1],np.ones(len(xy))))
        fit=np.linalg.lstsq(matrix,(xy*xy).sum(axis=1),rcond=None)[0]
        center=fit[:2]+a.mean(axis=0)
        radius=float(np.sqrt(max(0.,fit[2]+fit[0]**2+fit[1]**2)))
        residual=float(np.std(np.linalg.norm(a-center,axis=1)))
        if .20<=radius<=.40 and residual<=.025:
            result.append(dict(xy=center.tolist(),radius=radius,points=len(group),residual=residual))
    return result


def still_plan(robot,now):
    return dict(stamp=now,frame_id='odom',positions=[dict(dt_s=0.,xy_m=robot[:2]),dict(dt_s=3.,xy_m=robot[:2])])


def reusable_identity(cache,track,now):
    """Only transfer a unique, live laser chain to a fully valid camera ID.

    Called after the original camera contract, including velocity, has passed.
    No invalid camera track, spatial extrapolation or stale support is accepted.
    """
    matches=[]
    for ident,c in cache.items():
        support=c.get('support')
        if not c.get('confirmed') or not c.get('established') or not support:continue
        if not 0<=now-support['stamp']<=.30:continue
        distance=math.dist(c['position'],track['xyz_odom_m'][:2])
        delta=math.dist(support['measured_velocity'],track['velocity_odom_mps'][:2])
        if distance<.55 and delta<=.35:matches.append(ident)
    return matches[0] if len(matches)==1 else None


class Handoff:
    def __init__(self):self.cache={};self.last_now=None

    def build(self,packet,receipt,scan,robot,now,wall):
        if self.last_now is not None and now<self.last_now:self.cache={}
        self.last_now=now
        camera_live=(isinstance(packet,dict) and number(receipt) and 0<=wall-receipt<=.5
            and number(packet.get('timestamp')) and 0<=now-packet['timestamp']<=.5
            and isinstance(packet.get('tracks'),list))
        if not vector(robot,3):return None
        # Continuously measured laser geometry can update existing memory while
        # camera motion permission is absent. Never initialize from stale images,
        # renew their semantic age, or emit a packet without a live camera.
        camera_tracks=packet['tracks'] if camera_live else []
        candidates=[]
        if fresh_scan(scan,now,wall):
            candidates=scan['clusters']
        plan=still_plan(robot,now);stamp=packet['timestamp'] if camera_live else None;valid={};invalid=[]
        ids=[t.get('id') for t in camera_tracks if isinstance(t,dict)]
        if len(ids)!=len(camera_tracks) or any(not isinstance(i,int) or isinstance(i,bool) or i<0 for i in ids) or len(set(ids))!=len(ids):return None
        for t in camera_tracks:
            check=vision_assess(dict(timestamp=stamp,tracks=[t]),plan,now)
            if check['valid']:
                valid[t['id']]=t
                old=self.cache.get(t['id'])
                if old is None:
                    prior=reusable_identity(self.cache,t,now)
                    if prior is not None:
                        old=self.cache.pop(prior)
                        old['identity_handoff']=dict(from_id=prior,to_id=t['id'],camera_stamp=stamp,
                            laser_stamp=old['support']['stamp'],
                            position_delta_m=math.dist(old['position'],t['xyz_odom_m'][:2]),
                            velocity_delta_mps=math.dist(old['support']['measured_velocity'],t['velocity_odom_mps'][:2]))
                    else:
                        for ident,c in list(self.cache.items()):
                            if math.dist(c['position'],t['xyz_odom_m'][:2])<.6:del self.cache[ident]
                        old=dict(history=[],position=t['xyz_odom_m'][:2],last_laser=None,confirmed=False,established=False)
                    self.cache[t['id']]=old
                old.update(vision=t,vision_stamp=stamp,position=t['xyz_odom_m'][:2] if old['last_laser'] is None else old['position'])
            else:invalid.append(t['id'])
        for ident,c in list(self.cache.items()):
            # The semantic label expires after60s. Current laser geometry may
            # remain a generic obstacle only while its measurement chain is live.
            if now-c['vision_stamp']>2. and (c['last_laser'] is None or now-c['last_laser']>2.):
                del self.cache[ident];continue
            matched=[]
            prediction=c['position']
            for candidate in candidates:
                if math.dist(candidate['xy'],prediction)<.55:matched.append(candidate)
            c['support']=None
            if len(matched)!=1:continue
            m=matched[0];ss=scan['stamp'];last=c['last_laser']
            if last is not None and ss<last:self.cache={};return None
            if last is None or ss>last:
                if last is not None and (ss-last>.35 or math.dist(m['xy'],c['position'])>1.2*(ss-last)+.08):
                    c['history']=[];c['confirmed']=False
                c['history'].append((ss,m['xy']));c['history']=[x for x in c['history'] if ss-x[0]<=1.]
                c['last_laser']=ss;c['position']=m['xy']
            history=c['history'];velocity=[0.,0.]
            if len(history)>=4 and history[-1][0]-history[0][0]>=.4:
                times=np.array([x[0] for x in history]);times-=times.mean()
                positions=np.array([x[1] for x in history])
                velocity=((times[:,None]*positions).sum(axis=0)/(times@times)).tolist()
                # Initial handoff only after the camera AND laser establish slow motion.
                initial=(now-c['vision_stamp']<=.5 and math.hypot(*velocity)<=.15
                         and math.hypot(*c['vision']['velocity_odom_mps'][:2])<=.15
                         and math.dist(m['xy'],c['vision']['xyz_odom_m'][:2])<.55)
                if initial or (c['established'] and math.hypot(*velocity)<=1.2):
                    c['confirmed']=True;c['established']=True
            if math.hypot(*velocity)>1.2:c['confirmed']=False
            if c['confirmed']:
                measured=velocity
                # Slow standing sway is inside the existing .15m/s uncertainty
                # envelope, rather than repeatedly extrapolating noisy sway.
                velocity=[0.,0.] if math.hypot(*velocity)<=.15 else velocity
                c['support']=dict(stamp=ss,position=m['xy'],velocity=velocity,measured_velocity=measured,points=m['points'],
                                  residual=m['residual'],radius=m['radius'])
        if not camera_live:return None
        result=[]
        for ident,t in valid.items():
            c=self.cache[ident];result.append(dict(source='vision_rgbd_tf',id=ident,raw=t,
                lidar_support=c.get('support'),vision_last_stamp=stamp))
        for ident,c in self.cache.items():
            if ident in valid:continue
            bearing=math.atan2(c['position'][1]-robot[1],c['position'][0]-robot[0])-robot[2]
            bearing=math.atan2(math.sin(bearing),math.cos(bearing))
            if c.get('support') is None or abs(bearing)<.55:return None
            result.append(dict(source='laser_scan_tf',id=ident,lidar_support=c['support'],vision_last_stamp=c['vision_stamp'],
                               semantic_status='camera_identity' if now-c['vision_stamp']<=60. else 'generic_obstacle_continuously_measured'))
        if any(i not in self.cache for i in invalid):return None
        if not result:return None
        return dict(schema='camera_laser_handoff_v1',timestamp=stamp,tracks=result,
                    scan_stamp=scan['stamp'] if candidates else None,frame_id='odom',
                    sensor_pose=scan.get('sensor_pose') if candidates else None,
                    identity_handoffs=[c['identity_handoff'] for c in self.cache.values() if 'identity_handoff' in c])


def route_changed(previous,current,threshold=.30):
    if not previous or not current:return False
    def distance(p,a,b):
        d=[b[i]-a[i] for i in range(2)];den=sum(x*x for x in d)
        f=max(0.,min(1.,sum((p[i]-a[i])*d[i] for i in range(2))/den)) if den else 0.
        return math.dist(p,[a[i]+f*d[i] for i in range(2)])
    return any(min(distance(p,a,b) for a,b in zip(previous,previous[1:]))>threshold for p in current)
