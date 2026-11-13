"""Dynamic obstacle contract and continuous-time collision screening, no actuation.

Robot plan: stamp (ROS seconds), frame_id=odom, positions[{dt_s, xy_m}].
Positions must cover [now, now+horizon]. Linear interpolation within segments.
Vision input is the existing /vision/tracks packet; no target selection.
"""
import math


def number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def vector(x, n):
    return isinstance(x, (list, tuple)) and len(x) == n and all(number(v) for v in x)


def assess(packet, robot_plan, now, horizon=2.0, robot_radius=.42,
           person_radius=.30, margin=.20, uncertainty_rate=.15):
    """Conservative constant-velocity model; parameters are provisional, in SI.

    Uncertainty grows from the observation through the whole horizon. Comparing
    exact segment CPA against this maximum envelope is conservative. An invalid
    track/plan is unknown, never a declaration of clear space.
    """
    out=dict(schema_version=1, generated_stamp=now if number(now) else None,
             frame_id='odom', valid=False, risk='UNKNOWN', reason='INVALID_INPUT',
             horizon_s=horizon, obstacles=[], robot_plan_valid=False,
             model='constant_velocity_continuous_CPA', actuation=False)
    if not (number(now) and number(horizon) and horizon>0 and
            all(number(v) and v>=0 for v in
                (robot_radius,person_radius,margin,uncertainty_rate))):
        return out
    if not isinstance(packet,dict) or not number(packet.get('timestamp')):
        return out
    stamp=packet['timestamp'];age=now-stamp
    if not 0<=age<=.5:
        out['reason']='STALE_OR_FUTURE_IMAGE';return out
    tracks=packet.get('tracks')
    if not isinstance(tracks,list):return out
    segments=[]
    if isinstance(robot_plan,dict) and robot_plan.get('frame_id')=='odom' and number(robot_plan.get('stamp')):
        ps=robot_plan.get('positions');rs=robot_plan['stamp']
        if isinstance(ps,list) and len(ps)>=2 and 0<=now-rs<=.5:
            points=[]
            for p in ps:
                if not isinstance(p,dict) or not number(p.get('dt_s')) or p['dt_s']<0 or not vector(p.get('xy_m'),2):break
                points.append((rs+p['dt_s']-now, p['xy_m']))
            if (len(points)==len(ps) and points[0][0]<=0 and points[-1][0]>=horizon
                and all(b[0]>a[0] for a,b in zip(points,points[1:]))):
                for (ta,pa),(tb,pb) in zip(points,points[1:]):
                    lo,hi=max(0.,ta),min(horizon,tb)
                    if hi<=lo:continue
                    v=[(pb[i]-pa[i])/(tb-ta) for i in range(2)]
                    r=[pa[i]+v[i]*(lo-ta) for i in range(2)]
                    segments.append((lo,hi,r,v))
    out['robot_plan_valid']=bool(segments)
    seen=set();invalid=False
    for t in tracks:
        ident=t.get('id') if isinstance(t,dict) else None
        d=dict(id=ident if isinstance(ident,int) and not isinstance(ident,bool) else None,
               status='invalid', risk='UNKNOWN', position_m=None, velocity_mps=None,
               predicted_positions=[], reason='INVALID_TRACK')
        out['obstacles'].append(d)
        good=(isinstance(t,dict) and isinstance(ident,int) and not isinstance(ident,bool)
              and ident>=0 and ident not in seen and number(t.get('timestamp'))
              and abs(t['timestamp']-stamp)<=.001 and t.get('reference_valid') is True
              and t.get('odom_frame_id')=='odom' and t.get('odom_source')=='tf2_ekf'
              and all(number(t.get(k)) and abs(t[k]-stamp)<=.001 for k in
                      ('reference_query_stamp','reference_transform_stamp'))
              and number(t.get('confidence')) and .4<=t['confidence']<=1
              and t.get('pose_status') in ('OK','OK_PROFILE')
              and number(t.get('depth_m')) and .3<=t['depth_m']<=8
              and vector(t.get('xyz_odom_m'),3) and t.get('velocity_odom_valid') is True
              and vector(t.get('velocity_odom_mps'),3))
        if not good:invalid=True;continue
        seen.add(ident);p=t['xyz_odom_m'][:2];v=t['velocity_odom_mps'][:2]
        envelope=robot_radius+person_radius+margin+uncertainty_rate*(age+horizon)
        d.update(status='active',position_m=p,velocity_mps=v,stamp=stamp,age_s=age,
                 confidence=t['confidence'],radius_m=person_radius,
                 uncertainty_radius_m=margin+uncertainty_rate*(age+horizon),
                 predicted_positions=[dict(dt_s=h,xy_m=[p[i]+v[i]*h for i in range(2)])
                                      for h in (.5,1.,2.)],reason='NO_VALID_ROBOT_PLAN')
        if not segments:continue
        best=None
        for lo,hi,r,rv in segments:
            rel=[p[i]+v[i]*(age+lo)-r[i] for i in range(2)]
            vel=[v[i]-rv[i] for i in range(2)];speed2=sum(a*a for a in vel)
            tau=max(0.,min(hi-lo,-sum(a*b for a,b in zip(rel,vel))/speed2)) if speed2>1e-12 else 0.
            distance=math.hypot(*(rel[i]+vel[i]*tau for i in range(2)))
            if best is None or distance<best[0]:best=(distance,lo+tau)
        distance,ttc=best
        d.update(risk='POTENTIAL_COLLISION' if distance<=envelope else 'CLEAR_WITHIN_MODEL',
                 reason='OK',closest_approach_m=distance,time_to_closest_s=ttc,
                 required_separation_m=envelope,clearance_bound_m=distance-envelope)
    out['valid']=bool(segments) and not invalid
    out['reason']='INVALID_TRACKS' if invalid else ('OK' if segments else 'NO_VALID_ROBOT_PLAN')
    risks=[d['risk'] for d in out['obstacles']]
    out['risk']=('POTENTIAL_COLLISION' if 'POTENTIAL_COLLISION' in risks else
                 'CLEAR_WITHIN_MODEL' if out['valid'] and risks else 'UNKNOWN')
    if not tracks:out.update(valid=False,risk='UNKNOWN',reason='NO_OBSERVATIONS')
    return out
