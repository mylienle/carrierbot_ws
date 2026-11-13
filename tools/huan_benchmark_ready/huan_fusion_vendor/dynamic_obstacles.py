"""Existing RGBD contract plus explicit, fresh LiDAR-supported obstacle source."""
import math
from .bearing_support import camera_bearing_proof
from .vision_contract import assess as vision_assess,number,vector


def assess(packet,robot_plan,now,horizon=2.,robot_radius=.42,person_radius=.30,margin=.20,uncertainty_rate=.15):
    if not isinstance(packet,dict) or packet.get('schema') not in ('camera_laser_handoff_v1','camera_laser_handoff_v2','camera_laser_handoff_v3'):
        return vision_assess(packet,robot_plan,now,horizon,robot_radius,person_radius,margin,uncertainty_rate)
    # Reuse only plan validation; camera and laser tracks keep distinct timestamps.
    out=vision_assess(dict(timestamp=packet.get('timestamp'),tracks=[]),robot_plan,now,horizon,robot_radius,person_radius,margin,uncertainty_rate)
    out.update(valid=False,risk='UNKNOWN',reason='INVALID_HANDOFF',obstacles=[])
    if not out['robot_plan_valid'] or not number(packet.get('timestamp')) or not 0<=now-packet['timestamp']<=.5:return out
    tracks=packet.get('tracks')
    if not isinstance(tracks,list) or not tracks:return out
    seen=set();invalid=False
    for t in tracks:
        if not isinstance(t,dict) or not isinstance(t.get('id'),int) or t['id'] in seen:invalid=True;continue
        seen.add(t['id']);support=t.get('lidar_support');supported=False
        if isinstance(support,dict):
            supported=(number(support.get('stamp')) and 0<=now-support['stamp']<=.30
                and vector(support.get('position'),2) and vector(support.get('velocity'),2)
                and math.hypot(*support['velocity'])<=1.2 and isinstance(support.get('points'),int) and support['points']>=4
                and number(support.get('residual')) and 0<=support['residual']<=.025
                and number(support.get('radius')) and .20<=support['radius']<=.40
                and number(t.get('vision_last_stamp')) and 0<=now-t['vision_last_stamp'])
        if t.get('source')=='vision_rgbd_tf':
            raw=t.get('raw');checked=vision_assess(dict(timestamp=packet['timestamp'],tracks=[raw]),robot_plan,now,horizon,robot_radius,person_radius,margin,uncertainty_rate)
            if not checked['valid']:invalid=True;continue
            obs=checked['obstacles'][0];obs.update(source='vision_rgbd_tf',lidar_confirmed=supported)
            out['obstacles'].append(obs);continue
        bearing=t.get('source')=='laser_scan_tf_bearing'
        generic=t.get('source') in ('laser_scan_tf_reacquired','laser_scan_tf_bearing')
        if generic:
            history=t.get('history_scan_stamps');birth=t.get('geometry_birth_stamp')
            proof=(packet.get('schema') in ('camera_laser_handoff_v2','camera_laser_handoff_v3') and t.get('class_name')=='generic_obstacle'
                and t.get('identity_valid') is False and t.get('semantic_status')==('generic_laser_geometry_with_2d_camera_bearing' if bearing else 'generic_reacquired_from_laser')
                and number(birth) and isinstance(history,list) and len(history)>=4
                and all(number(x) and birth<=x<=now for x in history)
                and all(b>a for a,b in zip(history,history[1:])) and .4<=history[-1]-history[0]<=1.000001
                and isinstance(support,dict) and history[-1]==support.get('stamp'))
            if bearing:
                bh=t.get('bearing_history')
                projected=camera_bearing_proof(packet.get('raw_camera_evidence'),bh,support.get('radius'),now) if isinstance(support,dict) else None
                proof=(proof and packet.get('schema')=='camera_laser_handoff_v3' and isinstance(bh,list)
                    and isinstance(packet.get('raw_camera_evidence'),dict) and packet['raw_camera_evidence'].get('timestamp')==packet['timestamp']
                    and [r.get('stamp') for r in bh]==history and bh[-1].get('position')==support.get('position') and projected is not None
                    and projected==t.get('camera_bearing_support'))
            if not proof:invalid=True;continue
        if t.get('source') not in ('laser_scan_tf','laser_scan_tf_reacquired','laser_scan_tf_bearing') or not supported:invalid=True;continue
        p=support['position'];v=support['velocity'];age=now-support['stamp']
        # Additional .12m association/shape allowance, beyond the baseline model.
        uncertainty=margin+uncertainty_rate*(age+horizon)+.12
        physical_radius=max(person_radius,support['radius']) if generic else person_radius
        envelope=robot_radius+physical_radius+uncertainty;best=None
        points=robot_plan['positions'];rs=robot_plan['stamp']
        for a,b in zip(points,points[1:]):
            ta=rs+a['dt_s']-now;tb=rs+b['dt_s']-now;lo=max(0.,ta);hi=min(horizon,tb)
            if hi<=lo:continue
            rv=[(b['xy_m'][i]-a['xy_m'][i])/(tb-ta) for i in range(2)]
            r=[a['xy_m'][i]+rv[i]*(lo-ta) for i in range(2)]
            rel=[p[i]+v[i]*(age+lo)-r[i] for i in range(2)];vel=[v[i]-rv[i] for i in range(2)];speed2=sum(x*x for x in vel)
            tau=max(0.,min(hi-lo,-sum(x*y for x,y in zip(rel,vel))/speed2)) if speed2>1e-12 else 0.
            d=math.hypot(*(rel[i]+vel[i]*tau for i in range(2)))
            if best is None or d<best[0]:best=(d,lo+tau)
        out['obstacles'].append(dict(id=t['id'],status='active',source=t['source'],lidar_confirmed=True,
            position_m=p,velocity_mps=v,stamp=support['stamp'],age_s=age,confidence=None,radius_m=physical_radius,
            uncertainty_radius_m=uncertainty,reason='FRESH_LASER_WITH_BOUNDED_CAMERA_IDENTITY',
            predicted_positions=[dict(dt_s=h,xy_m=[p[i]+v[i]*h for i in range(2)]) for h in (.5,1.,2.)],
            closest_approach_m=best[0],time_to_closest_s=best[1],required_separation_m=envelope,
            clearance_bound_m=best[0]-envelope,risk='POTENTIAL_COLLISION' if best[0]<=envelope else 'CLEAR_WITHIN_MODEL'))
    out['valid']=not invalid and len(out['obstacles'])==len(tracks)
    out['reason']='OK' if out['valid'] else 'INVALID_HANDOFF'
    out['risk']=('POTENTIAL_COLLISION' if any(o['risk']=='POTENTIAL_COLLISION' for o in out['obstacles'])
                 else 'CLEAR_WITHIN_MODEL' if out['valid'] else 'UNKNOWN')
    return out
