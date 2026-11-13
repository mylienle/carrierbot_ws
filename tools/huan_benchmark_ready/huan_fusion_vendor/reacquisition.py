"""New anonymous geometry from NEW scan history after the known chain expires.

No person identity is recovered. No old position or velocity enters the fit.
Permission to move still requires a live camera heartbeat, an empty raw image
track list, fresh exact-time laser TF and the original downstream gate.
"""
import math
import numpy as np
from .base_handoff import fresh_scan
from .vision_contract import number,vector

def history_velocity(history):
    times=np.asarray([x['stamp'] for x in history],dtype=float);times-=times.mean()
    points=np.asarray([x['position'] for x in history],dtype=float)
    denom=float(times@times)
    return ((times[:,None]*points).sum(axis=0)/denom).tolist() if denom>0 else None

class GeometryReacquisition:
    def __init__(self):self.track=None;self.sequence=0;self.reason='NOT_INITIALIZED'
    def clear(self,reason):self.track=None;self.reason=reason
    def suspend(self,reason,now):
        # Missing input denies this tick's packet. It is not itself evidence
        # of a different object. Keep only a still-contiguous measurement
        # history; the existing 0.35s scan-gap rule remains the expiry bound.
        if self.track is not None and (now<self.track['last_laser'] or now-self.track['last_laser']>.35):
            self.clear(reason+'_HISTORY_EXPIRED')
        else:self.reason=reason
        return None
    def update(self,scan,now,wall):
        if not fresh_scan(scan,now,wall):return self.suspend('NO_FRESH_SCAN',now)
        stamp=scan['stamp']
        if not (vector(scan.get('sensor_pose'),3) and number(scan.get('query_stamp'))
            and number(scan.get('transform_stamp')) and abs(scan['query_stamp']-stamp)<=.001
            and abs(scan['transform_stamp']-stamp)<=.001):
            self.clear('NO_EXACT_SCAN_TF');return None
        candidates=scan.get('clusters')
        if isinstance(candidates, list) and len(candidates) > 1:
            # Deny this scan. Reuse history only if the next unique fit
            # arrives within the existing time and spatial continuity limits.
            return self.suspend('NO_UNIQUE_GEOMETRY', now)
        if not isinstance(candidates,list) or len(candidates)!=1:
            self.clear('NO_UNIQUE_GEOMETRY');return None
        fit=candidates[0]
        if not (vector(fit.get('xy'),2) and number(fit.get('radius')) and .20<=fit['radius']<=.40
            and isinstance(fit.get('points'),int) and fit['points']>=4 and number(fit.get('residual')) and 0<=fit['residual']<=.025):
            self.clear('INVALID_FIT');return None
        current=self.track
        if current is not None:
            delta=stamp-current['last_laser']
            if delta<0:self.clear('SCAN_CLOCK_RESET');return None
            if delta>.35 or math.dist(fit['xy'],current['position'])>1.2*max(delta,0)+.08:
                self.clear('OBSERVATION_CHAIN_BROKEN');current=None
        if current is None:
            self.sequence+=1
            current=dict(id=1000000000+self.sequence,birth_stamp=stamp,history=[],position=fit['xy'],
                         last_laser=None,confirmed=False,generation=self.sequence)
            self.track=current
        if current['last_laser'] is None or stamp>current['last_laser']:
            current['history'].append(dict(stamp=stamp,position=list(fit['xy'])))
            current['history']=[r for r in current['history'] if stamp-r['stamp']<=1.]
            current.update(last_laser=stamp,position=list(fit['xy']))
        history=current['history']
        if len(history)<4 or history[-1]['stamp']-history[0]['stamp']<.4:
            current['confirmed']=False;self.reason='FRESH_HISTORY_WARMUP';return None
        velocity=history_velocity(history)
        if velocity is None or not vector(velocity,2) or math.hypot(*velocity)>1.2:
            self.clear('UNSUPPORTED_SPEED');return None
        current['confirmed']=True;self.reason='NEW_GEOMETRY_CONFIRMED'
        support=dict(stamp=stamp,position=list(fit['xy']),measured_velocity=velocity,
            velocity=[0.,0.] if math.hypot(*velocity)<=.15 else velocity,
            radius=fit['radius'],residual=fit['residual'],points=fit['points'])
        return dict(id=current['id'],source='laser_scan_tf_reacquired',class_name='generic_obstacle',
            identity_valid=False,semantic_status='generic_reacquired_from_laser',
            geometry_generation=current['generation'],geometry_birth_stamp=current['birth_stamp'],
            history_scan_stamps=[r['stamp'] for r in history],lidar_support=support,
            position_reference='laser_fitted_circle_center_xy')

def camera_permission(packet,receipt,now,wall):
    return (isinstance(packet,dict) and packet.get('source')=='gazebo' and number(receipt)
        and 0<=wall-receipt<=.5 and number(packet.get('timestamp')) and 0<=now-packet['timestamp']<=.5
        and isinstance(packet.get('tracks'),list) and not packet['tracks'])
