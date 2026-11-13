"""Original handoff plus a separate, anonymous, measurement-only reacquisition."""
import os
from .base_handoff import Handoff as OriginalHandoff
from .base_handoff import clusters,fresh_scan,still_plan,route_changed,reusable_identity
from .vision_contract import vector,number
from .bearing_support import camera_bearing_proof
from .reacquisition import GeometryReacquisition,camera_permission

class Handoff:
    def __init__(self):
        self.base=OriginalHandoff();self.geometry=GeometryReacquisition();self.enabled=os.getenv('REACQUIRE_ENABLE','1')=='1'
        self.last_valid_camera_stamp=None;self.last_now=None;self.last_diagnostic={}
        self.bearing_geometry=GeometryReacquisition();self.bearing_enabled=os.getenv('BEARING_FUSION_ENABLE','1')=='1'
    @property
    def cache(self):
        if self.geometry.track is None:return self.base.cache
        c=self.geometry.track
        merged=dict(self.base.cache)
        merged[c['id']]=dict(confirmed=c['confirmed'],history=c['history'],last_laser=c['last_laser'],
            vision_stamp=self.last_valid_camera_stamp,generic=True)
        return merged
    def build(self,packet,receipt,scan,robot,now,wall):
        if self.last_now is not None and now<self.last_now:
            self.base=OriginalHandoff();self.geometry=GeometryReacquisition();self.last_valid_camera_stamp=None
        if self.last_now is not None and now<self.last_now:self.bearing_geometry=GeometryReacquisition()
        self.last_now=now
        normal=self.base.build(packet,receipt,scan,robot,now,wall)
        if normal is not None:
            for t in normal['tracks']:
                if t['source']=='vision_rgbd_tf':self.last_valid_camera_stamp=t['vision_last_stamp']
            if self.last_valid_camera_stamp is not None:self.bearing_geometry.update(scan,now,wall)
            self.geometry.clear('ORIGINAL_HANDOFF_VALID');self.last_diagnostic=dict(mode='original');return normal
        if not self.enabled:
            self.geometry.clear('BASELINE_DISABLED');self.last_diagnostic=dict(mode='baseline');return None
        # An incomplete new image ID is never promoted to a camera/depth track.
        # A separately observed circle can instead remain anonymous if the
        # calibrated image ray actually hits that same, live physical chain.
        bt=self.bearing_geometry.update(scan,now,wall) if self.last_valid_camera_stamp is not None else None
        if self.bearing_enabled and self.base.cache and len(self.base.cache)==1 and bt is not None and vector(robot,3):
            known=next(iter(self.base.cache.values()));support=known.get('support')
            same=(known.get('established') and known.get('confirmed') and isinstance(support,dict)
                and support['stamp']==bt['lidar_support']['stamp'] and support['position']==bt['lidar_support']['position'])
            live=(isinstance(packet,dict) and packet.get('source')=='gazebo' and number(receipt) and 0<=wall-receipt<=.5)
            proof=camera_bearing_proof(packet,self.bearing_geometry.track['history'],bt['lidar_support']['radius'],now) if same and live else None
            if proof is not None:
                bt.update(id=bt['id']+2000000000,source='laser_scan_tf_bearing',semantic_status='generic_laser_geometry_with_2d_camera_bearing',
                    vision_last_stamp=self.last_valid_camera_stamp,camera_bearing_support=proof,
                    bearing_history=[dict(stamp=r['stamp'],position=list(r['position'])) for r in self.bearing_geometry.track['history']])
                self.last_diagnostic=dict(mode='camera_bearing',reason='ANONYMOUS_MEASURED_GEOMETRY_WITH_MATCHED_2D_RAY',
                    camera_2d_id=proof['track_2d_id'],identity_recovered=False)
                return dict(schema='camera_laser_handoff_v3',timestamp=packet['timestamp'],frame_id='odom',tracks=[bt],
                    scan_stamp=scan['stamp'],sensor_pose=scan['sensor_pose'],raw_camera_evidence=packet,
                    scope='Anonymous LiDAR geometry; bbox horizontal ray matched at image time; no recovered depth or person identity')
        if self.base.cache:
            self.geometry.clear('ORIGINAL_TRACK_STILL_EXISTS');self.last_diagnostic=dict(mode='original_invalid');return None
        if self.last_valid_camera_stamp is None:
            self.geometry.clear('NO_PREVIOUS_VALID_VISION');self.last_diagnostic=dict(mode='not_armed');return None
        # The scan already has its own exact-time odom transform. A missing
        # CURRENT robot pose must stop dispatch, without discarding that valid
        # scan history. No pose is fabricated or substituted.
        t=self.geometry.update(scan,now,wall)
        permitted=camera_permission(packet,receipt,now,wall)
        self.last_diagnostic=dict(mode='reacquisition',reason=self.geometry.reason,camera_permission=permitted,
            last_valid_camera_stamp=self.last_valid_camera_stamp,
            fresh_history_samples=len(self.geometry.track['history']) if self.geometry.track else 0)
        if not vector(robot,3):
            self.last_diagnostic['mode']='no_robot_tf';return None
        if t is None or not permitted:return None
        t['vision_last_stamp']=self.last_valid_camera_stamp
        return dict(schema='camera_laser_handoff_v2',timestamp=packet['timestamp'],frame_id='odom',tracks=[t],
            scan_stamp=scan['stamp'],sensor_pose=scan['sensor_pose'],
            scope='Anonymous geometry reinitialized from new scan history; no recovered human identity')
