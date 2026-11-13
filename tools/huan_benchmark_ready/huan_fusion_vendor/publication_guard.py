"""Recheck unchanged source age limits immediately before use/publication."""
from .vision_contract import number

def current_packet(packet,camera_receipt,now,wall):
    if not (isinstance(packet,dict) and number(now) and number(wall) and number(camera_receipt)
        and 0<=wall-camera_receipt<=.5 and number(packet.get('timestamp')) and 0<=now-packet['timestamp']<=.5
        and packet.get('frame_id')=='odom' and isinstance(packet.get('tracks'),list) and packet['tracks']):return None
    for track in packet['tracks']:
        if track.get('source') not in ('vision_rgbd_tf','laser_scan_tf','laser_scan_tf_reacquired','laser_scan_tf_bearing'):return None
        support=track.get('lidar_support')
        if support is not None:
            if not (isinstance(support,dict) and number(support.get('stamp')) and 0<=now-support['stamp']<=.30):return None
        elif track['source']!='vision_rgbd_tf':return None
    return packet
