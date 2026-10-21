#!/usr/bin/env python3
"""RGB-D for RTAB-Map from a ZED 2 running on USB 2.0 (no ZED SDK).

Subscribes to the side-by-side stereo frame published by v4l2_camera,
rectifies both eyes with the factory calibration file, computes depth with
OpenCV StereoSGBM and publishes, with the same stamp and frame:
  rgb/image_rect   (bgr8, rectified left eye)
  depth/image      (32FC1, meters, 0 = invalid)
  rgb/camera_info  (rectified left intrinsics)
"""

import configparser

# cv_bridge on the Xavier must be imported after cv2.
import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def load_zed_calibration(path, resolution, image_size):
    conf = configparser.ConfigParser()
    if not conf.read(path):
        raise RuntimeError(f'Cannot read calibration file {path}')

    def intrinsics(side):
        sec = conf[f'{side}_CAM_{resolution}']
        k = np.array([[float(sec['fx']), 0.0, float(sec['cx'])],
                      [0.0, float(sec['fy']), float(sec['cy'])],
                      [0.0, 0.0, 1.0]])

        dist = conf[f'{side}_DISTO'] if conf.has_section(f'{side}_DISTO') else sec

        d = np.array([
            float(dist.get(name, '0'))
            for name in ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')
        ], dtype=np.float64)

        return k, d

    k_left, d_left = intrinsics('LEFT')
    k_right, d_right = intrinsics('RIGHT')

    stereo = conf['STEREO']
    rotation, _ = cv2.Rodrigues(np.array([
        float(stereo.get(f'RX_{resolution}', '0')),
        float(stereo.get(f'CV_{resolution}', '0')),
        float(stereo.get(f'RZ_{resolution}', '0')),
    ]))
    translation = np.array([
        float(stereo['Baseline']),
        float(stereo.get('TY', '0')),
        float(stereo.get('TZ', '0')),
    ])

    r_left, r_right, p_left, p_right, _, _, _ = cv2.stereoRectify(
        k_left, d_left, k_right, d_right, image_size, rotation, translation,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

    maps_left = cv2.initUndistortRectifyMap(
        k_left, d_left, r_left, p_left, image_size, cv2.CV_16SC2)
    maps_right = cv2.initUndistortRectifyMap(
        k_right, d_right, r_right, p_right, image_size, cv2.CV_16SC2)

    # Baseline in the calibration file is in millimeters.
    baseline_m = abs(p_right[0, 3] / p_right[0, 0]) / 1000.0
    return maps_left, maps_right, p_left, baseline_m


class ZedUsb2Rgbd(Node):
    def __init__(self):
        super().__init__('zed_usb2_rgbd')

        calibration_file = self.declare_parameter(
            'calibration_file', '/usr/local/zed/settings/SN29068780.conf').value
        resolution = self.declare_parameter('resolution', 'VGA').value
        self.width = self.declare_parameter('eye_width', 672).value
        self.height = self.declare_parameter('eye_height', 376).value
        self.frame_id = self.declare_parameter('frame_id', 'zed_left_optical_frame').value
        self.min_depth = self.declare_parameter('min_depth', 0.3).value
        self.max_depth = self.declare_parameter('max_depth', 4.0).value
        num_disparities = self.declare_parameter('num_disparities', 64).value
        block_size = self.declare_parameter('block_size', 5).value

        size = (self.width, self.height)
        self.maps_left, self.maps_right, self.p_left, baseline_m = load_zed_calibration(
            calibration_file, resolution, size)
        self.focal_baseline = self.p_left[0, 0] * baseline_m
        self.get_logger().info(
            f'Calibration {calibration_file}: rectified fx={self.p_left[0, 0]:.2f} px, '
            f'baseline={baseline_m * 1000.0:.2f} mm')

        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * block_size * block_size,
            P2=32 * block_size * block_size,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

        self.bridge = CvBridge()
        self.camera_info = self.make_camera_info()

        self.rgb_pub = self.create_publisher(Image, 'rgb/image_rect', 5)
        self.depth_pub = self.create_publisher(Image, 'depth/image', 5)
        self.info_pub = self.create_publisher(CameraInfo, 'rgb/camera_info', 5)
        # Depth 1: drop stale frames instead of queuing when SGBM is slower than the camera.
        self.create_subscription(Image, 'stereo/image_raw', self.image_callback, 1)

    def make_camera_info(self):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = self.p_left[:, :3].flatten().tolist()
        info.r = np.eye(3).flatten().tolist()
        info.p = self.p_left.flatten().tolist()
        return info

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame.shape[1] != 2 * self.width or frame.shape[0] != self.height:
            self.get_logger().error(
                f'Expected {2 * self.width}x{self.height} side-by-side frame, '
                f'got {frame.shape[1]}x{frame.shape[0]}', throttle_duration_sec=5.0)
            return

        left = cv2.remap(frame[:, :self.width], *self.maps_left, cv2.INTER_LINEAR)
        right = cv2.remap(frame[:, self.width:], *self.maps_right, cv2.INTER_LINEAR)

        disparity = self.sgbm.compute(
            cv2.cvtColor(left, cv2.COLOR_BGR2GRAY),
            cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)).astype(np.float32) / 16.0

        depth = np.zeros(disparity.shape, dtype=np.float32)
        valid = disparity > 0.0
        depth[valid] = self.focal_baseline / disparity[valid]
        depth[(depth < self.min_depth) | (depth > self.max_depth)] = 0.0

        rgb_msg = self.bridge.cv2_to_imgmsg(left, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        for out in (rgb_msg, depth_msg, self.camera_info):
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.frame_id

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(self.camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = ZedUsb2Rgbd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
