#!/usr/bin/env python3
"""RGB-D for RTAB-Map from a ZED 2 / 2i through the ZED SDK (pyzed).

Drop-in replacement for zed_usb2_rgbd: depth is computed by the SDK on the GPU (ULTRA by default, NEURAL needs TensorRT)
instead of StereoSGBM on the CPU.  Publishes, with the same stamp and frame:
  rgb/image_rect   (bgr8, rectified left eye)
  depth/image      (32FC1, meters, 0 = invalid)
  rgb/camera_info  (rectified left intrinsics)
"""

import numpy as np
import pyzed.sl as sl

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image


class ZedSdkRgbd(Node):
    def __init__(self):
        super().__init__('zed_sdk_rgbd')

        resolution = self.declare_parameter('resolution', 'VGA').value
        fps = self.declare_parameter('fps', 15).value
        depth_mode = self.declare_parameter('depth_mode', 'ULTRA').value
        self.frame_id = self.declare_parameter('frame_id', 'zed_left_optical_frame').value
        min_depth = self.declare_parameter('min_depth', 0.3).value
        max_depth = self.declare_parameter('max_depth', 3.0).value

        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, resolution)
        init.camera_fps = fps
        init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        init.depth_minimum_distance = min_depth
        init.depth_maximum_distance = max_depth

        self.zed = sl.Camera()
        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Cannot open ZED camera: {err}')

        config = self.zed.get_camera_information().camera_configuration
        self.width = config.resolution.width
        self.height = config.resolution.height
        cam = config.calibration_parameters.left_cam
        self.get_logger().info(
            f'ZED SDK {self.zed.get_sdk_version()}, {self.width}x{self.height} @ {config.fps} fps, '
            f'depth {depth_mode}, fx={cam.fx:.2f} px')

        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info.width = self.width
        self.camera_info.height = self.height
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [0.0] * 5
        self.camera_info.k = [cam.fx, 0.0, cam.cx, 0.0, cam.fy, cam.cy, 0.0, 0.0, 1.0]
        self.camera_info.r = np.eye(3).flatten().tolist()
        self.camera_info.p = [cam.fx, 0.0, cam.cx, 0.0, 0.0, cam.fy, cam.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.runtime = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.depth = sl.Mat()

        self.rgb_pub = self.create_publisher(Image, 'rgb/image_rect', 5)
        self.depth_pub = self.create_publisher(Image, 'depth/image', 5)
        self.info_pub = self.create_publisher(CameraInfo, 'rgb/camera_info', 5)

    def make_image(self, data, encoding, step, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = encoding
        msg.step = step
        # frombytes() skips the per-byte range check that `msg.data = bytes` does (~1 s/frame).
        msg.data.frombytes(data.tobytes())
        return msg

    def grab_and_publish(self):
        # grab() blocks until the next frame, so it also paces the loop.
        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
        stamp = Time(nanoseconds=self.zed.get_timestamp(
            sl.TIME_REFERENCE.IMAGE).get_nanoseconds()).to_msg()

        bgr = np.ascontiguousarray(self.image.get_data()[:, :, :3])
        depth = np.array(self.depth.get_data(), dtype=np.float32)
        # The SDK marks invalid / too close / too far pixels with NaN or +-inf.
        depth[~np.isfinite(depth)] = 0.0

        self.camera_info.header.stamp = stamp
        self.rgb_pub.publish(self.make_image(bgr, 'bgr8', self.width * 3, stamp))
        self.depth_pub.publish(self.make_image(depth, '32FC1', self.width * 4, stamp))
        self.info_pub.publish(self.camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = ZedSdkRgbd()
    try:
        while rclpy.ok():
            node.grab_and_publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.zed.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
