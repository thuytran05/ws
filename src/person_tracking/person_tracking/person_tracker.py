#!/usr/bin/env python3
"""YOLO Pose selection + registered RGB/depth. Does not publish robot commands.

Target PointStamped: optical x right, y down, z forward, meters.
Invalid target: NaN point. A lost selected ID is latched until reset.
"""
import time

import cv2
import message_filters
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32, Int32MultiArray
from std_srvs.srv import Trigger
from ultralytics import YOLO


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker', namespace='person_tracking')
        defaults = dict(image_topic='/camera/color/image_raw',
                        depth_topic='/camera/depth/image_raw',
                        info_topic='/camera/color/camera_info',
                        model='yolo26n-pose.pt', imgsz=640, conf=0.35,
                        gesture_seconds=2.0, gesture_gap_seconds=0.75,
                        keypoint_conf=0.50, device='auto',
                        max_capture_age=0.40, sync_slop=0.06,
                        stream_timeout=0.40)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        device = str(self.p('device'))
        self.device = ('0' if torch.cuda.is_available() else 'cpu') if device == 'auto' else device
        self.get_logger().info(f'Loading {self.p("model")}, device={self.device}')
        self.model = YOLO(self.p('model'))
        self.bridge = CvBridge()
        self.selected_id = None
        self.lost = False
        self.raised = {}
        self.info = None
        self.last_pair = time.monotonic()
        self.create_subscription(CameraInfo, self.p('info_topic'), self.on_info, qos_profile_sensor_data)
        self.annotated_pub = self.create_publisher(Image, 'image_annotated', qos_profile_sensor_data)
        self.id_pub = self.create_publisher(Int32, 'selected_id', 10)
        self.pixel_pub = self.create_publisher(Int32MultiArray, 'target_pixel', 1)
        self.visible_pub = self.create_publisher(Bool, 'target_visible', 1)
        self.target_pub = self.create_publisher(PointStamped, 'target_position', 1)
        self.create_service(Trigger, 'reset', self.reset)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb_sub = message_filters.Subscriber(self, Image, self.p('image_topic'), qos_profile=qos)
        self.depth_sub = message_filters.Subscriber(self, Image, self.p('depth_topic'), qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=self.p('sync_slop'))
        self.sync.registerCallback(self.on_images)
        self.create_timer(0.10, self.watchdog)
        self.get_logger().info('Ready: hold hand above nose; registered depth required; reset after loss.')

    def p(self, name):
        return self.get_parameter(name).value

    def on_info(self, msg):
        self.info = msg

    def invalid(self, header=None):
        msg = PointStamped()
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
        msg.point.x = msg.point.y = msg.point.z = float('nan')
        self.target_pub.publish(msg)
        self.visible_pub.publish(Bool(data=False))

    def latch(self, reason):
        if self.selected_id is not None and not self.lost:
            self.lost = True
            self.get_logger().warn(reason + '; reset required')

    def reset(self, request, response):
        self.selected_id = None
        self.lost = False
        self.raised.clear()
        self.invalid()
        self.id_pub.publish(Int32(data=-1))
        response.success = True
        response.message = 'Selection cleared; raise hand again'
        return response

    def watchdog(self):
        if time.monotonic() - self.last_pair > self.p('stream_timeout'):
            self.latch('RGB/depth stream timeout')
            self.invalid()

    def hand_up(self, xy, conf, i, height):
        threshold = self.p('keypoint_conf')
        if conf is None or conf[i, 0] < threshold:
            return False
        return any(conf[i, wrist] >= threshold and
                   xy[i, wrist, 1] < xy[i, 0, 1] - max(10.0, height * 0.035)
                   for wrist in (9, 10))

    def measure(self, depth, encoding, box, xy, conf, i):
        x1, y1, x2, y2 = box
        # Prefer the torso between shoulders and hips, unaffected by raised hands.
        if conf is not None and all(conf[i, j] >= self.p('keypoint_conf') for j in (5, 6, 11, 12)):
            left = min(xy[i, 5, 0], xy[i, 6, 0], xy[i, 11, 0], xy[i, 12, 0])
            right = max(xy[i, 5, 0], xy[i, 6, 0], xy[i, 11, 0], xy[i, 12, 0])
            top = (xy[i, 5, 1] + xy[i, 6, 1]) / 2
            bottom = (xy[i, 11, 1] + xy[i, 12, 1]) / 2
        else:
            left, right = x1 + 0.30 * (x2-x1), x1 + 0.70 * (x2-x1)
            top, bottom = y1 + 0.25 * (y2-y1), y1 + 0.60 * (y2-y1)
        # Inner patch reduces background near silhouette edges.
        mx, my = (left+right)/2, (top+bottom)/2
        rx, ry = max(2, (right-left)*0.25), max(2, (bottom-top)*0.25)
        height, width = depth.shape
        a, b = max(0, int(mx-rx)), min(width, int(mx+rx)+1)
        c, d = max(0, int(my-ry)), min(height, int(my+ry)+1)
        patch = depth[c:d, a:b].astype(np.float32)
        if encoding == '16UC1':
            patch *= 0.001
        elif encoding != '32FC1':
            raise ValueError('Unsupported depth encoding: ' + encoding)
        valid = patch[np.isfinite(patch) & (patch >= 0.4) & (patch <= 5.0)]
        if valid.size < max(30, patch.size * 0.30):
            return None, (a, c, b, d)
        z = float(np.median(valid))
        if float(np.median(np.abs(valid-z))) > max(0.10, z*0.08):
            return None, (a, c, b, d)
        return z, (a, c, b, d)

    def on_images(self, rgb_msg, depth_msg):
        self.last_pair = time.monotonic()
        try:
            info = self.info
            if info is None:
                self.invalid(rgb_msg.header)
                return
            if (rgb_msg.width, rgb_msg.height) != (depth_msg.width, depth_msg.height) or (
                info.width, info.height) != (rgb_msg.width, rgb_msg.height) or not (
                rgb_msg.header.frame_id == depth_msg.header.frame_id == info.header.frame_id):
                raise ValueError('RGB/depth/CameraInfo dimensions or frames do not match')
            if not (info.k[0] > 0 and info.k[4] > 0):
                raise ValueError('Invalid camera intrinsics')
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            result = self.model.track(bgr, persist=True, tracker='bytetrack.yaml',
                                      conf=float(self.p('conf')), imgsz=int(self.p('imgsz')),
                                      device=self.device, verbose=False)[0]
            out = result.plot()
            now = time.monotonic()
            # Test capture age AFTER inference as inference itself may be slow.
            age = (self.get_clock().now().nanoseconds - (
                rgb_msg.header.stamp.sec*10**9 + rgb_msg.header.stamp.nanosec))/1e9
            fresh = -0.05 <= age <= self.p('max_capture_age')
            if not fresh:
                self.latch('Image too old after inference')
            boxes = result.boxes
            ids = boxes.id.int().cpu().tolist() if boxes is not None and boxes.id is not None else []
            coords = boxes.xyxy.cpu().numpy() if ids else []
            kps = result.keypoints
            xy = kps.xy.cpu().numpy() if kps is not None else None
            conf = kps.conf.cpu().numpy() if kps is not None and kps.conf is not None else None
            found = False
            candidates = {}
            for i, (track_id, box) in enumerate(zip(ids, coords)):
                x1, y1, x2, y2 = box
                if self.selected_id is None and fresh and xy is not None and self.hand_up(xy, conf, i, y2-y1):
                    old = self.raised.get(track_id)
                    start, count = (now, 1) if old is None or now-old[1] > self.p('gesture_gap_seconds') else (old[0], old[2]+1)
                    candidates[track_id] = (start, now, count)
                    held = now-start
                    cv2.putText(out, f'ID {track_id}: {held:.1f}/{self.p("gesture_seconds"):.1f}s',
                                (int(x1), max(25, int(y1)-10)), 0, 0.6, (0,255,255), 2)
                    if held >= self.p('gesture_seconds') and count >= 4:
                        self.selected_id = track_id
                        self.id_pub.publish(Int32(data=track_id))
                if track_id != self.selected_id or self.lost or not fresh:
                    continue
                found = True
                u, v = int((x1+x2)/2), int((y1+y2)/2)
                self.pixel_pub.publish(Int32MultiArray(data=[track_id, u, v]))
                z, roi = self.measure(depth, depth_msg.encoding, box, xy, conf, i)
                cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 3)
                a,c,b,d = roi
                cv2.rectangle(out, (a,c), (b,d), (255,255,0), 2)
                if z is None:
                    self.latch('Invalid torso depth')
                    continue
                target = PointStamped()
                target.header = rgb_msg.header
                target.point.x = float((u-info.k[2])*z/info.k[0])
                target.point.y = float((v-info.k[5])*z/info.k[4])
                target.point.z = z
                self.target_pub.publish(target)
                self.visible_pub.publish(Bool(data=True))
                cv2.putText(out, f'ID {track_id} Z={z:.2f}m u={u} age={age:.2f}s',
                            (12, 65), 0, 0.65, (0,255,0), 2)
            if self.selected_id is None:
                # Preserve brief detection gaps up to gesture_gap_seconds.
                self.raised = {k: val for k, val in self.raised.items()
                               if now-val[1] <= self.p('gesture_gap_seconds')}
                self.raised.update(candidates)
            else:
                self.raised.clear()
                if not found:
                    self.latch('Selected ID not visible')
            if self.selected_id is None or self.lost or not found:
                self.invalid(rgb_msg.header)
            status = 'LOST / INVALID - RESET REQUIRED' if self.lost else (
                'Raise hand above head to select' if self.selected_id is None else 'TRACKING')
            cv2.putText(out, status, (12, 30), 0, 0.65, (0,0,255) if self.lost else (0,255,255), 2)
            cv2.line(out, (rgb_msg.width//2, 0), (rgb_msg.width//2, rgb_msg.height-1), (255,200,0), 1)
            output = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
            output.header = rgb_msg.header
            self.annotated_pub.publish(output)
        except Exception as exc:
            self.latch('Processing error')
            self.invalid(rgb_msg.header)
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
