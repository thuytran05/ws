#!/usr/bin/env python3
"""ROS 2 Jazzy: YOLO Pose + persistent person tracking + raise-hand selection.

Stage 1 only: RGB image and pixel coordinates; never commands robot motion.
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, Int32MultiArray
from std_srvs.srv import Trigger
from ultralytics import YOLO
import torch


class PersonPoseNode(Node):
    def __init__(self):
        super().__init__('person_pose_node')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('model', 'yolo26n-pose.pt')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('gesture_seconds', 2.0)
        self.declare_parameter('gesture_gap_seconds', 0.75)
        self.declare_parameter('keypoint_conf', 0.50)
        self.declare_parameter('device', 'auto')

        model_name = self.get_parameter('model').value
        device_param = str(self.get_parameter('device').value)
        self.device = ('0' if torch.cuda.is_available() else 'cpu') if device_param == 'auto' else device_param
        self.get_logger().info(f'Loading YOLO Pose {model_name}; device={self.device}')
        self.model = YOLO(model_name)
        self.bridge = CvBridge()
        self.selected_id = None
        # Each value: (time first raised, time last observed raised, consecutive frames)
        self.raised = {}
        self.last_seen_selected = None

        self.annotated_pub = self.create_publisher(Image, '/person_tracking/image_annotated', qos_profile_sensor_data)
        self.id_pub = self.create_publisher(Int32, '/person_tracking/selected_id', 10)
        self.pixel_pub = self.create_publisher(Int32MultiArray, '/person_tracking/target_pixel', 10)
        self.visible_pub = self.create_publisher(Bool, '/person_tracking/target_visible', 10)
        self.reset_srv = self.create_service(Trigger, '/person_tracking/reset', self.reset_selection)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.on_image, qos_profile_sensor_data)
        self.get_logger().info('Ready. Hold a hand ABOVE the head for 2 seconds to select target.')

    def reset_selection(self, request, response):
        self.selected_id = None
        self.raised.clear()
        self.last_seen_selected = None
        self.id_pub.publish(Int32(data=-1))
        response.success = True
        response.message = 'Target selection cleared'
        self.get_logger().info('Selection reset')
        return response

    def hand_above_head(self, xy, conf, i, person_height):
        # COCO: nose=0, left wrist=9, right wrist=10
        if conf is None or conf[i, 0] < self.get_parameter('keypoint_conf').value:
            return False
        head_y = float(xy[i, 0, 1])
        margin = max(10.0, person_height * 0.035)
        threshold = self.get_parameter('keypoint_conf').value
        for wrist in (9, 10):
            if conf[i, wrist] >= threshold and float(xy[i, wrist, 1]) < head_y - margin:
                return True
        return False

    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.track(
                bgr, persist=True, tracker='bytetrack.yaml',
                conf=float(self.get_parameter('conf').value),
                imgsz=int(self.get_parameter('imgsz').value),
                device=self.device, verbose=False,
            )
            result = results[0]
            out = result.plot()
            now = time.monotonic()
            seen_raised = set()
            target_visible = False
            boxes = result.boxes
            if boxes is not None and boxes.id is not None and result.keypoints is not None:
                ids = boxes.id.int().cpu().tolist()
                xyxy = boxes.xyxy.cpu().numpy()
                xy = result.keypoints.xy.cpu().numpy()
                kp_conf = result.keypoints.conf
                kp_conf = kp_conf.cpu().numpy() if kp_conf is not None else None
                for i, (track_id, box) in enumerate(zip(ids, xyxy)):
                    x1, y1, x2, y2 = [int(v) for v in box]
                    h = max(1, y2 - y1)
                    hand_up = self.hand_above_head(xy, kp_conf, i, h)
                    if self.selected_id is None and hand_up:
                        seen_raised.add(track_id)
                        old = self.raised.get(track_id)
                        if old is None or now - old[1] > float(self.get_parameter('gesture_gap_seconds').value):
                            start, count = now, 1
                        else:
                            start, count = old[0], old[2] + 1
                        self.raised[track_id] = (start, now, count)
                        held = now - start
                        cv2.putText(out, f'ID {track_id}: raise {held:.1f}s / 2.0s',
                                    (max(0, x1), max(25, y1 - 12)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        if held >= float(self.get_parameter('gesture_seconds').value) and count >= 4:
                            self.selected_id = track_id
                            self.id_pub.publish(Int32(data=track_id))
                            self.get_logger().info(f'SELECTED person ID={track_id}')
                    if track_id == self.selected_id:
                        target_visible = True
                        self.last_seen_selected = now
                        u, v = (x1+x2)//2, (y1+y2)//2
                        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 3)
                        cv2.putText(out, f'SELECTED ID {track_id} ({u},{v})',
                                    (max(0, x1), max(28, y1 - 35)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                        # [tracking ID, horizontal pixel, vertical pixel]
                        self.pixel_pub.publish(Int32MultiArray(data=[track_id, u, v]))
            if self.selected_id is None:
                self.raised = {k: val for k, val in self.raised.items() if k in seen_raised}
            else:
                self.raised.clear()
                if not target_visible:
                    cv2.putText(out, 'TARGET LOST - no automatic reselection', (12, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if self.selected_id is None:
                cv2.putText(out, 'Raise one hand above head for 2 seconds', (12, 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            self.visible_pub.publish(Bool(data=target_visible))
            output = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
            output.header = msg.header  # retain image capture timestamp and frame_id
            self.annotated_pub.publish(output)
        except Exception as exc:
            self.get_logger().error(f'Processing error: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonPoseNode()
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
