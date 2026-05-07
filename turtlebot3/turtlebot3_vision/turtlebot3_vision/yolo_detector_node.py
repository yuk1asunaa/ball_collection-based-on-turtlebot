#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import message_filters
import torch
torch.set_num_threads(1)
import numpy as np
import cv2
from ultralytics import YOLO
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RgbdYoloNode(Node):
    def __init__(self):
        super().__init__('rgbd_yolo_node')
        self.bridge = CvBridge()
        
        # Declare parameter for model path
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.05)
        self.declare_parameter('target_class_id', 32)
        self.declare_parameter('color_topic', '/depth_camera/image_raw')
        self.declare_parameter('depth_topic', '/depth_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_camera/camera_info')
        self.declare_parameter('sync_queue_size', 2)
        self.declare_parameter('sync_slop', 0.2)
        self.declare_parameter('inference_imgsz', 320)
        self.declare_parameter('inference_iou', 0.45)
        self.declare_parameter('max_det', 20)
        self.declare_parameter('inference_device', 'cpu')
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('publish_empty', True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/vision/debug_image')
        self.declare_parameter('min_valid_depth_m', 0.05)
        self.declare_parameter('max_valid_depth_m', 10.0)
        self.declare_parameter('depth_center_kernel_px', 7)
        self.declare_parameter('position_smoothing_alpha', 0.45)
        self.declare_parameter('max_track_jump_m', 1.2)
        self.declare_parameter('log_timing_interval_sec', 5.0)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.target_class_id = self.get_parameter('target_class_id').get_parameter_value().integer_value
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        sync_queue_size = self.get_parameter('sync_queue_size').get_parameter_value().integer_value
        sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value
        self.inference_imgsz = self.get_parameter('inference_imgsz').get_parameter_value().integer_value
        self.inference_iou = self.get_parameter('inference_iou').get_parameter_value().double_value
        self.max_det = self.get_parameter('max_det').get_parameter_value().integer_value
        self.inference_device = self.get_parameter('inference_device').get_parameter_value().string_value
        self.process_every_n_frames = max(1, self.get_parameter('process_every_n_frames').get_parameter_value().integer_value)
        self.publish_empty = self.get_parameter('publish_empty').get_parameter_value().bool_value
        self.publish_debug_image = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        self.debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.min_valid_depth_m = self.get_parameter('min_valid_depth_m').get_parameter_value().double_value
        self.max_valid_depth_m = self.get_parameter('max_valid_depth_m').get_parameter_value().double_value
        self.depth_center_kernel_px = max(3, self.get_parameter('depth_center_kernel_px').get_parameter_value().integer_value)
        self.position_smoothing_alpha = float(np.clip(
            self.get_parameter('position_smoothing_alpha').get_parameter_value().double_value,
            0.0,
            1.0,
        ))
        self.max_track_jump_m = max(0.05, self.get_parameter('max_track_jump_m').get_parameter_value().double_value)
        self.log_timing_interval_sec = self.get_parameter('log_timing_interval_sec').get_parameter_value().double_value

        self._frame_count = 0
        self._timing_window_count = 0
        self._timing_window_cb_ms = 0.0
        self._timing_window_inf_ms = 0.0
        self._last_timing_log_t = time.time()
        self._smoothed_points = []
        
        # load YOLO model
        self.model = YOLO(model_path) 
        
        #camera intrinsics parameters (/camera/camera_info)
        self.camera_info_received = False

        # Subscribe camera intrinsics from the same camera stream family used for RGB-D.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile)

        #create two independent subscribers
        self.color_sub = message_filters.Subscriber(self, Image, color_topic, qos_profile=qos_profile)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile)

        #alignment
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=sync_queue_size, slop=sync_slop)
        self.ts.registerCallback(self.sync_callback)

        self.target_pose_pub = self.create_publisher(PoseArray, '/vision/target_poses', 10)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.get_logger().info(
            f"RGB-D YOLO, The time alignment node has been activated.: color={color_topic}, depth={depth_topic}, "
            f"camera_info={camera_info_topic}, "
            f"conf={self.confidence_threshold}, class_id={self.target_class_id}, "
            f"slop={sync_slop}, queue={sync_queue_size}, imgsz={self.inference_imgsz}, "
            f"device={self.inference_device}, process_every_n_frames={self.process_every_n_frames}, "
            f"debug_image={self.publish_debug_image}({self.debug_image_topic})")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]  # fx
            self.fy = msg.k[4]  # fy
            self.cx = msg.k[2]  # cx
            self.cy = msg.k[5]  # cy
            self.camera_info_received = True
            self.get_logger().info(f"Camera intrinsics parameters have been updated.: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def sync_callback(self, color_msg, depth_msg):
        cb_t0 = time.perf_counter()

        # Validate camera info received before processing
        if not self.camera_info_received:
            self.get_logger().warn("Camera info not received yet, skipping frame")
            return

        current_time = time.perf_counter()
        if not hasattr(self, '_last_process_time'):
            self._last_process_time = 0.0
            
        if (current_time - self._last_process_time) < 1.0: # Only process 1 frame per second explicitly, dump others
            return
            
        self._last_process_time = current_time

        # Convert ROS images to numpy matrices
        cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")

        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # Perform YOLO inference on the color image
        inf_t0 = time.perf_counter()
        results = self.model(
            cv_color,
            verbose=False,
            conf=self.confidence_threshold,
            iou=self.inference_iou,
            imgsz=self.inference_imgsz,
            max_det=self.max_det,
            device=self.inference_device,
            classes=[int(self.target_class_id)]
        )
        inf_ms = (time.perf_counter() - inf_t0) * 1000.0

        points = []
        depth_invalid_count = 0
        depth_valid_count = 0
        debug_image = cv_color.copy() if self.publish_debug_image else None
        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) != int(self.target_class_id):
                    continue

                conf = float(box.conf[0]) if box.conf is not None else 0.0

                # Obtain the center pixel in RGB image coordinates.
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u_rgb = int((x1 + x2) / 2.0)
                v_rgb = int((y1 + y2) / 2.0)

                if self.publish_debug_image and debug_image is not None:
                    x1_i = int(max(0, min(cv_color.shape[1] - 1, x1)))
                    y1_i = int(max(0, min(cv_color.shape[0] - 1, y1)))
                    x2_i = int(max(0, min(cv_color.shape[1] - 1, x2)))
                    y2_i = int(max(0, min(cv_color.shape[0] - 1, y2)))
                    cv2.rectangle(debug_image, (x1_i, y1_i), (x2_i, y2_i), (0, 255, 0), 2)
                    cv2.circle(debug_image, (u_rgb, v_rgb), 3, (0, 0, 255), -1)
                    cv2.putText(
                        debug_image,
                        f"ball {conf:.2f}",
                        (x1_i, max(0, y1_i - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

                # Map RGB pixel coordinates to depth image coordinates.
                color_h, color_w = cv_color.shape[:2]
                depth_h, depth_w = cv_depth.shape[:2]
                sx = float(depth_w) / float(color_w)
                sy = float(depth_h) / float(color_h)

                u1 = int(round(float(x1) * sx))
                v1 = int(round(float(y1) * sy))
                u2 = int(round(float(x2) * sx))
                v2 = int(round(float(y2) * sy))

                u_min = min(u1, u2)
                u_max = max(u1, u2)
                v_min = min(v1, v2)
                v_max = max(v1, v2)

                u1 = int(np.clip(u_min, 0, depth_w - 1))
                u2 = int(np.clip(u_max, 0, depth_w - 1))
                v1 = int(np.clip(v_min, 0, depth_h - 1))
                v2 = int(np.clip(v_max, 0, depth_h - 1))

                u = int(round(u_rgb * sx))
                v = int(round(v_rgb * sy))

                # Clip to depth image bounds
                u = int(np.clip(u, 0, depth_w - 1))
                v = int(np.clip(v, 0, depth_h - 1))

                kernel_half = self.depth_center_kernel_px // 2
                pu1 = max(0, u - kernel_half)
                pu2 = min(depth_w - 1, u + kernel_half)
                pv1 = max(0, v - kernel_half)
                pv2 = min(depth_h - 1, v + kernel_half)

                # Prefer center patch depth to reduce edge/background contamination.
                su1 = max(u1, pu1)
                su2 = min(u2, pu2)
                sv1 = max(v1, pv1)
                sv2 = min(v2, pv2)
                if su1 > su2 or sv1 > sv2:
                    su1, su2, sv1, sv2 = pu1, pu2, pv1, pv2

                if depth_msg.encoding == '16UC1':
                    roi_depth_m = cv_depth[sv1:sv2 + 1, su1:su2 + 1].astype(np.float32) / 1000.0
                elif depth_msg.encoding == '32FC1':
                    roi_depth_m = cv_depth[sv1:sv2 + 1, su1:su2 + 1].astype(np.float32)
                else:
                    self.get_logger().warn(f"Unsupported depth encoding: {depth_msg.encoding}")
                    continue

                valid = np.isfinite(roi_depth_m) & (roi_depth_m >= self.min_valid_depth_m) & (roi_depth_m <= self.max_valid_depth_m)
                if not np.any(valid):
                    depth_invalid_count += 1
                    if self.publish_debug_image and debug_image is not None:
                        cv2.putText(
                            debug_image,
                            'no depth',
                            (u_rgb, min(cv_color.shape[0] - 1, v_rgb + 16)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            (0, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )
                    continue

                depth_m = float(np.median(roi_depth_m[valid]))
                depth_valid_count += 1

                Z_c = depth_m

                # Reproject with scaled intrinsics that match depth resolution.
                fx_eff = self.fx * sx
                fy_eff = self.fy * sy
                cx_eff = self.cx * sx
                cy_eff = self.cy * sy

                # back projection
                X_c = (u - cx_eff) * Z_c / fx_eff
                Y_c = (v - cy_eff) * Z_c / fy_eff

                points.append([X_c, Y_c, Z_c])

        if points:
            points = self.smooth_points(points)
            self.publish_target_poses(points, depth_msg.header)
        else:
            self._smoothed_points = []
            self.get_logger().debug("No valid ball detections in this frame")
            if self.publish_empty:
                self.publish_target_poses([], depth_msg.header)

        if depth_invalid_count > 0:
            self.get_logger().debug(
                f"Depth rejected for {depth_invalid_count} detections; valid_depth={depth_valid_count}, "
                f"range=[{self.min_valid_depth_m:.2f},{self.max_valid_depth_m:.2f}]m")

        if self.publish_debug_image and debug_image is not None and self.debug_image_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = color_msg.header
            self.debug_image_pub.publish(debug_msg)

        cb_ms = (time.perf_counter() - cb_t0) * 1000.0
        self._timing_window_count += 1
        self._timing_window_cb_ms += cb_ms
        self._timing_window_inf_ms += inf_ms
        now_t = time.time()
        if (now_t - self._last_timing_log_t) >= self.log_timing_interval_sec and self._timing_window_count > 0:
            avg_cb = self._timing_window_cb_ms / self._timing_window_count
            avg_inf = self._timing_window_inf_ms / self._timing_window_count
            self.get_logger().info(
                f"Timing window: samples={self._timing_window_count}, avg_cb={avg_cb:.1f}ms, avg_inf={avg_inf:.1f}ms")
            self._timing_window_count = 0
            self._timing_window_cb_ms = 0.0
            self._timing_window_inf_ms = 0.0
            self._last_timing_log_t = now_t

    def publish_target_poses(self, points, header):
        pose_array = PoseArray()
        pose_array.header = header

        for p in points:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.target_pose_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(points)} target poses")

    def smooth_points(self, points):
        if not points:
            return []

        points_np = [np.array(p, dtype=np.float32) for p in points]
        if not self._smoothed_points:
            self._smoothed_points = [p.copy() for p in points_np]
            return [[float(v) for v in p] for p in self._smoothed_points]

        updated = []
        used_prev = set()
        for p in points_np:
            best_i = -1
            best_dist = float('inf')
            for i, prev in enumerate(self._smoothed_points):
                if i in used_prev:
                    continue
                dist = float(np.linalg.norm(p - prev))
                if dist < best_dist:
                    best_dist = dist
                    best_i = i

            if best_i >= 0 and best_dist <= self.max_track_jump_m:
                used_prev.add(best_i)
                prev = self._smoothed_points[best_i]
                p = self.position_smoothing_alpha * p + (1.0 - self.position_smoothing_alpha) * prev

            updated.append(p)

        self._smoothed_points = updated
        return [[float(v) for v in p] for p in self._smoothed_points]

def main(args=None):
    rclpy.init(args=args)
    node = RgbdYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()