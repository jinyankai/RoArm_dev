#!/usr/bin/env python3
"""
detector_node.py – ROS 2 (Jazzy) object-detection + 3-D localisation node.

* Subscribes
  - RGB image              (/ascamera_hp60c/…/rgb0/image)        sensor_msgs/Image (BGR8)
  - Organised depth cloud  (/ascamera_hp60c/…/depth0/points)     sensor_msgs/PointCloud2

* Publishes
  - DetectedObject3D (/detected_objects/point_cloud)             custom message
  - Overlay image    (~/overlay_image)                           sensor_msgs/Image

Author : jinyankai
"""

from __future__ import annotations

import os
import sys
import time
import logging
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from object_detection_3d.msg import DetectedObject3D  # auto-generated after build


@dataclass
class YoloDetection:
    """Convenience container."""
    xyxy: Tuple[int, int, int, int]
    conf: float
    cls_name: str


class DetectorNode(Node):
    """
    Runs YOLO v11-S on RGB images, extracts 3-D points inside each bounding-box,
    and publishes DetectedObject3D messages plus an overlay image.
    """

    def __init__(self) -> None:
        super().__init__("detector_node")
        self.declare_parameter("model_name", "yolov11s")        # Ultralytics hub name
        self.declare_parameter("score_threshold", 0.4)
        self.declare_parameter("use_gpu", True)

        self._score_threshold: float = self.get_parameter("score_threshold").value

        # ------------------------- load YOLO model --------------------------
        import torch
        from ultralytics import YOLO                         # pip install ultralytics>=v11

        want_gpu = self.get_parameter("use_gpu").value
        device = 0 if (want_gpu and torch.cuda.is_available()) else "cpu"
        self.get_logger().info(f"Loading YOLO on device: {device}")
        model_name: str = self.get_parameter("model_name").value
        self._model = YOLO(model_name).to(device)
        self._device = device

        # ------------------------- ROS comms --------------------------------
        self._bridge = CvBridge()
        self._pub_det = self.create_publisher(
            DetectedObject3D, "/detected_objects/point_cloud", 10
        )
        self._pub_overlay = self.create_publisher(
            Image, "~/overlay_image", qos_profile_sensor_data
        )

        # Use exact-time synchronisation (timestamps are hardware-sync’d)
        sub_img = Subscriber(
            self, Image, "/ascamera_hp60c/camera_publisher/rgb0/image", qos_profile_sensor_data
        )
        sub_pc = Subscriber(
            self, PointCloud2, "/ascamera_hp60c/camera_publisher/depth0/points", qos_profile_sensor_data
        )

        sync = ApproximateTimeSynchronizer([sub_img, sub_pc], queue_size=10, slop=0.05)
        sync.registerCallback(self._callback)

        self.get_logger().info("DetectorNode initialised")


    # --------------------------------------------------------------------- #
    #                                callbacks                              #
    # --------------------------------------------------------------------- #
    def _callback(self, img_msg: Image, cloud_msg: PointCloud2) -> None:
        """
        One synch’d RGB frame + organised point-cloud.
        """
        tic = time.perf_counter()

        # ----- RGB image to OpenCV BGR8
        frame: np.ndarray = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        # ----- YOLO inference
        yolo_out = self._model(frame, verbose=False, conf=self._score_threshold, device=self._device)[0]

        detections: List[YoloDetection] = []
        for xyxy, conf, cls_id in zip(yolo_out.boxes.xyxy.cpu().numpy(),
                                      yolo_out.boxes.conf.cpu().numpy(),
                                      yolo_out.boxes.cls.cpu().numpy()):
            x1, y1, x2, y2 = map(int, xyxy)
            cls_name = self._model.names[int(cls_id)]
            detections.append(YoloDetection((x1, y1, x2, y2), float(conf), cls_name))

        if not detections:
            return  # nothing to do

        # ----- point cloud to numpy (organised: height = img.height, width = img.width)
        cloud_np = np.array(list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"),
                                                 skip_nans=False))).reshape(
            cloud_msg.height, cloud_msg.width, 3
        )  # shape (H, W, 3) – may contain np.nan

        # ----- process each detection
        overlay = frame.copy()
        header = Header(stamp=img_msg.header.stamp, frame_id=cloud_msg.header.frame_id)

        for det in detections:
            x1, y1, x2, y2 = det.xyxy
            x1, y1 = max(x1, 0), max(y1, 0)
            x2, y2 = min(x2, cloud_msg.width - 1), min(y2, cloud_msg.height - 1)

            # slice points in bbox, flatten to N×3
            box_points = cloud_np[y1:y2 + 1, x1:x2 + 1, :].reshape(-1, 3)
            finite_mask = np.isfinite(box_points[:, 0])
            box_points = box_points[finite_mask]

            if box_points.size == 0:
                continue  # skip empty cloud

            # centroid
            centroid_xyz = box_points.mean(axis=0)
            centroid_msg = Point(x=float(centroid_xyz[0]),
                                 y=float(centroid_xyz[1]),
                                 z=float(centroid_xyz[2]))

            # pack subset as PointCloud2 (helper provides list-of-tuples)
            cloud_sub = pc2.create_cloud_xyz32(header, box_points.astype(np.float32))

            out_msg = DetectedObject3D(
                header=header,
                class_name=det.cls_name,
                confidence=det.conf,
                cloud=cloud_sub,
                centroid=centroid_msg,
            )
            self._pub_det.publish(out_msg)

            # draw rectangle & label on overlay
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det.cls_name} {det.conf:.2f}"
            cv2.putText(overlay, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # publish overlay image
        overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = header
        self._pub_overlay.publish(overlay_msg)

        self.get_logger().debug(f"Frame processed – {len(detections)} dets "
                                f"in {(time.perf_counter() - tic)*1000:.1f} ms")


def main(argv: List[str] | None = None) -> None:         # entry-point for `ros2 run`
    rclpy.init(args=argv)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":                               # python detector_node.py
    main(sys.argv)
