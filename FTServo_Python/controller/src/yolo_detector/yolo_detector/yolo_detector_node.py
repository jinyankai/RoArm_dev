
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import message_filters
# (ros_env) user@computer:~$
# Source your main ROS2 installation
# source /opt/ros/jazzy/setup.bash

# Navigate to your ROS2 workspace
# cd ~/ros2_ws

# Now, build your package. Colcon will use the Python from your conda env.
# colcon build --packages-select yolo_detector

# Make sure you are in a terminal with the conda env active
# (ros_env) user@computer:~$

# Navigate to your workspace
# cd ~/ros2_ws

# Source the local workspace setup file (this also sources the main ROS2 file if not already done)
# source install/setup.bash

# Now you can run your node
# ros2 run yolo_detector yolo_detector_node --ros-args -p yolo_model:='yolov8n.pt'
# ros2 run yolo_detector yolo_detector_node --ros-args -p yolo_model:='/path/to/your/yolov8n.pt'

class YoloDetectorNode(Node):
    """
    A ROS2 node that performs YOLO object detection on an RGB image, visualizes
    the detections, and uses a synchronized point cloud to determine the 3D
    location of the detected objects.
    """

    def __init__(self):
        """
        Initializes the node, loads the YOLO model, sets up subscribers with a
        time synchronizer, and creates a publisher for visualization.
        """
        super().__init__('yolo_detector_node')

        # --- Parameters ---
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('image_topic', '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.declare_parameter('pointcloud_topic', '/ascamera_hp60c/camera_publisher/depth0/points')
        self.declare_parameter('visualization_topic', '/yolo_detections/image')

        model_path = self.get_parameter('yolo_model').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        visualization_topic = self.get_parameter('visualization_topic').get_parameter_value().string_value

        # --- Initialization ---
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            rclpy.shutdown()
            return

        self.cv_bridge = CvBridge()

        # --- Publisher for Visualization ---
        self.vis_publisher_ = self.create_publisher(Image, visualization_topic, 10)
        self.get_logger().info(f"Publishing visualization to: {visualization_topic}")

        self.get_logger().info("Node initialized successfully.")

        # --- Subscribers with Message Filters Time Synchronizer ---
        self.get_logger().info(f"Subscribing to Image topic: {image_topic}")
        self.get_logger().info(f"Subscribing to PointCloud2 topic: {pointcloud_topic}")

        image_sub = message_filters.Subscriber(self, Image, image_topic)
        pointcloud_sub = message_filters.Subscriber(self, PointCloud2, pointcloud_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, pointcloud_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, image_msg, pointcloud_msg):
        """
        This callback is triggered only when synchronized image and point cloud
        messages are received. It performs detection, localization, and visualization.
        """
        self.get_logger().info('Received synchronized messages.', once=True)

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # Perform YOLO detection
        results = self.model(cv_image, verbose=False)

        # Process detections
        for box in results[0].boxes:
            if box.conf[0] > self.conf_threshold:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                # --- Draw bounding box and label on the image ---
                label = f"{class_name}: {confidence:.2f}"
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # --- Get 3D coordinates from Point Cloud ---
                try:
                    points_at_center = list(pc2.read_points(
                        pointcloud_msg,
                        field_names=("x", "y", "z"),
                        skip_nans=False,
                        uvs=[(center_x, center_y)]
                    ))

                    if points_at_center:
                        # The point is a tuple (x, y, z). We check each element.
                        # Explicitly convert to a numpy array for robust NaN checking.
                        point = np.array(points_at_center[0], dtype=np.float32)

                        # Check if any of the coordinates are NaN (Not a Number)
                        if not np.any(np.isnan(point)):
                            self.get_logger().info(
                                f"Detected '{class_name}' at 3D: [x: {point[0]:.3f}, y: {point[1]:.3f}, z: {point[2]:.3f}]"
                            )
                        else:
                            self.get_logger().warn(
                                f"Depth data invalid for '{class_name}' at pixel ({center_x}, {center_y}).")
                except Exception as e:
                    self.get_logger().error(f"Error processing point cloud data: {e}")

        # --- Publish the visualized image ---
        try:
            vis_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            vis_msg.header = image_msg.header  # Keep the same timestamp and frame_id
            self.vis_publisher_.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f"Could not convert or publish visualization image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
