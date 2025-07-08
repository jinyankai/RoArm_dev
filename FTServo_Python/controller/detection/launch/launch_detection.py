from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="object_detection_3d",
            executable="detector_node.py",
            name="detector",
            output="screen",
            parameters=[{
                "score_threshold": 0.4,
                "model_name": "yolov11s",      # any Ultralytics-compatible model path
                "use_gpu": True,
            }],
        )
    ])
