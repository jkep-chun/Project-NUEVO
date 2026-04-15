from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vision",
            executable="vision_node",
            name="vision_node",
            output="screen",
            parameters=[
                {
                    "image_topic": "/camera/image_raw",
                    "model_path": "",
                    "labels_path": "",
                }
            ],
        )
    ])
