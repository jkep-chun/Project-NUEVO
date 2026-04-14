from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("model_path", "")
        self.declare_parameter("labels_path", "")

        self.image_topic = self.get_parameter("image_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.labels_path = self.get_parameter("labels_path").value

        self.publisher = self.create_publisher(String, "/face_detected", 10)
        self.timer = self.create_timer(1.0, self.publish_test)

        self.get_logger().info("Vision node started")
        self.get_logger().info(f"Configured image topic: {self.image_topic}")

        if self.model_path:
            if os.path.exists(self.model_path):
                self.get_logger().info(f"Model path found: {self.model_path}")
            else:
                self.get_logger().warn(f"Model path does not exist: {self.model_path}")
        else:
            self.get_logger().warn("No model_path provided yet")

        if self.labels_path:
            if os.path.exists(self.labels_path):
                self.get_logger().info(f"Labels path found: {self.labels_path}")
            else:
                self.get_logger().warn(f"Labels path does not exist: {self.labels_path}")
        else:
            self.get_logger().warn("No labels_path provided yet")

        self.get_logger().info("Vision package scaffold ready for face recognition integration")

    def publish_test(self):
        msg = String()
        msg.data = "test_face_detected"
        self.publisher.publish(msg)
        self.get_logger().info("Published test face_detected")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
