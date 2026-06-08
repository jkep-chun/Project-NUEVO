import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from bridge_interfaces.msg import FusedPose

class SLAMBridge(Node):
    def __init__(self):
        super().__init__('slam_bridge')

        self._sub_slam_pose = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/pose', 
            self._on_slam_pose, 
            10
        )

        self._pub_slam_pose = self.create_publisher(
            FusedPose, 
            '/slam_pose_update', 
            10
        )

        self.get_logger().info("SLAM Bridge Initialized: /pose (m) -> /slam_pose_update (mm)")

    def _on_slam_pose(self, msg: PoseWithCovarianceStamped):
        # Simplified message to NUEVO
        out = FusedPose()

        # Metres to Millimetres
        out.x = msg.pose.pose.position.x * 1000.0
        out.y = msg.pose.pose.position.y * 1000.0

        # Quaternion to simple Theta (Yaw)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        out.theta = 2.0 * math.atan2(qz, qw)

        self._pub_slam_pose.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
