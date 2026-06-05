import math
import rclpy
from rclpy.node import Node
from bridge_interfaces.msg import SensorKinematics
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class NavigationBridge(Node):
    def __init__(self):
        super().__init__('navigation_bridge')

        self._sub = self.create_subscription(
            msg_type=SensorKinematics,
            topic='/sensor_kinematics',
            callback=self._callback,
            qos_profile=10
        )

        self._pub = self.create_publisher(
            msg_type=Odometry,
            topic='/odom',
            qos_profile=10
        )

        self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Navigation Bridge Node Started")
    
    def _callback(self, msg: SensorKinematics):
        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Convert position mm -> m
        odom.pose.pose.position.x = msg.x / 1000.0
        odom.pose.pose.position.y = msg.y / 1000.0
        odom.pose.pose.position.z = 0

        # Sensor_kinematics conversion to quaternion
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(msg.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(msg.theta / 2.0)

        # Convert twist  mm/s -> m/s
        odom.twist.twist.linear.x = msg.vx / 1000.0
        odom.twist.twist.linear.y = msg.vy / 1000.0
        odom.twist.twist.angular.z = msg.v_theta # Already in rad/s

        # Transform broadcaster
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = odom.pose.pose.orientation.x
        t.transform.rotation.y = odom.pose.pose.orientation.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w

        self._tf_broadcaster.sendTransform(t)

        # Publish the configured message
        self._pub.publish(odom)
        

def main(args=None):
    rclpy.init(args=args)
    node = NavigationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()