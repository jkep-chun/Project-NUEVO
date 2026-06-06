import math
import rclpy
from robot.hardware_map import WHEEL_BASE, WHEEL_DIAMETER, LEFT_WHEEL_MOTOR, RIGHT_WHEEL_MOTOR, LEFT_WHEEL_DIR_INVERTED, RIGHT_WHEEL_DIR_INVERTED
from robot.robot import Robot
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from bridge_interfaces.msg import SensorKinematics, DCSetVelocity
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist

class NavigationBridge(Node):
    def __init__(self):
        super().__init__('navigation_bridge')

        self.WHEEL_BASE_M = WHEEL_BASE / 1000.0
        self.WHEEL_DIAMETER_M = WHEEL_DIAMETER / 1000.0
        self.ENCODER_PPR = Robot.ENCODER_PPR
        self.ticks_per_m = self.ENCODER_PPR / (self.WHEEL_DIAMETER_M * math.pi)

        # Subscribers
        self._sub_sensor_kinematics = self.create_subscription(SensorKinematics, '/sensor_kinematics', self._on_sensor_kinematics, 10)
        self._sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_twist, 10)

        # Publishers
        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self._pub_cmd_vel = self.create_publisher(DCSetVelocity, '/dc_set_velocity', 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Navigation Bridge Node Started")
    
    def _on_sensor_kinematics(self, msg: SensorKinematics):
        odom = Odometry()

        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Convert position mm -> m
        odom.pose.pose.position.x = msg.x / 1000.0
        odom.pose.pose.position.y = msg.y / 1000.0
        odom.pose.pose.position.z = 0.0

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
        self._pub_odom.publish(odom)

    def _on_cmd_twist(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Extract differential drive velocities
        l_vel_m_s = linear - (angular * self.WHEEL_BASE_M / 2.0)
        r_vel_m_s = linear + (angular * self.WHEEL_BASE_M / 2.0)

        if LEFT_WHEEL_DIR_INVERTED: l_vel_m_s = -l_vel_m_s
        if RIGHT_WHEEL_DIR_INVERTED: r_vel_m_s = -r_vel_m_s

        # Publish
        l_msg = DCSetVelocity()
        l_msg.motor_number = LEFT_WHEEL_MOTOR
        l_msg.target_ticks = int(l_vel_m_s * self.ticks_per_m)
        self._pub_cmd_vel.publish(l_msg)

        r_msg = DCSetVelocity()
        r_msg.motor_number = RIGHT_WHEEL_MOTOR
        r_msg.target_ticks = int(r_vel_m_s * self.ticks_per_m)
        self._pub_cmd_vel.publish(r_msg)
        

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