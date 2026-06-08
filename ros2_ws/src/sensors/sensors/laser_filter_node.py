import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter')
        
        # Default parameters from hardware_map.py
        # LIDAR_FOV_DEG = (-70, 70.0)
        # LIDAR_RANGE_MIN_MM = 70.0
        # LIDAR_RANGE_MAX_MM = 3500.0
        
        self.declare_parameter('fov_min_deg', -70.0)
        self.declare_parameter('fov_max_deg', 70.0)
        self.declare_parameter('lidar_yaw_deg', 0.0)
        self.declare_parameter('range_min_m', 0.07)
        self.declare_parameter('range_max_m', 3.5)
        
        self.update_parameters()
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1,
            ))
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info('Laser Filter Node initialized')

    def update_parameters(self):
        self.fov_min_rad = math.radians(self.get_parameter('fov_min_deg').value)
        self.fov_max_rad = math.radians(self.get_parameter('fov_max_deg').value)
        self.lidar_yaw_rad = math.radians(self.get_parameter('lidar_yaw_deg').value)
        self.range_min = self.get_parameter('range_min_m').value
        self.range_max = self.get_parameter('range_max_m').value
        
        self.get_logger().info(f'Filter settings: FOV [{self.get_parameter("fov_min_deg").value}, {self.get_parameter("fov_max_deg").value}] deg, '
                               f'Range [{self.range_min}, {self.range_max}] m, '
                               f'Lidar Yaw: {self.get_parameter("lidar_yaw_deg").value} deg')

    def parameters_callback(self, params):
        for param in params:
            if param.name in ['fov_min_deg', 'fov_max_deg', 'lidar_yaw_deg', 'range_min_m', 'range_max_m']:
                self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        
        # Need to handle the case where update_parameters might be called with old values 
        # but this is a simple node so we just re-read all.
        self.update_parameters()
        return rclpy.parameter.SetParametersResult(successful=True)

    def listener_callback(self, msg):
        ranges = np.array(msg.ranges)
        # LaserScan angles are relative to the lidar's own zero-angle
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        
        # Step 1: Basic range filtering based on message limits and our parameters
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if self.range_min > 0.0:
            valid &= (ranges >= self.range_min)
        if self.range_max > 0.0:
            valid &= (ranges <= self.range_max)
            
        # Step 2: FOV filter
        # Rotate angles by lidar_yaw_rad to get robot-frame angles (centred on lidar)
        # This assumes 0 deg is robot forward.
        robot_angles = angles + self.lidar_yaw_rad
        # Wrap to [-pi, pi]
        robot_angles = (robot_angles + math.pi) % (2 * math.pi) - math.pi
        
        valid &= (robot_angles >= self.fov_min_rad) & (robot_angles <= self.fov_max_rad)
        
        # Create filtered message
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        
        # "Remove" scans outside FOV by setting to inf
        # Some consumers prefer inf, others NaN. AMCL typically handles inf.
        new_ranges = np.where(valid, ranges, float('inf'))
        filtered_msg.ranges = new_ranges.tolist()
        
        if len(msg.intensities) > 0:
            intensities = np.array(msg.intensities)
            new_intensities = np.where(valid, intensities, 0.0)
            filtered_msg.intensities = new_intensities.tolist()
        
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
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
