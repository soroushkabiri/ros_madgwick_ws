#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math

class YawExtractor(Node):
    def __init__(self):
        super().__init__('yaw_extractor')
        self.subscription = self.create_subscription(Imu,'/imu/data',self.imu_callback,10)
        self.publisher = self.create_publisher(Float32, '/imu/yaw', 10)
        self.get_logger().info('Yaw Extractor Node has been started.')
        self.last_yaw_deg = 0.0  # Initial value if no IMU yet
        #self.timer = self.create_timer(0.05, self.print_yaw)


    def imu_callback(self, msg):
        # Extract quaternion
        q = msg.orientation
        yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # Convert to degrees
        yaw_deg = math.degrees(yaw_rad)
        # Publish yaw in degrees
        yaw_msg = Float32()
        yaw_msg.data = yaw_deg
        self.last_yaw_deg=yaw_deg
        self.publisher.publish(yaw_msg)
        #self.get_logger().info(f"Published yaw: {yaw_deg:.2f} degrees")

    #def print_yaw(self):

        #print(f"Current IMU Yaw: {self.last_yaw_deg:.2f} deg")  # Print to terminal

    def quaternion_to_yaw(self, x, y, z, w):
        # Convert quaternion to yaw angle (Z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = YawExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
