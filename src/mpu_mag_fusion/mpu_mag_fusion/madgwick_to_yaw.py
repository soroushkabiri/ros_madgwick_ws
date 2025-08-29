#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math
import time

class YawExtractor(Node):
    def __init__(self):
        super().__init__('yaw_extractor')
        self.subscription_L = self.create_subscription(Imu,'/imu/data/L',self.imu_callback_L,10)
        self.publisher_L = self.create_publisher(Float32, '/robot0_0/yaw_deg', 10)

        self.subscription_F1 = self.create_subscription(Imu,'/imu/data/F1',self.imu_callback_F1,10)
        self.publisher_F1 = self.create_publisher(Float32, '/robot0_1/yaw_deg', 10)

        # Offsets
        self.offset_yaw_L = None
        self.offset_yaw_F1 = None
        # Startup timestamp
        self.start_time = time.time()

        self.get_logger().info('Yaw Extractor Node has been started.')
        #self.last_yaw_deg_L = 0.0  # Initial value if no IMU yet
        #self.last_yaw_deg_F1 = 0.0  # Initial value if no IMU yet

        #self.timer = self.create_timer(0.05, self.print_yaw)


    def imu_callback_L(self, msg):
        # Extract quaternion
        q = msg.orientation
        yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # Convert to degrees
        yaw_deg = math.degrees(yaw_rad)



        # Publish yaw in degrees
        #yaw_msg = Float32()
        #yaw_msg.data = yaw_deg
        #self.last_yaw_deg_L=yaw_deg
        #self.publisher_L.publish(yaw_msg)




        # Set offset after 0.5s from startup (first valid reading)
        if self.offset_yaw_L is None and (time.time() - self.start_time) > 0.5:
            self.offset_yaw_L = yaw_deg
            self.get_logger().info(f"Leader yaw zeroed at {yaw_deg:.2f}°")

        yaw_adj = yaw_deg - (self.offset_yaw_L if self.offset_yaw_L is not None else yaw_deg)
        yaw_adj = (yaw_adj + 180) % 360 - 180  # Normalize to [-180, 180]

        self.publisher_L.publish(Float32(data=yaw_adj))





    def imu_callback_F1(self, msg):
        # Extract quaternion
        q = msg.orientation
        yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # Convert to degrees
        yaw_deg = math.degrees(yaw_rad)



        # Publish yaw in degrees
        #yaw_msg = Float32()
        #yaw_msg.data = yaw_deg
        #self.last_yaw_deg_F1=yaw_deg
        #self.publisher_F1.publish(yaw_msg)


        if self.offset_yaw_F1 is None and (time.time() - self.start_time) > 0.5:
            self.offset_yaw_F1 = yaw_deg
            self.get_logger().info(f"Follower 1 yaw zeroed at {yaw_deg:.2f}°")

        yaw_adj = yaw_deg - (self.offset_yaw_F1 if self.offset_yaw_F1 is not None else yaw_deg)
        yaw_adj = (yaw_adj + 180) % 360 - 180

        self.publisher_F1.publish(Float32(data=yaw_adj))


    def quaternion_to_yaw(self, x, y, z, w):
        # Convert quaternion to yaw angle (Z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = YawExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
