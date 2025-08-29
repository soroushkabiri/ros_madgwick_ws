# imu_mag_wifi_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import math
from geometry_msgs.msg import Quaternion
import socket
import requests

ESP32_IP_F2 = "192.168.1.119"  # Default IP
ESP32_IP_F1 = "192.168.1.120"  # Default IP
ESP32_IP_L = "192.168.1.121"  # Default IP

class IMUWIFINode(Node):
    def __init__(self):
        super().__init__('imu_wifi_node')

        self.esp32_url_L = 'http://192.168.1.121/imu_L'  
        self.esp32_url_F1 = 'http://192.168.1.120/imu_F1'  
        
        self.pub_imu_L = self.create_publisher(Imu, 'imu/data_raw/L', 10)
        self.pub_imu_F1 = self.create_publisher(Imu, 'imu/data_raw/F1', 10)

        #self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.create_timer(1/40, self.read_data)  # 40 Hz

    def read_data(self):
        try:
            response_L = requests.get(self.esp32_url_L, timeout=0.5)
            response_F1 = requests.get(self.esp32_url_F1, timeout=0.5)

            if response_L.status_code == 200 and response_F1.status_code == 200:
                self.parse_json_L(response_L.json())
                self.parse_json_F1(response_F1.json())

            else:
                self.get_logger().warn(f"HTTP error code for leader: {response_L.status_code}")
                self.get_logger().warn(f"HTTP error code for follower: {response_F1.status_code}")

        except Exception as e:
            self.get_logger().warn(f"HTTP request failed: {e}")

    def parse_json_L(self, data):
        try:
            ax_L = float(data["ax_L"])
            ay_L = float(data["ay_L"])
            az_L = float(data["az_L"])
            gx_L = float(data["gx_L"])
            gy_L = float(data["gy_L"])
            gz_L = float(data["gz_L"])

            imu_msg_L = Imu()
            imu_msg_L.linear_acceleration.x = ax_L * 9.81
            imu_msg_L.linear_acceleration.y = ay_L * 9.81
            imu_msg_L.linear_acceleration.z = az_L * 9.81
            imu_msg_L.angular_velocity.x = math.radians(gx_L)
            imu_msg_L.angular_velocity.y = math.radians(gy_L)
            imu_msg_L.angular_velocity.z = math.radians(gz_L)
            imu_msg_L.header.stamp = self.get_clock().now().to_msg()
            imu_msg_L.header.frame_id = 'base_link_L'
            self.pub_imu_L.publish(imu_msg_L)
        
        except Exception as e:
            self.get_logger().warn(f"JSON parsing error: {e}")


    def parse_json_F1(self, data):
        try:
            ax_F1 = float(data["ax_F1"])
            ay_F1 = float(data["ay_F1"])
            az_F1 = -float(data["az_F1"])
            gx_F1 = float(data["gx_F1"])
            gy_F1 = float(data["gy_F1"])
            gz_F1 = float(data["gz_F1"])

            imu_msg_F1 = Imu()
            imu_msg_F1.linear_acceleration.x = ax_F1 * 9.81
            imu_msg_F1.linear_acceleration.y = ay_F1 * 9.81
            imu_msg_F1.linear_acceleration.z = az_F1 * 9.81
            imu_msg_F1.angular_velocity.x = math.radians(gx_F1)
            imu_msg_F1.angular_velocity.y = math.radians(gy_F1)
            imu_msg_F1.angular_velocity.z = math.radians(gz_F1)
            imu_msg_F1.header.stamp = self.get_clock().now().to_msg()
            imu_msg_F1.header.frame_id = 'base_link_F1'
            self.pub_imu_F1.publish(imu_msg_F1)
        except Exception as e:
            self.get_logger().warn(f"JSON parsing error: {e}")

def main():
    rclpy.init()
    node = IMUWIFINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
