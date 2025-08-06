# imu_mag_wifi_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import math
from geometry_msgs.msg import Quaternion
import socket
import requests

class IMUWIFINode(Node):
    def __init__(self):
        super().__init__('imu_wifi_node')
        self.esp32_url = 'http://192.168.4.1/imu'  # Replace with esp32 route
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        #self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.create_timer(0.01, self.read_data)  # 100 Hz

    def read_data(self):
        try:
            response = requests.get(self.esp32_url, timeout=0.5)
            if response.status_code == 200:
                self.parse_json(response.json())
            else:
                self.get_logger().warn(f"HTTP error code: {response.status_code}")
        except Exception as e:
            self.get_logger().warn(f"HTTP request failed: {e}")

    def parse_json(self, data):
        try:
            ax = float(data["ax"])
            ay = float(data["ay"])
            az = float(data["az"])
            gx = float(data["gx"])
            gy = float(data["gy"])
            gz = float(data["gz"])
            #mx = float(data["mx"])
            #my = float(data["my"])
            #mz = float(data["mz"])
            # === SIMPLE PRINT TO TERMINAL ===



            #print("-" * 40)
            #print(f"Accelerometer (m/s²): ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}")
            #print(f"Gyroscope (°/s)     : gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}")
            
            
            
            #print(f"Magnetometer (uT)   : mx={mx:.6f}, my={my:.6f}, mz={mz:.6f}")
            # Correct orientation calculation from magnetometer
            #or_mag = math.atan2(my, mx) * 180 / math.pi
            #if or_mag < 0:
             #   or_mag += 360  # Convert to 0–360 degrees range

            #print(f"Orientation from mag: {or_mag:.2f}°")


            imu_msg = Imu()
            imu_msg.linear_acceleration.x = ax * 9.81
            imu_msg.linear_acceleration.y = ay * 9.81
            imu_msg.linear_acceleration.z = az * 9.81
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'
            self.pub_imu.publish(imu_msg)

            #mag_msg = MagneticField()
            #mag_msg.magnetic_field.x = mx 
            #mag_msg.magnetic_field.y = my 
            #mag_msg.magnetic_field.z = mz 
            #mag_msg.header = imu_msg.header
            #self.pub_mag.publish(mag_msg)
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
