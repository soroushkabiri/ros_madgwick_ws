
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your serial reader
        Node(
            package='mpu_mag_fusion',
            executable='orientation_publisher',
            name='imu_serial_node',
            output='screen'
        ),
        # Madgwick filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',  # or 'nwu' if you prefer
                'magnetic_declination_radians': 0.0,
                'reverse_mag_y': False,
                'reverse_mag_z': False
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/mag', 'imu/mag'),
                ('imu/data', 'imu/data')  # final orientation
            ]
        ),
        # magdwich to yaw
        Node(
            package='mpu_mag_fusion',
            executable='madgwick_to_yaw',
            name='yaw_extractor',
            output='screen'
        ),
           
        
    ])
