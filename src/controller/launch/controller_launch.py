import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the controller'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    return launch.LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        
        # Controller Publisher Node
        launch_ros.actions.Node(
            package='controller',
            executable='controller_pub',
            name='controller_pub',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }],
            output='screen'
        ),

        # IMU Converter Node
        launch_ros.actions.Node(
            package='controller',
            executable='imu_converter',
            name='imu_converter',
            output='screen'
        )
    ])
