import os
import subprocess
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def check_and_launch_drone_movement(context, *args, **kwargs):
    """Check if 'drone_movement' is running and launch it if not found."""
    try:
        # ✅ Run `ros2 node list` to check if 'drone_movement' is already running
        result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
        running_nodes = result.stdout.splitlines()

        if '/drone_movement' in running_nodes:
            print("✅ 'drone_movement' is already running, skipping launch.")
            return []  # Don't launch another instance
        else:
            print("🚀 'drone_movement' is not running, launching now.")
            return [
                launch_ros.actions.Node(
                    package='controller',
                    executable='drone_movement',
                    name='drone_movement',
                    output='screen',
                    parameters=[{
                        'some_param': 'value'  # Replace with actual parameters if needed
                    }]
                )
            ]

    except Exception as e:
        print(f"⚠️ Error checking running nodes: {e}")
        return []  # Fallback to not launching if an error occurs

def generate_launch_description():
    # ✅ Declare launch arguments correctly
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

    hand_arg = DeclareLaunchArgument(
        'hand',
        default_value='right',
        description='Which hand the controller is on'
    )

    return launch.LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        hand_arg,

        # ✅ Controller Publisher Node
        launch_ros.actions.Node(
            package='controller',
            executable='controller_pub',
            name='controller_pub_',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'hand': LaunchConfiguration('hand')
            }],
            output='screen'
        ),

        # ✅ IMU Converter Node
        launch_ros.actions.Node(
            package='controller',
            executable='imu_converter',
            name='imu_converter_',
            parameters=[{
                'hand': LaunchConfiguration('hand')
            }],
            output='screen'
        ),

        # ✅ Potentiometer Converter Node
        launch_ros.actions.Node(
            package='controller',
            executable='pot_converter',
            name='pot_converter_',
            parameters=[{
                'hand': LaunchConfiguration('hand')
            }],
            output='screen'
        ),

        # ✅ Check and launch drone_movement only if it's not running
        OpaqueFunction(function=check_and_launch_drone_movement)
    ])
