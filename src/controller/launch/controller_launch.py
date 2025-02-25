import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction

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
            name=TextSubstitution(text='controller_pub_'), # + LaunchConfiguration('hand'),  # ✅ Correct concatenation
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
            name=TextSubstitution(text='imu_converter_'), # + LaunchConfiguration('hand'),  # ✅ Correct concatenation
            parameters=[{
                'hand': LaunchConfiguration('hand')
            }],
            output='screen'
        ),

        # ✅ Potentiometer Converter Node
        launch_ros.actions.Node(
            package='controller',
            executable='pot_converter',
            name=TextSubstitution(text='pot_converter_'),# + LaunchConfiguration('hand'),  # ✅ Correct concatenation
            parameters=[{
                'hand': LaunchConfiguration('hand')
            }],
            output='screen'
        )
    ])
