import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # MAVROS Node for ArduPilot
        launch_ros.actions.Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14550@',  # Adjust if needed
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
            }]
        ),

        # Custom Nodes from Magnetron
        launch_ros.actions.Node(
            package='magnetron',
            executable='stepper_node',
            name='stepper_node',
            output='screen'
        )
    ])
