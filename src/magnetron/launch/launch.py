import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch stepper_node
        launch_ros.actions.Node(
            package='magnetron',
            executable='stepper_node',
            name='stepper_node',
            output='screen'
        ),

        # Launch servo_node
        launch_ros.actions.Node(
            package='magnetron',
            executable='servo_node',
            name='servo_node',
            output='screen'
        )
    ])