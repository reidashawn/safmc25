import launch
import launch_ros.actions
import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get MAVROS package share directory
    mavros_launch_file = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'apm.launch.py'
    )

    return launch.LaunchDescription([
        # Include MAVROS APM launch file without any additional parameters
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(mavros_launch_file),
        ),

        # Custom Nodes from Magnetron
        launch_ros.actions.Node(
            package='magnetron',
            executable='stepper_node',
            name='stepper_node',
            output='screen'
        )
    ])
