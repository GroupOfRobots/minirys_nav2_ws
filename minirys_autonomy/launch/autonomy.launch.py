import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    namespace=LaunchConfiguration('namespace', default='minirys')

    autonomy_node = launch_ros.actions.Node(
        package='minirys_autonomy',
        executable='autonomy',
        name='autonomy',
        namespace=namespace,
    )

    return launch.LaunchDescription([
        autonomy_node,
    ])
