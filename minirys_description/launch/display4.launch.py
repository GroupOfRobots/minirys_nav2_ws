import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    namespace=LaunchConfiguration('namespace')
    # arg_namespace = actions.DeclareLaunchArgument('namespace', default_value='minirys')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    pkg_share = launch_ros.substitutions.FindPackageShare(package='minirys_description').find('minirys_description')
    default_model_path = os.path.join(pkg_share, 'src/description/minirys_description4.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        namespace=namespace,
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # namespace=namespace,
        remappings=remappings,
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        # namespace='minirys4',
    )

    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                             description='Flag to enable use_sim_time'),
        # launch.actions.DeclareLaunchArgument('namespace', default_value=namespace, description='Top-level namespace'),
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
        #                                      description='Flag to enable joint_state_publisher_gui'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        # joint_state_publisher_gui_node,
    ])

