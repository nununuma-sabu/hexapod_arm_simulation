import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('hexapod_arm_bot_gazebo')
    pkg_description = get_package_share_directory('hexapod_arm_bot_description')
    pkg_moveit = get_package_share_directory('hexapod_arm_bot_moveit_config')

    xacro_file = os.path.join(pkg_description, 'urdf', 'hexapod.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    srdf_file = os.path.join(pkg_moveit, 'config', 'hexapod_arm_bot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    kinematics_yaml = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    ompl_yaml = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')
    import yaml
    with open(kinematics_yaml, 'r') as f:
        kinematics_config = yaml.safe_load(f)
    with open(ompl_yaml, 'r') as f:
        ompl_config = yaml.safe_load(f)

    # MoveIt move_group node (IK / Motion Planning)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'robot_description_kinematics': kinematics_config},
            {'move_group': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                'start_state_max_bounds_error': 0.1,
            }},
            {'robot_description_planning': ompl_config},
            {'use_sim_time': True},
        ],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Ignition Gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': '-r empty.sdf'}.items(),
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'hexapod_arm_bot',
                   '-allow_renaming', 'true'],
    )

    # ROS-Ignition Bridge for clock
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Joint State Broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Arm Trajectory Controller
    trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        move_group_node,
        ign_gazebo,
        bridge,
        spawn_entity,
        jsb_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[trajectory_spawner],
            )
        )
    ])
