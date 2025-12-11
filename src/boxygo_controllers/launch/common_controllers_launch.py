from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='real',
        description='Mode of operation: real, gazebo, or isaac'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    start_control_node_arg = DeclareLaunchArgument(
        'start_control_node',
        default_value='true',
        description='Start the standalone ros2_control_node'
    )

    sim_mode = LaunchConfiguration('sim_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_control_node = LaunchConfiguration('start_control_node')

    
    pkg_boxygo_description = FindPackageShare("boxygo_description")
    pkg_boxygo_controllers = FindPackageShare("boxygo_controllers")

    urdf_path = PathJoinSubstitution([pkg_boxygo_description, "urdf", "luksusowy.urdf.xacro"])
    controller_config = PathJoinSubstitution([pkg_boxygo_controllers, "config", "diff_drive_controller.yaml"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), " ",
            urdf_path, " ",
            "sim_mode:=", sim_mode
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {'use_sim_time': use_sim_time}],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
        condition=IfCondition(start_control_node)
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    delayed_spawners = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            robot_controller_spawner
        ]
    )

    return LaunchDescription([
        sim_mode_arg,
        use_sim_time_arg,
        start_control_node_arg,
        robot_state_pub_node,
        control_node,
        delayed_spawners,
    ])