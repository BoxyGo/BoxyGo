from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Ścieżki do plików
    pkg_boxygo_description = get_package_share_directory('boxygo_description')
    pkg_boxygo_controllers = get_package_share_directory('boxygo_controllers')

    # ZMIANA: Wskazujemy na główny plik (zwrotnicę), a nie eksperymentalny
    # Upewnij się, że nazwa pliku to 'luksusowy.urdf.xacro' lub 'robot.urdf.xacro' (zależnie jak go nazwałeś)
    urdf_file_name = 'luksusowy.urdf.xacro' 
    urdf_path = os.path.join(pkg_boxygo_description, 'urdf', urdf_file_name)
    
    # Plik konfiguracyjny kontrolerów
    controller_config = os.path.join(pkg_boxygo_controllers, 'config', 'diff_drive_controller.yaml')

    # ZMIANA: Argument use_sim_time (dla Isaaca zazwyczaj True)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ZMIANA: Komenda Xacro z argumentem sim_mode:=isaac
    # To tutaj decydujemy, że ładujemy 'config/isaac_control.xacro'
    robot_description_content = Command([
        'xacro ', urdf_path,
        ' sim_mode:=isaac'
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # 2. Definicja Węzłów

    # A. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # B. ROS2 Control Node (MÓZG)
    # W Isaac Sim ten węzeł jest niezbędny, bo pełni rolę sterownika
    # (w Gazebo robi to plugin wewnątrz symulatora, tu robimy to osobnym węzłem)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # C. Spawner: Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # D. Spawner: Diff Drive Controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Sekwencja uruchamiania
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Isaac) clock if true'),
            
        robot_state_publisher_node,
        control_node,
        
        # Czekamy chwilę, aż ros2_control_node załaduje hardware_interface (TopicBasedSystem)
        TimerAction(
            period=3.0,
            actions=[
                joint_state_broadcaster_spawner,
                diff_drive_spawner
            ]
        )
    ])