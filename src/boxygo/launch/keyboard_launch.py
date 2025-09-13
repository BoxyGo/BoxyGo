from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def _setup(context, *args, **kwargs):
    topic = LaunchConfiguration('cmd_vel_topic').perform(context)
    cmd = [
        'gnome-terminal', '--', 'bash', '-lc',
        f'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:={topic}; exec bash'
    ]
    return [ExecuteProcess(cmd=cmd, output='screen')]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/diff_cont/cmd_vel_unstamped',
            description='Docelowy topic Twist'
        ),
        OpaqueFunction(function=_setup),
    ])