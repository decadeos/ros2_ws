from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robot')
    world_path = os.path.join(pkg_path, 'worlds', 'moving_world.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),

        Node(
            package='robot',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher',
            output='screen'
        )
    ])
