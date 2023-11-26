# Este archivo debe llamarse my_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-paused', '-world', get_package_share_directory('utrafman') + '/worlds/tutos/tuto1.world']
        ),
    ])
