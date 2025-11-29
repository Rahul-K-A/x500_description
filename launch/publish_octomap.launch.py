from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    octomap_path_ = PathJoinSubstitution([FindPackageShare('x500_description'), 'rtab_map_files', 'mapfile.bt'])
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'octomap_path': octomap_path_
            }],
        )
    ])