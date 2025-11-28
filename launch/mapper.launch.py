import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    launch_robot = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            PathJoinSubstitution([
                                FindPackageShare('x500_description'),
                                'launch',
                                'rviz.launch.py'
                            ])
                        ])
                    )

    # ROS-GZ bridges
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
        output='screen'
    )

    image_bridge_camera = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera'],
        output='screen'
    )

    image_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/depth_camera'],
        output='screen'
    )

    # Include RTAB-MAP launch
    rtabmap_include = IncludeLaunchDescription( PythonLaunchDescriptionSource([
                                                        PathJoinSubstitution([
                                                            FindPackageShare('rtabmap_launch'),
                                                            'launch',
                                                            'rtabmap.launch.py'
                                                        ])
                                                    ]), 
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'rgbd': 'true',
            'rgb_topic': '/camera',
            'depth_topic': '/depth_camera',
            'camera_info_topic': '/camera_info',
            'frame_id': '/camera_link',
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        #px4_process,
        launch_robot,
        parameter_bridge,
        image_bridge_camera,
        image_bridge_depth,
        rtabmap_include
    ])