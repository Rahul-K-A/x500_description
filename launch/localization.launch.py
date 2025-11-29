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


    start_gz_ros_bridge = Node(
                                package='ros_gz_bridge',
                                executable='parameter_bridge',
                                output='screen',
                                arguments=[
                                           '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', 
                                           '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                                           '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                                           '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                                            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image',
                                           # Added these as stamped pose since RViz uses stamped pose
                                        #    '/my_tot_topic/EstimatedPose1@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
                                        #    '/my_tot_topic/EstimatedPose2@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',	
                                           ]
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
            'localization' : 'true',
            'initial_pose' : '0 0 0 0 0 0',
            'rgbd': 'true',
            'rgb_topic': '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            'depth_topic': '/depth_camera',
            'camera_info_topic': '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info',
            'frame_id': 'base_link',
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        #px4_process,
        launch_robot,
        # parameter_bridge,
        # image_bridge_camera,
        # image_bridge_depth,
        start_gz_ros_bridge, 
        rtabmap_include
    ])