from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    launch_description = LaunchDescription()



    world_sdf_file = PathJoinSubstitution(
                                                [
                                                    FindPackageShare('starling2_description'),
                                                    "worlds",
                                                    "sample_world.sdf"
                                                ]
                                            )
    
    starling_urdf_file = PathJoinSubstitution(
                                                [
                                                    FindPackageShare('starling2_description'),
                                                    "urdf",
                                                    "starling2.urdf"
                                                ]
                                            )
    
    rviz_config_file = PathJoinSubstitution(
                                                [
                                                    FindPackageShare('edumip_gazebo'),
                                                    "rviz",
                                                    "conf.rviz"
                                                ]
                                            )

    starling_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            starling_urdf_file,
        ]
    )

    # Adds our install folder to the Gazebo plugin path as well - Pretty Neat
    launch_gazebo_world = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource([
                                PathJoinSubstitution([
                                    FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'
                                ])
                            ]),
                            # -r starts the simulation automatically on launch
                            launch_arguments={
                                "gz_args": ["-v 4 ", world_sdf_file],
                            }.items()
                        )

    # run_robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[
    #                     {"robot_description": starling_description_content}
    #                ],
    # )

    # Spawn the edumip inside the Gazebo world
    spawn_edumip_cmd = Node(
                            package='ros_gz_sim',
                            executable='create',
                            output='screen',
                            arguments=['-z','0.1', '-string', starling_description_content]
                        )
    

    # # Start GZ-ROS bridge
    # start_ign_ros_bridge = Node(
    #                             package='ros_ign_bridge',
    #                             executable='parameter_bridge',
    #                             output='screen',
    #                             arguments=['/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model', 
    #                                        '/edumip_laser@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    #                                        '/model/edumip/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
    #                                        # Added these as stamped pose since RViz uses stamped pose
    #                                        '/my_tot_topic/EstimatedPose1@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
    #                                        '/my_tot_topic/EstimatedPose2@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',	
    #                                        ]
    #                         )
    # Launch Rviz
    # start_rviz_cmd = Node(
    #                 package='rviz2',
    #                 executable='rviz2',
    #                 name='rviz2',
    #                 output='screen',
    #                 arguments=['-d', rviz_config_file]
    #                 )
    

    # Note: Stuff happens in the order that you add it
    launch_description.add_action(launch_gazebo_world)
    launch_description.add_action(spawn_edumip_cmd)
    # launch_description.add_action(start_ign_ros_bridge)
    # launch_description.add_action(run_robot_state_publisher_node)
    # launch_description.add_action(start_rviz_cmd)





    return launch_description