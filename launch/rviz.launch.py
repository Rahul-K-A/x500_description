from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros import parameter_descriptions

def generate_launch_description():

    launch_description = LaunchDescription()

    x500_urdf_file = PathJoinSubstitution(
                                                [
                                                    FindPackageShare('x500_description'),
                                                    "urdf",
                                                    "x500_depth.urdf.xacro"
                                                ]
                                            )

    x500_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            x500_urdf_file,
        ]
    )


    rviz_config_file = PathJoinSubstitution(
                                                [
                                                    FindPackageShare('x500_description'),
                                                    "rviz",
                                                    "conf.rviz"
                                                ]
                                            )

    robot_description = {"robot_description": parameter_descriptions.ParameterValue(x500_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     output="both",
    #     parameters=[robot_description],
    # )

    # Launch Rviz
    start_rviz_cmd = Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_file]
                    )

    launch_description.add_action(robot_state_publisher_node)
    # launch_description.add_action(joint_state_publisher_node)
    launch_description.add_action(start_rviz_cmd)


    return launch_description