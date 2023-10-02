from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    # General arguments
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")

    # Configuration
    roboteq_configuration = PathJoinSubstitution([FindPackageShare(config_package), "config/", config_file.perform(context)])

    # Driver node
    driver_node = Node(
        package="roboteq_motor_controller_driver",
        executable="roboteq_motor_controller_driver",
        output="screen",
        parameters=[roboteq_configuration],
    )

    # Odometry node
    odom_node = Node(
        package="roboteq_motor_controller_driver",
        executable="diff_odom",
        output="screen",
        parameters=[roboteq_configuration],
    )

    
    nodes_to_start = [
        driver_node,
        # odom_node
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
   
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_package",
            default_value="roboteq_motor_controller_driver",
            description="Package with configuration file"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="roboteq_driver.yaml",
            description="Configuration file",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])