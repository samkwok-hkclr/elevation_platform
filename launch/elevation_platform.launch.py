import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    NUM_OF_JOINT_MOTOR = 4

    ld = LaunchDescription()

    pkg_name = "elevation_platform"

    bringup_dir = get_package_share_directory(pkg_name)
    launch_dir = os.path.join(bringup_dir, "launch")

    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "joint_motor_config.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)

    for i in range(0, int(NUM_OF_JOINT_MOTOR)):
        node = Node(
            package=pkg_name,
            namespace=f"elevation_joint_motor_{i + 1}",
            executable="joint_motor_driver_node",
            # name="joint_motor_driver_node",
            parameters=[
                params_file,
            ],
            respawn=use_respawn,
            respawn_delay=3.0,
            output="screen",
        )
        ld.add_action(node)

    elev_plf_node = Node(
        package=pkg_name,
        executable="elevation_platform_node",
        parameters=[
            params_file,
        ],
        respawn=use_respawn,
        respawn_delay=3.0,
        output="screen",
    )

    ld.add_action(elev_plf_node)

    return ld
