import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cfg = (
        MoveItConfigsBuilder("warehouse_amr", package_name="warehouse_amr")
        .robot_description(file_path="urdf/warehouse_amr_ur5e.xacro")
        .robot_description_semantic(file_path="config/warehouse_amr.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/moveit_joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[cfg.to_dict(), {"use_sim_time": True}],
    )

    rviz_config = os.path.join(
        get_package_share_directory("ur_moveit_config"), "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config, "-f", "base_footprint"],
        parameters=[
            cfg.robot_description,
            cfg.robot_description_semantic,
            cfg.robot_description_kinematics,
            cfg.planning_pipelines,
            cfg.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group, rviz])
