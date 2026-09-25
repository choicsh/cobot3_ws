"""병원 정류장 경로 테스트용 Nav2 launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    carter_share = get_package_share_directory("carter_navigation")
    nav2_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    default_map_file = os.path.join(
        carter_share, "maps", "hospital_integration_human.yaml"
    )
    default_params_file = os.path.join(
        carter_share, "params", "hospital_navigation_params.yaml"
    )
    rviz_config = os.path.join(
        carter_share, "rviz2", "carter_navigation.rviz"
    )

    # 800 KB 라이다 PointCloud2 가 기본 UDP 수신 버퍼(208 KB)에서 유실되지 않게 한다.
    dds_profile = os.path.join(carter_share, "params", "fastdds_udp_4mb.xml")

    return LaunchDescription(
        [
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", dds_profile),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_file,
                description="병원 맵 YAML",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Nav2 파라미터 파일",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="True"
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="True",
                description="RViz 실행 여부 (선택적으로 use_rviz:=False)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, "rviz_launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                launch_arguments={
                    "namespace": "",
                    "use_namespace": "False",
                    "use_sim_time": use_sim_time,
                    "rviz_config": rviz_config,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            # 병원 경로는 직접 만든 Path를 FollowPath로 실행하므로 lane mask
            # server/costmap filter/lifecycle manager를 띄우지 않는다.
            # Isaac start_pose.json seeds AMCL once; AMCL alone owns map -> odom.
            Node(
                package="nav_to_goal",
                executable="hospital_amcl_initial_pose",
                name="hospital_amcl_initial_pose",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", "/front_3d_lidar/lidar_points"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "target_frame": "front_3d_lidar",
                        "transform_tolerance": 0.01,
                        "min_height": -0.8,
                        "max_height": 3.0,
                        "angle_min": -3.1416,
                        "angle_max": 3.1416,
                        "angle_increment": 0.0087,
                        "scan_time": 0.3333,
                        "range_min": 0.05,
                        "range_max": 100.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav_to_goal",
                executable="hospital_scan_self_filter",
                name="hospital_scan_self_filter",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "body_bounds": [-1.38, 0.48, -0.5, 0.5],
                }],
            ),
            Node(
                package="nav_to_goal",
                executable="hospital_moving_obstacle_predictor",
                name="hospital_moving_obstacle_predictor",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # cmd_vel_smoothed -> guard -> cmd_vel_human_checked -> collision_monitor.
            # guard가 없으면 collision_monitor 입력이 끊겨 로봇이 움직이지 않는다.
            Node(
                package="nav_to_goal",
                executable="hospital_velocity_guard",
                name="hospital_velocity_guard",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
