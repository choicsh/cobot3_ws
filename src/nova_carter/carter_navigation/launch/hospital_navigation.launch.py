"""병원 정류장 경로 테스트용 Nav2 launch.

로봇 1대(기본): 토픽·노드가 전역 이름 (/scan, /cmd_vel, /tf ...).
다중 로봇(P7): namespace:=robotN — Nav2 와 병원 노드를 /robotN 아래에 띄우고 /tf 를 robotN/tf 로 잇는다
(Isaac run_fleet_sim 이 로봇마다 /robotN/tf, /robotN/front_3d_lidar/lidar_points 를 낸다. frame id 는 그대로).
파라미터 파일의 <robot_namespace> 는 "" 또는 /robotN 으로 바꾼다 — 코스트맵 레이어 토픽처럼 상대 이름이
로봇 네임스페이스로 풀리지 않는 곳에 쓴다. RViz 설정의 토픽도 /robotN 아래로 바꿔 띄운다.
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def _namespaced_copy(path, prefix, kind):
    """<robot_namespace> 를 prefix 로 바꾼 임시 파일. RViz 설정은 절대 토픽 앞에 prefix 를 붙인다."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    if kind == "params":
        text = text.replace("<robot_namespace>", prefix)
    elif prefix:
        text = re.sub(r"(\n\s+Value: )/(?=[A-Za-z])", rf"\1{prefix}/", text)
    fd, out = tempfile.mkstemp(prefix=f"hospital{prefix.replace('/', '_')}_", suffix=os.path.splitext(path)[1])
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)
    return out


def _nodes(context):
    carter_share = get_package_share_directory("carter_navigation")
    nav2_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ns = LaunchConfiguration("namespace").perform(context).strip("/")
    prefix = f"/{ns}" if ns else ""
    remaps = TF_REMAPS if ns else []
    params_file = _namespaced_copy(LaunchConfiguration("params_file").perform(context), prefix, "params")
    rviz_config = _namespaced_copy(
        os.path.join(carter_share, "rviz2", "carter_navigation.rviz"), prefix, "rviz")

    actions = []
    if LaunchConfiguration("use_rviz").perform(context).lower() == "true":
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=ns,
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=remaps,
            output="screen",
        ))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": ns,
            "use_namespace": str(bool(ns)),
            "map": LaunchConfiguration("map"),
            "params_file": params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    ))
    # 병원 경로는 직접 만든 Path를 FollowPath로 실행하므로 lane mask
    # server/costmap filter/lifecycle manager를 띄우지 않는다.
    # Isaac start_pose.json seeds AMCL once; AMCL alone owns map -> odom.
    actions += [
        Node(
            package="nav_to_goal",
            executable="hospital_amcl_initial_pose",
            name="hospital_amcl_initial_pose",
            namespace=ns,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time,
                         "start_pose_path": LaunchConfiguration("start_pose_path")}],
            remappings=remaps,
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            namespace=ns,
            remappings=remaps + [
                ("cloud_in", "front_3d_lidar/lidar_points"),
                ("scan", "scan"),
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
            namespace=ns,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "body_bounds": [-1.38, 0.48, -0.5, 0.5],
            }],
            remappings=remaps,
        ),
        Node(
            package="nav_to_goal",
            executable="hospital_moving_obstacle_predictor",
            name="hospital_moving_obstacle_predictor",
            namespace=ns,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=remaps,
        ),
        # cmd_vel_smoothed -> guard -> cmd_vel_human_checked -> collision_monitor.
        # guard가 없으면 collision_monitor 입력이 끊겨 로봇이 움직이지 않는다.
        Node(
            package="nav_to_goal",
            executable="hospital_velocity_guard",
            name="hospital_velocity_guard",
            namespace=ns,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=remaps,
        ),
    ]
    return actions


def generate_launch_description():
    carter_share = get_package_share_directory("carter_navigation")
    # 800 KB 라이다 PointCloud2 가 기본 UDP 수신 버퍼(208 KB)에서 유실되지 않게 한다.
    dds_profile = os.path.join(carter_share, "params", "fastdds_udp_4mb.xml")

    return LaunchDescription(
        [
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", dds_profile),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(carter_share, "maps", "hospital_integration_human.yaml"),
                description="병원 맵 YAML",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(carter_share, "params", "hospital_navigation_params.yaml"),
                description="Nav2 파라미터 파일 (<robot_namespace> 를 네임스페이스로 바꾼다)",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="로봇 네임스페이스 (다중 로봇: robot1..3). 비우면 전역 이름",
            ),
            DeclareLaunchArgument(
                "start_pose_path",
                default_value=os.path.expanduser("~/cobot3_ws/isaacpjt/assets/start_pose.json"),
                description="AMCL 초기 위치 파일 (run_hospital_sim: start_pose.json, "
                            "run_fleet_sim: start_pose_robotN.json)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="True",
                description="RViz 실행 여부 (선택적으로 use_rviz:=False)",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
