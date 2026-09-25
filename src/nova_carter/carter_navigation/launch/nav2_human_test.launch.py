# 사람 회피 실험용 단일 로봇 Nav2 스택.
#
#   ros2 launch carter_navigation nav2_human_test.launch.py
#
# 전제: Isaac 에서 isaacpjt/pjt_alpha/run_human_scene.py 로 integration_human.usd 가 떠 있을 것.
#
# carter_navigation.launch.py 에서 이 실험에 불필요하거나 방해가 되는 것을 뺐다.
#
#   - ground_truth_localization  : params 의 amcl 이 tf_broadcast: true 라 AMCL 이 map->odom 을 낸다.
#                                  정적 TF 를 같이 띄우면 같은 변환을 두 노드가 발행해 충돌한다.
#                                  (carter_navigation.launch.py 의 "tf_broadcast: false" 주석은
#                                   현재 params 와 맞지 않는다. 실제 값은 true.)
#   - filter_mask_server         : global_costmap 에 filters 키가 없어 keepout 필터가 물려 있지 않다.
#     costmap_filter_info_server   즉 lane_mask 는 아무도 구독하지 않는 토픽을 내고 있었다.
#     lifecycle_manager_costmap_filters
#
# 남긴 것은 세 개뿐이다.
#   1. nav2 bringup        map_server + AMCL + planner/controller/bt_navigator/behaviors/
#                          velocity_smoother/collision_monitor/lifecycle
#   2. pointcloud_to_laserscan   /front_3d_lidar/lidar_points -> /scan
#                          필수다. global_costmap 의 obstacle_layer 와 collision_monitor 가 /scan 을 본다.
#                          이게 없으면 사람이 costmap 에 전혀 안 찍힌다.
#   3. rviz

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    carter_dir = get_package_share_directory("carter_navigation")
    nav2_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            # 이력: intergration_nova.yaml -> integration_hospital.yaml
            #       -> hospital_integration_human.yaml (2026-09-23, 씬에서 새로 뽑은 맵)
            # 씬을 hospital_integration_human.usd 로 바꾸면서 같이 옮겼다.
            # ⚠ params 의 amcl initial_pose 와 through_pose_human_test.py 의 WAYPOINTS 는
            #   아직 옛 맵 좌표라 무효다. 새 맵 좌표로 다시 정해야 한다.
            #
            # 맵 제원 (occupancy map 추출값, 두 맵 모두 같은 월드 좌표계):
            #   integration_hospital       1510x660, origin [-49.975,  -9.475]  -> X -49.98~25.53, Y -9.48~23.53
            #   hospital_integration_human 1480x635, origin [-49.275,  -8.875]  -> X -49.28~24.73, Y -8.88~22.88
            # 월드 원점이 같으므로 기존 좌표값은 새 맵 범위 안에 있으면 그대로 유효하다.
            default_value=os.path.join(carter_dir, "maps", "hospital_integration_human.yaml"),
            description="맵 yaml. hospital_integration 씬용 기본값",
        ),
        DeclareLaunchArgument(
            "params_file",
            # 이 로봇은 M0609 를 얹어 footprint 가 1.86 x 1.0 m 다.
            # params/office/multi_robot_*.yaml 은 스톡 Carter(0.747 x 0.5) 기준이라 쓰면 안 된다.
            default_value=os.path.join(
                carter_dir, "params", "carter_navigation_params_human_test.yaml"),
            description="Nav2 파라미터. carter_navigation_params.yaml 복사본 + 경유지 수정 2곳",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="True",
            description="Isaac /clock 을 쓴다. False 로 두면 barrier/타임아웃이 전부 어긋난다",
        ),
        DeclareLaunchArgument("use_rviz", default_value="True"),

        # AMCL 은 params 의 set_initial_pose/initial_pose (1.233, 3.363, yaw 180deg) 로
        # 스스로 초기화한다. 별도로 2D Pose Estimate 를 찍을 필요 없다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
            launch_arguments={
                "map": map_yaml,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
            }.items(),
        ),

        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[("cloud_in", "/front_3d_lidar/lidar_points"), ("scan", "/scan")],
            parameters=[{
                "target_frame": "front_3d_lidar",
                "transform_tolerance": 0.01,
                "min_height": -0.8,    # 로봇 2배 스케일: 라이다 장착 높이도 2배
                "max_height": 2.0,     # 3.0 -> 2.0 (사람 키 수준 제한, 상공 연산 배제)
                # XT-32 는 원래 360도 회전형인데 여기서 +-90도로 잘라내고 있었다.
                # 그러면 로봇이 지나온 자리가 뒤로 빠지는 순간 관측이 끊기고, raytrace clearing 은
                # "지금 광선이 통과한 셀"만 지우므로 사람이 떠난 자리가 영구 장애물로 남는다.
                # (로컬 costmap 은 PointCloud2 를 직접 받아 이미 360도다. 잘려 있던 건 /scan 을 쓰는
                #  global_costmap 뿐이고, 전역은 롤링 윈도우도 없어 잔상이 훨씬 오래 남았다.)
                # 정후방은 자기 차체가 광선을 막아 여전히 사각이며 그건 후방 라이다가 담당한다.
                # 차체 자기 반사는 footprint_clearing_enabled 가 지운다.
                "angle_min": -3.14159,  # -M_PI
                "angle_max": 3.14159,   #  M_PI
                # 0.0087(722빔) -> 0.0174(약 360빔, 1도 분해능). CPU 변환 부하 50% 절감
                "angle_increment": 0.0174,
                "scan_time": 0.3333,
                "range_min": 0.05,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
                "use_sim_time": use_sim_time,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "rviz_launch.py")),
            condition=IfCondition(use_rviz),
            launch_arguments={
                "namespace": "",
                "use_namespace": "False",
                "rviz_config": os.path.join(carter_dir, "rviz2", "carter_navigation.rviz"),
            }.items(),
        ),
    ])
