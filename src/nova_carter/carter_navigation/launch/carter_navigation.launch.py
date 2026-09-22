# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("carter_navigation"), "maps", "intergration_nova.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("carter_navigation"), "params", "carter_navigation_params.yaml"
        ),
    )

    lane_mask_yaml = os.path.join(get_package_share_directory("carter_navigation"), "maps", "lane_mask.yaml")

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("carter_navigation"), "rviz2", "carter_navigation.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="True", description="Use simulation (Omniverse Isaac Sim) clock if True"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
            ),

            # 위치: Isaac Sim 의 odom 은 물리 엔진 실제 자세라 drift 가 없고 Play 시점이 원점이므로,
            # map->odom 은 run_nova_sim.py 가 기록한 start_pose.json 값 그대로의 정적 TF 로 충분하다.
            # AMCL 은 bringup 안에서 같이 뜨지만 tf_broadcast: false 라 TF 를 내지 않는다 (params 참고).
            # 차선 선호 마스크 (global_costmap 의 keepout_filter 가 구독). maps/make_lane_mask.py 참고.
            Node(
                package='nav2_map_server', executable='map_server',
                name='filter_mask_server', output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': lane_mask_yaml,
                             'topic_name': '/filter_mask', 'frame_id': 'map'}],
            ),
            Node(
                package='nav2_map_server', executable='costmap_filter_info_server',
                name='costmap_filter_info_server', output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'type': 0,   # 0 = keepout
                             # 코스트맵 노드는 /global_costmap 네임스페이스라 상대 토픽이면 못 찾는다 -> 절대 경로
                             'filter_info_topic': '/costmap_filter_info', 'mask_topic': '/filter_mask',
                             'base': 0.0, 'multiplier': 1.0}],
            ),
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters', output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                             'node_names': ['filter_mask_server', 'costmap_filter_info_server']}],
            ),

            # start_pose.json -> map->odom 정적 TF
            Node(
                package='nav_to_goal', executable='ground_truth_localization',
                name='ground_truth_localization', output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),

            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', ['/front_3d_lidar/lidar_points']),
                            ('scan', ['/scan'])],
                parameters=[{
                    'target_frame': 'front_3d_lidar',
                    'transform_tolerance': 0.01,
                    'min_height': -0.8,   # 로봇 2배 스케일: 라이다 장착 높이도 2배
                    'max_height': 3.0,    # 로봇 2배 스케일
                    'angle_min': -3.1416,  # -M_PI  (3D 라이다는 360deg. 옆/뒤에서 오는 사람도 costmap 에 찍히게)
                    'angle_max': 3.1416,  # M_PI
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.05,
                    'range_max': 100.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                    # 'concurrency_level': 1,
                    'use_sim_time': True,
                }],
                name='pointcloud_to_laserscan'
            )
        ]
    )
