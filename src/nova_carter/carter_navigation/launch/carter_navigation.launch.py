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
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # localization:=truth (기본) -> Isaac odom 이 drift 가 없으므로 map->odom 을 start_pose.json 의
    #                              정적 TF 로 발행하고, AMCL 은 TF 를 내보내지 않게 한다 (amcl_pose 만 발행).
    # localization:=amcl          -> 실기와 같은 구성. AMCL 이 map->odom 을 추정한다.
    localization = LaunchConfiguration("localization", default="truth")
    amcl_tf_broadcast = PythonExpression(["'", localization, "' == 'amcl'"])

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


    # AMCL 의 tf_broadcast 만 localization 인자에 맞춰 덮어쓴 params 사본
    params_for_bringup = RewrittenYaml(
        source_file=param_dir,
        param_rewrites={"tf_broadcast": amcl_tf_broadcast},
        convert_types=True,
    )

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
            DeclareLaunchArgument(
                "localization", default_value="truth", choices=["truth", "amcl"],
                description="truth: Isaac ground-truth map->odom (sim only), amcl: AMCL estimates map->odom",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": params_for_bringup}.items(),
            ),

            # Isaac Sim(run_nova_sim.py)이 기록한 start_pose.json 을 AMCL 초기 위치로 넘긴다.
            # 씬에서 로봇 시작점을 옮겨도 params 의 숫자를 고칠 필요가 없다.
            Node(
                package='nav_to_goal', executable='initial_pose_from_sim',
                name='initial_pose_from_sim', output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),

            # localization:=truth 일 때만: start_pose.json 으로 map->odom 정적 TF 발행 (AMCL 은 TF 를 안 냄)
            Node(
                package='nav_to_goal', executable='ground_truth_localization',
                name='ground_truth_localization', output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=LaunchConfigurationEquals('localization', 'truth'),
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
                    'angle_min': -1.5708,  # -M_PI/2
                    'angle_max': 1.5708,  # M_PI/2
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
