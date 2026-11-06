# Copyright 2026 TagSLAM contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """Validate inputs and start RViz, a paused player, and the trajectory node."""
    bag = LaunchConfiguration('in_bag').perform(context)
    if not bag or not (Path(bag) / 'metadata.yaml').is_file():
        raise RuntimeError(f'in_bag must be a ROS bag directory: {bag!r}')
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        raise RuntimeError('a graphical session is required (DISPLAY or WAYLAND_DISPLAY)')

    def positive(name):
        raw = LaunchConfiguration(name).perform(context)
        try:
            value = float(raw)
        except ValueError as error:
            raise RuntimeError(f'{name} must be a finite positive number: {raw!r}') from error
        if not math.isfinite(value) or value <= 0:
            raise RuntimeError(f'{name} must be a finite positive number: {raw!r}')
        return value

    rate = positive('rate')
    max_gap = positive('max_gap')
    rviz_name = 'tagslam_view_rviz'
    player_name = 'tagslam_bag_player'
    config = os.path.join(get_package_share_directory('tagslam'),
                          'config', 'job_004_filtered.rviz')
    if not os.path.isfile(config):
        raise RuntimeError(f'RViz configuration not found: {config}')

    rviz = Node(package='rviz2', executable='rviz2', name=rviz_name,
                arguments=['-d', config], parameters=[{'use_sim_time': True}],
                output='screen')
    player = Node(package='rosbag2_transport', executable='player', name=player_name,
                  parameters=[{'storage.uri': bag, 'play.rate': rate,
                               'play.start_paused': True,
                               'play.clock_publish_frequency': 50.0,
                               'play.disable_keyboard_controls': True}],
                  remappings=[('/tf2', '/tf')], output='screen')
    trajectory = Node(package='tagslam', executable='trajectory_viewer',
                      name='tagslam_trajectory_viewer',
                      parameters=[{'use_sim_time': True, 'max_gap': max_gap,
                                   'rviz_node': rviz_name, 'player_node': player_name}],
                      output='screen')

    def on_rviz_exit(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError(f'RViz exited with status {event.returncode}')
        return [EmitEvent(event=Shutdown(reason='RViz closed'))]

    def on_required_exit(label):
        def handle(event, launch_context):
            if launch_context.is_shutdown:
                return []
            raise RuntimeError(f'{label} exited unexpectedly with status {event.returncode}')
        return handle

    return [
        RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=on_rviz_exit)),
        RegisterEventHandler(OnProcessExit(
            target_action=player, on_exit=on_required_exit('bag player'))),
        RegisterEventHandler(OnProcessExit(
            target_action=trajectory, on_exit=on_required_exit('trajectory viewer'))),
        rviz, player, trajectory,
    ]


def generate_launch_description():
    """View a TagSLAM output bag, with gaps shown as separate line segments."""
    return LaunchDescription([
        DeclareLaunchArgument('in_bag', description='Path to the TagSLAM result bag'),
        DeclareLaunchArgument('rate', default_value='0.5', description='Playback rate'),
        DeclareLaunchArgument('max_gap', default_value='0.03',
                              description='Maximum timestamp gap in seconds within a segment'),
        OpaqueFunction(function=launch_setup),
    ])
