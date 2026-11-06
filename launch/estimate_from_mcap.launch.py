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

from datetime import datetime
import json
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """Validate the request and launch the three managed stages in order."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    in_bag = Path(arg('in_bag')).expanduser().resolve()
    if not in_bag.is_file() or in_bag.suffix.lower() != '.mcap':
        raise RuntimeError(f'in_bag must be an existing MCAP file: {in_bag}')
    calib_arg = arg('calib')
    calib = Path(calib_arg).expanduser().resolve() if calib_arg else in_bag.parent / 'calib.yaml'
    if not calib.is_file():
        raise RuntimeError(f'calibration file not found: {calib}')
    image_topic = arg('image_topic')
    if not image_topic or not image_topic.startswith('/'):
        raise RuntimeError('image_topic must be an absolute topic name')
    try:
        minimum_tags = int(arg('minimum_tags_for_pose'))
    except ValueError as error:
        raise RuntimeError('minimum_tags_for_pose must be a nonnegative integer') from error
    if minimum_tags < 0 or minimum_tags > 2_147_483_647:
        raise RuntimeError('minimum_tags_for_pose must be a nonnegative 32-bit integer')
    keep_arg = arg('keep_intermediate').lower()
    if keep_arg not in ('true', 'false'):
        raise RuntimeError('keep_intermediate must be true or false')
    keep_intermediate = keep_arg == 'true'
    output_arg = arg('output_dir')
    output = (Path(output_arg).expanduser().resolve() if output_arg else
              Path.cwd() / 'tagslam_runs' /
              f'{in_bag.stem}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}')
    # Reserve the directory before any stage can create or replace output data.
    output.mkdir(parents=True, exist_ok=False)
    print(f'TagSLAM output directory: {output}', flush=True)

    share = Path(get_package_share_directory('tagslam'))
    helper = Path(get_package_prefix('tagslam')) / 'lib' / 'tagslam' / 'estimate_from_mcap.py'
    template = share / 'config' / 'mcap_stereo_tagslam.yaml'
    if not helper.is_file() or not template.is_file():
        raise RuntimeError('installed MCAP processing resources are missing')
    env = {'ROS_LOG_DIR': str(output / 'ros_logs')}
    prepare_command = [sys.executable, str(helper), 'prepare', '--in-bag', str(in_bag),
                       '--calib', str(calib), '--output-dir', str(output),
                       '--image-topic', image_topic, '--template', str(template),
                       '--minimum-tags-for-pose', str(minimum_tags)]
    if keep_intermediate:
        prepare_command.append('--keep-intermediate')
    prepare = ExecuteProcess(cmd=prepare_command, cwd=str(output), additional_env=env,
                             output='screen')

    def log_events(action, name):
        def append(event):
            with (output / f'{name}.log').open('ab') as stream:
                stream.write(event.text)
        return RegisterEventHandler(OnProcessIO(target_action=action,
                                                on_stdout=append, on_stderr=append))

    def check_exit(event, launch_context, stage):
        if launch_context.is_shutdown:
            return False
        if event.returncode != 0:
            raise RuntimeError(f'{stage} failed with exit status {event.returncode}; '
                               f'diagnostics: {output}')
        return True

    def on_export(event, launch_context):
        if not check_exit(event, launch_context, 'CSV export'):
            return []
        print(f'Completed frame-aligned trajectory: {output / "trajectory.csv"}', flush=True)
        return [EmitEvent(event=Shutdown(reason='MCAP trajectory export completed'))]

    def on_slam(event, launch_context):
        if not check_exit(event, launch_context, 'TagSLAM'):
            return []
        export = ExecuteProcess(cmd=[sys.executable, str(helper), 'export',
                                     '--output-dir', str(output)], cwd=str(output),
                                additional_env=env, output='screen')
        return [log_events(export, 'export'),
                RegisterEventHandler(OnProcessExit(target_action=export, on_exit=on_export)),
                export]

    def on_prepare(event, launch_context):
        if not check_exit(event, launch_context, 'MCAP preparation'):
            return []
        metadata = json.loads((output / 'run_parameters.json').read_text(encoding='utf-8'))
        frame_count = metadata['frame_count']
        slam = Node(package='tagslam', executable='tagslam_from_bag', output='screen',
                    cwd=str(output), additional_env=env, parameters=[{
                        'cameras': str(output / 'config' / 'cameras.yaml'),
                        'camera_poses': str(output / 'config' / 'camera_poses.yaml'),
                        'tagslam_config': str(output / 'config' / 'tagslam.yaml'),
                        'in_bag': str(output / 'stereo_bag'),
                        'outbag': str(output / 'out_bag'),
                        'output_directory': str(output),
                        'max_number_of_frames': frame_count + 1,
                        'minimum_tags_for_pose': minimum_tags,
                    }])
        return [log_events(slam, 'tagslam'),
                RegisterEventHandler(OnProcessExit(target_action=slam, on_exit=on_slam)), slam]

    return [log_events(prepare, 'prepare'),
            RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=on_prepare)),
            prepare]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('in_bag', description='Raw side-by-side stereo MCAP file'),
        DeclareLaunchArgument('calib', default_value='',
                              description='Calibration YAML; default: calib.yaml next to MCAP'),
        DeclareLaunchArgument('output_dir', default_value='',
                              description='New output directory; default: ./tagslam_runs/<name>'),
        DeclareLaunchArgument('image_topic', default_value='/stereo/image_raw',
                              description='Side-by-side CompressedImage topic'),
        DeclareLaunchArgument('minimum_tags_for_pose', default_value='3',
                              description='Minimum distinct usable tags; 0 disables the limit'),
        DeclareLaunchArgument('keep_intermediate', default_value='false',
                              description='Keep the split stereo bag after success'),
        OpaqueFunction(function=launch_setup),
    ])
