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

import csv
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
import yaml


MODULE_PATH = Path(__file__).resolve().parents[1] / 'src/python/estimate_from_mcap.py'
spec = importlib.util.spec_from_file_location('estimate_from_mcap', MODULE_PATH)
pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pipeline)


def calibration():
    camera = {'resolution': [8, 6], 'intrinsics': [10, 11, 3, 2],
              'distortion_model': 'fisheye', 'distortion_coeffs': [0, 0, 0, 0]}
    return {'cam0': dict(camera), 'cam1': dict(camera),
            'T_cam1_cam0': [[0, -1, 0, -100], [1, 0, 0, 20],
                            [0, 0, 1, 30], [0, 0, 0, 1]]}


def set_stamp(message, stamp):
    message.header.stamp.sec = stamp // 1_000_000_000
    message.header.stamp.nanosec = stamp % 1_000_000_000


def make_source(path, stamps, invalid_image=False, wrong_type=False):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                pipeline.converter_options())
    typename = 'sensor_msgs/msg/Image' if wrong_type else 'sensor_msgs/msg/CompressedImage'
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/stereo/image_raw', type=typename, serialization_format='cdr'))
    for index, stamp in enumerate(stamps):
        image = np.zeros((6, 16), dtype=np.uint8)
        image[:, :8] = 30 + index
        image[:, 8:] = 200 - index
        ok, data = cv2.imencode('.png', image)
        assert ok
        message = CompressedImage()
        set_stamp(message, stamp)
        message.header.frame_id = 'stereo'
        message.format = 'png'
        message.data = b'bad image' if invalid_image else data.tobytes()
        writer.write('/stereo/image_raw', serialize_message(message), 10_000_000_000 + index * 100)
    writer.close()


def args_for(tmp_path, stamps, invalid_image=False, wrong_type=False):
    source = tmp_path / 'raw'
    make_source(source, stamps, invalid_image, wrong_type)
    calib = tmp_path / 'calib.yaml'
    calib.write_text(yaml.safe_dump(calibration()), encoding='utf-8')
    template = tmp_path / 'tagslam.yaml'
    template.write_text('body_defaults: {}\n', encoding='utf-8')
    output = tmp_path / 'result'
    output.mkdir()
    return SimpleNamespace(in_bag=str(source), calib=str(calib), output_dir=str(output),
                           image_topic='/stereo/image_raw', template=str(template),
                           minimum_tags_for_pose=3, keep_intermediate=True)


def make_pose_bag(path, stamped_poses):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                pipeline.converter_options())
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name=pipeline.POSE_TOPIC, type='nav_msgs/msg/Odometry', serialization_format='cdr'))
    for stamp, value in stamped_poses:
        message = Odometry()
        set_stamp(message, stamp)
        message.header.frame_id = 'map'
        message.child_frame_id = 'rig'
        message.pose.pose.position.x = value
        message.pose.pose.position.y = -0.125
        message.pose.pose.position.z = 1.0
        message.pose.pose.orientation.w = 1.0
        writer.write(pipeline.POSE_TOPIC, serialize_message(message), stamp)
    writer.close()


def test_calibration_conversion():
    cameras, poses, size = pipeline.camera_configuration(calibration())
    assert size == (8, 6)
    assert cameras['cam1']['image_topic'] == '/cam1/image_raw'
    assert poses['cam0']['pose']['position'] == {'x': 0.0, 'y': 0.0, 'z': 0.0}
    cam1 = poses['cam1']['pose']
    assert list(cam1['position'].values()) == pytest.approx([-0.02, -0.1, -0.03])
    assert list(cam1['rotation'].values()) == pytest.approx([0, 0, -np.pi / 2])
    assert cam1['position_noise'] == {'x': 1e-7, 'y': 1e-7, 'z': 1e-7}
    bad = calibration()
    bad['cam1']['resolution'] = [8, 7]
    with pytest.raises(ValueError, match='resolutions'):
        pipeline.camera_configuration(bad)
    bad = calibration()
    bad['cam0']['intrinsics'] = [0, 11, 3, 2]
    with pytest.raises(ValueError, match='positive focal lengths'):
        pipeline.camera_configuration(bad)
    bad = calibration()
    bad['cam1']['intrinsics'] = [10, -11, 3, 2]
    with pytest.raises(ValueError, match='positive focal lengths'):
        pipeline.camera_configuration(bad)
    bad = calibration()
    bad['T_cam1_cam0'][0][0] = 7
    with pytest.raises(ValueError, match='rigid'):
        pipeline.camera_configuration(bad)


def test_split_and_frame_aligned_export(tmp_path):
    stamps = [1_700_000_000_123_456_789 + x for x in (0, 20_000_001, 40_000_002)]
    args = args_for(tmp_path, stamps)
    pipeline.prepare(args)
    output = Path(args.output_dir)
    assert pipeline.read_frames(output / 'frame_index.csv') == [
        (i, stamp, 10_000_000_000 + i * 100) for i, stamp in enumerate(stamps)]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(output / 'stereo_bag')),
                pipeline.converter_options())
    images = []
    topics = []
    while reader.has_next():
        topic, raw, bag_stamp = reader.read_next()
        topics.append(topic)
        if topic != pipeline.END_TOPIC:
            message = deserialize_message(raw, Image)
            images.append((topic, pipeline.timestamp_ns(message.header), bag_stamp,
                           np.frombuffer(message.data, dtype=np.uint8).reshape(6, 8).copy()))
    reader.close()
    assert len(images) == 6 and topics[-1] == pipeline.END_TOPIC
    for i in range(3):
        assert images[2 * i][:3] == ('/cam0/image_raw', stamps[i], 10_000_000_000 + i * 100)
        assert images[2 * i + 1][:3] == ('/cam1/image_raw', stamps[i], 10_000_000_000 + i * 100)
        assert np.all(images[2 * i][3] == 30 + i)
        assert np.all(images[2 * i + 1][3] == 200 - i)
    precise = 0.12345678901234566
    make_pose_bag(output / 'out_bag', [(stamps[0], precise), (stamps[2], -precise)])
    pipeline.export(SimpleNamespace(output_dir=str(output)))
    with (output / 'trajectory.csv').open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 3
    assert list(rows[0]) == list(pipeline.CSV_FIELDS)
    assert [int(row['timestamp_ns']) for row in rows] == stamps
    assert [row['relative_time_s'] for row in rows] == ['0.000000000', '0.020000001', '0.040000002']
    assert [row['pose_valid'] for row in rows] == ['1', '0', '1']
    assert float(rows[0]['x']) == precise and float(rows[2]['x']) == -precise
    assert [rows[1][key] for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')] == [''] * 7
    assert all(row['frame_id'] == 'map' and row['child_frame_id'] == 'rig' for row in rows)
    summary = json.loads((output / 'run_summary.json').read_text())
    assert (summary['frame_count'], summary['valid_pose_count'], summary['missing_pose_count']) == (3, 2, 1)
    assert (output / 'stereo_bag').exists()


def test_all_missing_and_cleanup(tmp_path):
    args = args_for(tmp_path, [1_700_000_000_000_000_001])
    args.keep_intermediate = False
    pipeline.prepare(args)
    output = Path(args.output_dir)
    make_pose_bag(output / 'out_bag', [])
    pipeline.export(SimpleNamespace(output_dir=str(output)))
    with (output / 'trajectory.csv').open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1 and rows[0]['pose_valid'] == '0' and rows[0]['x'] == ''
    assert not (output / 'stereo_bag').exists()


@pytest.mark.parametrize('stamps,error', [
    ([1_000_000_000, 1_000_000_000], 'not strictly increasing'),
    ([1_000_000_001, 1_000_000_000], 'not strictly increasing'),
])
def test_invalid_image_timestamps_fail(tmp_path, stamps, error):
    args = args_for(tmp_path, stamps)
    with pytest.raises(ValueError, match=error):
        pipeline.prepare(args)
    assert not (Path(args.output_dir) / 'run_parameters.json').exists()


def test_bad_image_and_topic_type(tmp_path):
    args = args_for(tmp_path, [1_000_000_000], invalid_image=True)
    with pytest.raises(ValueError, match='invalid stereo image'):
        pipeline.prepare(args)
    other = tmp_path / 'other'
    other.mkdir()
    args = args_for(other, [1_000_000_000], wrong_type=True)
    with pytest.raises(ValueError, match='not a CompressedImage'):
        pipeline.prepare(args)


@pytest.mark.parametrize('pose_stamps,error', [
    ([1_000_000_000, 1_000_000_000], 'duplicate pose'),
    ([1_000_000_001], 'does not match'),
])
def test_ambiguous_or_unknown_pose_fails_without_csv(tmp_path, pose_stamps, error):
    args = args_for(tmp_path, [1_000_000_000])
    pipeline.prepare(args)
    output = Path(args.output_dir)
    make_pose_bag(output / 'out_bag', [(stamp, 0.25) for stamp in pose_stamps])
    with pytest.raises(ValueError, match=error):
        pipeline.export(SimpleNamespace(output_dir=str(output)))
    assert not (output / 'trajectory.csv').exists()
