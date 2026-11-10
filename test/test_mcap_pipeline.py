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

import errno
import importlib.util
import json
from pathlib import Path
import sys
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


def make_source(path, stamps, invalid_image=False, wrong_type=False, odom_stamps=()):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                pipeline.converter_options())
    typename = 'sensor_msgs/msg/Image' if wrong_type else 'sensor_msgs/msg/CompressedImage'
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/stereo/image_raw', type=typename, serialization_format='cdr'))
    if odom_stamps:
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name='/openvins/odom', type=pipeline.ODOMETRY_TYPE,
            serialization_format='cdr'))
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
    for index, stamp in enumerate(odom_stamps):
        odom = Odometry()
        set_stamp(odom, stamp)
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'imu'
        odom.pose.pose.position.x = 0.125 + index
        odom.pose.pose.orientation.w = 1.0
        writer.write('/openvins/odom', serialize_message(odom), 10_500_000_000 + index * 100)
    writer.close()


def args_for(tmp_path, stamps, invalid_image=False, wrong_type=False, odom_stamps=()):
    source = tmp_path / 'raw'
    make_source(source, stamps, invalid_image, wrong_type, odom_stamps)
    calib = tmp_path / 'calib.yaml'
    calib_data = calibration()
    if odom_stamps:
        calib_data['cam0']['T_cam_imu'] = [[0, -1, 0, 45], [1, 0, 0, 2],
                                          [0, 0, 1, -19], [0, 0, 0, 1]]
    calib.write_text(yaml.safe_dump(calib_data), encoding='utf-8')
    template = tmp_path / 'tagslam.yaml'
    template.write_text('bodies: [{rig: {odom_frame_id: rig}}]\n', encoding='utf-8')
    output = tmp_path / 'result'
    output.mkdir()
    return SimpleNamespace(in_bag=str(source), calib=str(calib), output_dir=str(output),
                           image_topic='/stereo/image_raw', template=str(template),
                           minimum_tags_for_pose=3, keep_intermediate=True,
                           vio_odom_topic='/openvins/odom' if odom_stamps else '')


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


def test_prepare_preserves_vio_and_configures_rig(tmp_path):
    period = 33_333_333
    stamps = [1_700_000_000_123_456_789 + i * period for i in range(15)]
    # All odometry is recorded after the images, exceeding the detector's sync queue.
    odom_stamps = stamps[1:] + [stamps[-1] + period]
    args = args_for(tmp_path, stamps, odom_stamps=odom_stamps)
    pipeline.prepare(args)
    output = Path(args.output_dir)
    config = yaml.safe_load((output / 'config/tagslam.yaml').read_text(encoding='utf-8'))
    rig = config['bodies'][0]['rig']
    assert rig['odom_topic'] == args.vio_odom_topic
    assert rig['odom_frame_id'] == 'imu'
    extrinsic = rig['T_body_odom']
    assert list(extrinsic['position'].values()) == pytest.approx([0.045, 0.002, -0.019])
    assert list(extrinsic['rotation'].values()) == pytest.approx([0, 0, np.pi / 2])
    assert pipeline.read_frames(output / 'frame_index.csv') == [
        (i, stamp, 10_000_000_000 + i * 100) for i, stamp in enumerate(stamps)]
    metadata = json.loads((output / 'run_parameters.json').read_text(encoding='utf-8'))
    assert metadata['vio_odom_count'] == len(odom_stamps)
    assert metadata['vio_matched_frame_count'] == len(stamps) - 1

    def read_bag(path):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(path)), pipeline.converter_options())
        try:
            messages = []
            while reader.has_next():
                messages.append(reader.read_next())
            return messages
        finally:
            reader.close()

    original_odometry = [raw for topic, raw, _ in read_bag(args.in_bag)
                         if topic == args.vio_odom_topic]
    replay = read_bag(output / 'stereo_bag')
    assert [raw for topic, raw, _ in replay if topic == args.vio_odom_topic] == original_odometry
    expected_topics = [(stamp, topic) for stamp in stamps for topic in pipeline.OUTPUT_TOPICS]
    expected_topics.extend((stamp, args.vio_odom_topic) for stamp in odom_stamps)
    expected_topics.append((odom_stamps[-1] + 1, pipeline.END_TOPIC))
    assert [stamp for _, _, stamp in replay] == sorted(stamp for stamp, _ in expected_topics)
    assert sorted((stamp, topic) for topic, _, stamp in replay) == sorted(expected_topics)


def test_prepare_rejects_vio_without_matching_stamps(tmp_path):
    args = args_for(tmp_path, [1_000_000_000], odom_stamps=[1_000_000_001])
    with pytest.raises(ValueError, match='no odometry timestamps matching'):
        pipeline.prepare(args)


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
    lines = (output / 'trajectory.txt').read_text(encoding='utf-8').splitlines()
    rows = [line.split(' ') for line in lines]
    assert len(rows) == 2
    assert all(len(row) == 8 for row in rows)
    assert [row[0] for row in rows] == ['1700000000.123456789', '1700000000.163456791']
    assert [float(value) for value in rows[0][1:]] == [precise, -0.125, 1.0, 0, 0, 0, 1]
    assert [float(value) for value in rows[1][1:]] == [-precise, -0.125, 1.0, 0, 0, 0, 1]
    assert not (output / 'trajectory.csv').exists()
    assert not (output / 'trajectory.txt.tmp').exists()
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
    assert (output / 'trajectory.txt').read_bytes() == b''
    assert not (output / 'trajectory.csv').exists()
    assert not (output / 'stereo_bag').exists()
    summary = json.loads((output / 'run_summary.json').read_text())
    assert (summary['frame_count'], summary['valid_pose_count'], summary['missing_pose_count']) == (1, 0, 1)


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
def test_ambiguous_or_unknown_pose_fails_without_txt(tmp_path, pose_stamps, error):
    args = args_for(tmp_path, [1_000_000_000])
    pipeline.prepare(args)
    output = Path(args.output_dir)
    make_pose_bag(output / 'out_bag', [(stamp, 0.25) for stamp in pose_stamps])
    with pytest.raises(ValueError, match=error):
        pipeline.export(SimpleNamespace(output_dir=str(output)))
    assert not (output / 'trajectory.txt').exists()
    assert not (output / 'trajectory.txt.tmp').exists()


@pytest.mark.parametrize('name', ['trajectory.txt', 'trajectory.txt.tmp'])
def test_existing_output_is_not_overwritten(tmp_path, name):
    args = args_for(tmp_path, [1_000_000_000])
    pipeline.prepare(args)
    output = Path(args.output_dir)
    make_pose_bag(output / 'out_bag', [(1_000_000_000, 0.25)])
    existing = output / name
    existing.write_text('keep existing contents\n', encoding='utf-8')
    with pytest.raises(FileExistsError, match='already exists'):
        pipeline.export(SimpleNamespace(output_dir=str(output)))
    assert existing.read_text(encoding='utf-8') == 'keep existing contents\n'
    assert (output / 'stereo_bag').exists()


def test_publish_is_atomic_and_does_not_replace_racing_target(tmp_path, monkeypatch):
    args = args_for(tmp_path, [1_000_000_001])
    pipeline.prepare(args)
    output = Path(args.output_dir)
    make_pose_bag(output / 'out_bag', [(1_000_000_001, 0.25)])
    publish = pipeline.publish_trajectory

    def publish_with_racing_target(source, target):
        # The staged file is complete before the final name becomes visible.
        assert not target.exists()
        assert source.read_text(encoding='utf-8') == '1.000000001 0.25 -0.125 1 0 0 0 1\n'
        target.write_text('created concurrently\n', encoding='utf-8')
        publish(source, target)

    monkeypatch.setattr(pipeline, 'publish_trajectory', publish_with_racing_target)
    with pytest.raises(FileExistsError):
        pipeline.export(SimpleNamespace(output_dir=str(output)))
    assert (output / 'trajectory.txt').read_text(encoding='utf-8') == 'created concurrently\n'
    assert not (output / 'trajectory.txt.tmp').exists()
    assert (output / 'stereo_bag').exists()


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux uses renameat2')
def test_publish_without_hard_links(tmp_path, monkeypatch):
    partial = tmp_path / 'trajectory.txt.tmp'
    target = tmp_path / 'trajectory.txt'
    partial.write_text('complete trajectory\n', encoding='utf-8')

    def unsupported_link(*args):
        raise OSError(errno.EOPNOTSUPP, 'hard links are unsupported')

    monkeypatch.setattr(pipeline.os, 'link', unsupported_link)
    pipeline.publish_trajectory(partial, target)
    assert target.read_text(encoding='utf-8') == 'complete trajectory\n'
    assert not partial.exists()


def test_export_preserves_racing_temporary_file(tmp_path, monkeypatch):
    args = args_for(tmp_path, [1_000_000_000])
    pipeline.prepare(args)
    output = Path(args.output_dir)
    make_pose_bag(output / 'out_bag', [(1_000_000_000, 0.25)])
    partial = output / 'trajectory.txt.tmp'
    original_open = Path.open

    def open_with_racing_partial(path, mode='r', **kwargs):
        if path == partial and mode == 'x':
            with original_open(path, 'x', encoding='utf-8') as stream:
                stream.write('owned by another export\n')
        return original_open(path, mode, **kwargs)

    monkeypatch.setattr(Path, 'open', open_with_racing_partial)
    with pytest.raises(FileExistsError):
        pipeline.export(SimpleNamespace(output_dir=str(output)))
    assert partial.read_text(encoding='utf-8') == 'owned by another export\n'
    assert not (output / 'trajectory.txt').exists()
