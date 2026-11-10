#!/usr/bin/env python3
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
"""Prepare stereo MCAP input and export frame-aligned optimized poses."""

import argparse
from array import array
import csv
import ctypes
import errno
import json
import math
import os
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty
import yaml

OUTPUT_TOPICS = ('/cam0/image_raw', '/cam1/image_raw')
END_TOPIC = '/tagslam/end_of_bag'
POSE_TOPIC = '/tagslam/odom/body_rig'
INDEX_FIELDS = ('frame_index', 'timestamp_ns', 'bag_timestamp_ns')


def timestamp_ns(header):
    """Return a ROS header time exactly, without floating point arithmetic."""
    sec, nanosec = int(header.stamp.sec), int(header.stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError(f'invalid image or pose timestamp: {sec}.{nanosec}')
    return sec * 1_000_000_000 + nanosec


def camera_configuration(calibration):
    """Convert the supplied calibration convention into TagSLAM YAML data."""
    cameras = {}
    dimensions = []
    for name in ('cam0', 'cam1'):
        camera = calibration[name]
        resolution = camera['resolution']
        if (len(resolution) != 2 or any(isinstance(x, bool) or
                not isinstance(x, int) or x <= 0 for x in resolution)):
            raise ValueError(f'{name} resolution must contain two positive integers')
        dimensions.append(tuple(resolution))
        intrinsics = [float(x) for x in camera['intrinsics']]
        distortion = [float(x) for x in camera['distortion_coeffs']]
        if (len(intrinsics) != 4 or len(distortion) != 4 or
                not all(math.isfinite(x) for x in intrinsics + distortion) or
                intrinsics[0] <= 0 or intrinsics[1] <= 0):
            raise ValueError(f'{name} must have four finite intrinsics and distortion values and positive focal lengths')
        if camera['distortion_model'] != 'fisheye':
            raise ValueError(f'{name} must use the fisheye distortion model')
        cameras[name] = {
            'camera_model': 'pinhole', 'intrinsics': intrinsics,
            'distortion_model': 'fisheye', 'distortion_coeffs': distortion,
            'resolution': resolution, 'image_topic': f'/{name}/image_raw',
            'image_transport': 'raw', 'tag_topic': f'/{name}/tags',
            'tag_detector': 'umich', 'rig_body': 'rig',
        }
    if dimensions[0] != dimensions[1]:
        raise ValueError('left and right image resolutions must match')

    transform = np.asarray(calibration['T_cam1_cam0'], dtype=np.float64)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-5)):
        raise ValueError('T_cam1_cam0 must be a finite rigid 4x4 transform')
    transform = transform.copy()
    transform[:3, 3] /= 1000.0  # Input translation is in millimeters.
    inverse = np.linalg.inv(transform)
    rotvec, _ = cv2.Rodrigues(inverse[:3, :3])

    def pose(position, rotation):
        def xyz(values):
            return {key: float(value) for key, value in zip('xyz', values)}
        return {'pose': {'position': xyz(position), 'rotation': xyz(rotation),
                         'position_noise': xyz([1e-7] * 3),
                         'rotation_noise': xyz([1e-7] * 3)}}

    poses = {'cam0': pose([0, 0, 0], [0, 0, 0]),
             'cam1': pose(inverse[:3, 3], rotvec.ravel())}
    return cameras, poses, dimensions[0]


def converter_options():
    return rosbag2_py.ConverterOptions('cdr', 'cdr')


def prepare(args):
    output = Path(args.output_dir)
    config_dir = output / 'config'
    config_dir.mkdir()
    with open(args.calib, encoding='utf-8') as stream:
        calibration = yaml.safe_load(stream)
    cameras, camera_poses, (width, height) = camera_configuration(calibration)
    for filename, content in (('cameras.yaml', cameras), ('camera_poses.yaml', camera_poses)):
        with (config_dir / filename).open('w', encoding='utf-8') as stream:
            yaml.safe_dump(content, stream, sort_keys=False)
    shutil.copyfile(args.calib, config_dir / 'calib.yaml')
    shutil.copyfile(args.template, config_dir / 'tagslam.yaml')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.in_bag, storage_id='mcap'),
                converter_options())
    writer = None
    frame_count = 0
    last_header_stamp = None
    last_bag_stamp = None
    try:
        types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        if types.get(args.image_topic) != 'sensor_msgs/msg/CompressedImage':
            raise ValueError(f'{args.image_topic} is missing or is not a CompressedImage')
        reader.set_filter(rosbag2_py.StorageFilter(topics=[args.image_topic]))
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output / 'stereo_bag'), storage_id='mcap'),
                    converter_options())
        for topic in OUTPUT_TOPICS:
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0, name=topic, type='sensor_msgs/msg/Image', serialization_format='cdr'))
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=END_TOPIC, type='std_msgs/msg/Empty', serialization_format='cdr'))
        with (output / 'frame_index.csv').open('w', newline='', encoding='utf-8') as stream:
            index = csv.writer(stream)
            index.writerow(INDEX_FIELDS)
            while reader.has_next():
                _, serialized, bag_stamp = reader.read_next()
                compressed = deserialize_message(serialized, CompressedImage)
                stamp = timestamp_ns(compressed.header)
                if last_header_stamp is not None and stamp <= last_header_stamp:
                    raise ValueError(f'image timestamp at frame {frame_count} is not strictly increasing: '
                                     f'{stamp} <= {last_header_stamp}')
                try:
                    image = cv2.imdecode(np.frombuffer(compressed.data, dtype=np.uint8),
                                         cv2.IMREAD_GRAYSCALE)
                except cv2.error as error:
                    raise ValueError(f'cannot decode stereo image at {stamp}: {error}') from error
                if image is None or image.shape != (height, 2 * width):
                    shape = None if image is None else image.shape
                    raise ValueError(f'invalid stereo image at {stamp}: shape={shape}, '
                                     f'expected {(height, 2 * width)}')
                for camera_index, topic in enumerate(OUTPUT_TOPICS):
                    pixels = np.ascontiguousarray(
                        image[:, camera_index * width:(camera_index + 1) * width])
                    message = Image(header=compressed.header, height=height, width=width,
                                    encoding='mono8', is_bigendian=0, step=width,
                                    data=array('B', pixels.tobytes()))
                    writer.write(topic, serialize_message(message), bag_stamp)
                index.writerow((frame_count, stamp, bag_stamp))
                frame_count += 1
                last_header_stamp, last_bag_stamp = stamp, bag_stamp
                if frame_count % 100 == 0:
                    print(f'Prepared {frame_count} stereo frames', flush=True)
            if not frame_count:
                raise ValueError(f'no image messages found on {args.image_topic}')
            if frame_count >= 2_147_483_647:
                raise ValueError('too many frames for the TagSLAM frame limit')
            # One additional event ensures the last synchronized detections are processed.
            writer.write(END_TOPIC, serialize_message(Empty()), last_bag_stamp + 1)
    finally:
        if writer is not None:
            writer.close()
        reader.close()

    metadata = {'input_mcap': args.in_bag, 'calib': args.calib,
                'output_dir': args.output_dir, 'image_topic': args.image_topic,
                'minimum_tags_for_pose': args.minimum_tags_for_pose,
                'keep_intermediate': args.keep_intermediate, 'frame_count': frame_count}
    (output / 'run_parameters.json').write_text(json.dumps(metadata, indent=2) + '\n',
                                                 encoding='utf-8')
    print(f'Prepared {frame_count} frames in {output}', flush=True)


def read_frames(index_path):
    frames = []
    with open(index_path, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(INDEX_FIELDS):
            raise ValueError('invalid frame index columns')
        last = None
        for row in reader:
            frame = tuple(int(row[field]) for field in INDEX_FIELDS)
            if frame[0] != len(frames) or (last is not None and frame[1] <= last):
                raise ValueError(f'invalid frame order or timestamp at frame {len(frames)}')
            frames.append(frame)
            last = frame[1]
    if not frames:
        raise ValueError('frame index is empty')
    return frames


def read_poses(bag_path, valid_stamps):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path)), converter_options())
    poses = {}
    try:
        types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        if types.get(POSE_TOPIC) != 'nav_msgs/msg/Odometry':
            raise ValueError(f'output bag is missing {POSE_TOPIC} Odometry topic')
        reader.set_filter(rosbag2_py.StorageFilter(topics=[POSE_TOPIC]))
        while reader.has_next():
            _, data, _ = reader.read_next()
            message = deserialize_message(data, Odometry)
            stamp = timestamp_ns(message.header)
            if stamp not in valid_stamps:
                raise ValueError(f'pose timestamp does not match an input image: {stamp}')
            if stamp in poses:
                raise ValueError(f'duplicate pose timestamp: {stamp}')
            if message.header.frame_id != 'map' or message.child_frame_id != 'rig':
                raise ValueError(f'unexpected pose frames at {stamp}: '
                                 f'{message.header.frame_id!r}, {message.child_frame_id!r}')
            p, q = message.pose.pose.position, message.pose.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(x) for x in values):
                raise ValueError(f'non-finite pose at {stamp}')
            poses[stamp] = values
    finally:
        reader.close()
    return poses


def publish_trajectory(partial, target):
    """Atomically publish a completed trajectory without replacing the target."""
    if sys.platform == 'linux':
        rename = getattr(ctypes.CDLL(None, use_errno=True), 'renameat2', None)
        if rename is not None:
            rename.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int,
                               ctypes.c_char_p, ctypes.c_uint)
            at_fdcwd = -100
            rename_noreplace = 1
            if rename(at_fdcwd, os.fsencode(partial), at_fdcwd, os.fsencode(target),
                      rename_noreplace) == 0:
                return
            error = ctypes.get_errno()
            if error not in (errno.ENOSYS, errno.EINVAL, errno.EOPNOTSUPP):
                raise OSError(error, os.strerror(error), target)

    # Older systems and filesystems can still publish atomically with a hard link.
    os.link(partial, target)
    partial.unlink()


def export(args):
    output = Path(args.output_dir)
    metadata = json.loads((output / 'run_parameters.json').read_text(encoding='utf-8'))
    frames = read_frames(output / 'frame_index.csv')
    if len(frames) != metadata['frame_count']:
        raise ValueError('frame index does not match prepared frame count')
    poses = read_poses(output / 'out_bag', {frame[1] for frame in frames})
    target = output / 'trajectory.txt'
    partial = output / 'trajectory.txt.tmp'
    if target.exists() or partial.exists():
        raise FileExistsError('trajectory TXT or temporary output already exists')
    stream = partial.open('x', newline='', encoding='utf-8')
    try:
        with stream:
            for _, stamp, _ in frames:
                pose = poses.get(stamp)
                if pose is not None:
                    timestamp = f'{stamp // 1_000_000_000}.{stamp % 1_000_000_000:09d}'
                    values = ' '.join(format(value, '.17g') for value in pose)
                    stream.write(f'{timestamp} {values}\n')
        publish_trajectory(partial, target)
    except BaseException:
        partial.unlink(missing_ok=True)
        raise
    if not metadata['keep_intermediate']:
        shutil.rmtree(output / 'stereo_bag')
    metadata.update({'valid_pose_count': len(poses),
                     'missing_pose_count': len(frames) - len(poses), 'status': 'success'})
    summary = output / 'run_summary.json'
    summary.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    print(f'Exported {len(frames)} frames: {len(poses)} valid poses, '
          f'{len(frames) - len(poses)} missing; {target}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest='stage', required=True)
    prep = subcommands.add_parser('prepare')
    for name in ('in-bag', 'calib', 'output-dir', 'image-topic', 'template'):
        prep.add_argument('--' + name, required=True)
    prep.add_argument('--minimum-tags-for-pose', type=int, required=True)
    prep.add_argument('--keep-intermediate', action='store_true')
    exp = subcommands.add_parser('export')
    exp.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    try:
        (prepare if args.stage == 'prepare' else export)(args)
    except Exception as error:
        print(f'{args.stage} failed: {error}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
