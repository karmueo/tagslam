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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml


SPEC = importlib.util.spec_from_file_location(
    'compare_trajectories', Path(__file__).resolve().parents[1] /
    'src/python/compare_trajectories.py')
compare = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(compare)
BASE = 1793850309039957778


def poses(points, angles=None):
    result = np.tile(np.eye(4), (len(points), 1, 1))
    result[:, :3, 3] = points
    if angles is not None:
        result[:, :3, :3] = Rotation.from_euler('z', angles, degrees=True).as_matrix()
    return result


def trajectory(times, transforms, groups=None):
    return (np.asarray(times, dtype=np.int64), transforms,
            np.zeros(len(times), dtype=int) if groups is None else np.array(groups), {})


def write_poses(path, times, transforms):
    rows = []
    for stamp, transform in zip(times, transforms):
        rows.append([int(stamp), 1, *transform[:3, 3],
                     *Rotation.from_matrix(transform[:3, :3]).as_quat()])
    write_rows(path, rows)


def write_rows(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp_ns', 'pose_valid', *compare.POSE_FIELDS])
        writer.writerows(rows)


def test_rigid_alignment_and_rotation_error():
    ref = poses([[0, 0, 0], [1, 0, 0], [1, 2, 0], [0, 1, 1]], [0, 20, 45, 90])
    world = np.eye(4)
    world[:3, :3] = Rotation.from_euler('xyz', [30, -20, 80], degrees=True).as_matrix()
    world[:3, 3] = [10, -3, 5]
    aligned, fitted = compare.align_poses(ref, np.linalg.inv(world) @ ref)
    np.testing.assert_allclose(fitted, world, atol=1e-12)
    position, angle = compare.pose_errors(ref, aligned)
    np.testing.assert_allclose(position, 0, atol=1e-12)
    np.testing.assert_allclose(angle, 0, atol=1e-12)
    erroneous = ref.copy()
    erroneous[:, :3, 3] += [0.03, 0.04, 0]
    erroneous[:, :3, :3] = erroneous[:, :3, :3] @ Rotation.from_euler('x', 12, degrees=True).as_matrix()
    position, angle = compare.pose_errors(ref, erroneous)
    assert compare.statistics(position)['rmse'] == pytest.approx(0.05)
    assert compare.statistics(angle)['rmse'] == pytest.approx(12)


def test_extrinsic_direction_and_units(tmp_path):
    camera_from_imu = np.eye(4)
    camera_from_imu[:3, :3] = Rotation.from_euler('xyz', [25, 0, 90], degrees=True).as_matrix()
    camera_from_imu[:3, 3] = [120, -30, 45]
    path = tmp_path / 'calib.yaml'
    path.write_text(yaml.safe_dump({'cam0': {'T_cam_imu': camera_from_imu.tolist()}}))
    extrinsic = compare.load_extrinsic(path)
    camera_from_imu[:3, 3] /= 1000
    camera = poses([[1, 2, 3], [3, 4, 5]], [10, 90])
    imu = camera @ camera_from_imu
    np.testing.assert_allclose(imu @ extrinsic, camera, atol=1e-12)


def test_async_interpolation_and_nanosecond_precision():
    ref = trajectory([BASE + x for x in [1, 3, 5]], poses([[0, 0, 0]] * 3))
    est = trajectory([BASE, BASE + 6], poses([[0, 0, 0], [6, 0, 0]], [0, 90]))
    times, _, result, _ = compare.synchronize(ref, est)
    np.testing.assert_array_equal(times, [BASE + 1, BASE + 3, BASE + 5])
    np.testing.assert_allclose(result[:, 0, 3], [1, 3, 5])
    np.testing.assert_allclose(Rotation.from_matrix(result[:, :3, :3]).as_euler('xyz', degrees=True)[:, 2], [15, 45, 75])


def test_filter_duplicates_sign_and_breaks(tmp_path):
    path = tmp_path / 'poses.csv'
    def row(offset, valid=1, x=0, qw=1):
        return [BASE + offset, valid, x, 0, 0, 0, 0, 0, qw]
    write_rows(path, [row(30), row(0), row(10), row(10, x=3, qw=-2),
                      row(20, valid=0), row(10, valid=0), row(200_000_000),
                      row(40, qw=0), row(50, x='nan')])
    times, transforms, groups, counts = compare.read_trajectory(path, 0.1)
    np.testing.assert_array_equal(times, [BASE, BASE + 10, BASE + 30, BASE + 200_000_000])
    np.testing.assert_array_equal(groups, [0, 0, 1, 2])
    assert transforms[1, 0, 3] == 3
    np.testing.assert_allclose(transforms[1, :3, :3], np.eye(3))
    assert counts['invalid_rows'] == 4
    assert counts['duplicate_valid_rows'] == 1
    assert counts['rows'] == counts['valid_poses'] + counts['filtered_rows']


def test_no_interpolation_across_gap_or_extrapolation():
    ref = trajectory(np.arange(9), poses([[0, 0, 0]] * 9))
    est = trajectory([1, 3, 5, 7], poses([[0, 0, 0]] * 4), [0, 0, 1, 1])
    stamps, _, _, groups = compare.synchronize(ref, est)
    np.testing.assert_array_equal(stamps, [1, 2, 3, 5, 6, 7])
    np.testing.assert_array_equal(groups, [0, 0, 0, 1, 1, 1])


def test_exact_meter_scale_drift_and_tails():
    ref = poses([[0, 0, 0], [0.6, 0, 0], [1.4, 0, 0], [2.3, 0, 0]])
    est = ref.copy()
    est[:, :3, 3] *= 1.02
    times = np.array([BASE + i * 1_000_000 for i in range(4)])
    rows, distance, tail = compare.meter_drift(times, ref, est, np.zeros(4))
    assert len(rows) == 2
    assert distance == pytest.approx(2.3)
    assert tail == pytest.approx(0.3)
    assert rows[0]['end_timestamp_ns'] == BASE + 1_500_000
    assert rows[0]['end_timestamp_ns'] == rows[1]['start_timestamp_ns']
    assert [r['translation_drift_percent'] for r in rows] == pytest.approx([2, 2])
    assert [r['rotation_error_deg'] for r in rows] == pytest.approx([0, 0])


def test_relative_rotation_drift():
    ref = poses([[0, 0, 0], [1, 0, 0], [2, 0, 0]])
    est = poses([[0, 0, 0], [1, 0, 0], [2, 0, 0]], [0, 10, 20])
    rows, _, _ = compare.meter_drift(np.arange(3), ref, est, np.zeros(3))
    assert [r['rotation_drift_deg_per_m'] for r in rows] == pytest.approx([10, 10])


def test_meter_resets_and_stationary_samples():
    ref = poses([[0, 0, 0], [0, 0, 0], [0.7, 0, 0], [10, 0, 0], [10.7, 0, 0]])
    rows, distance, tail = compare.meter_drift(np.arange(5), ref, ref, np.array([0, 0, 0, 1, 1]))
    assert rows == []
    assert distance == pytest.approx(1.4)
    assert tail == pytest.approx(1.4)
    assert compare.statistics([]) is None


@pytest.mark.parametrize('points', [[[0, 0, 0]] * 4, [[i, 0, 0] for i in range(4)]])
def test_degenerate_alignment(points):
    ref = poses(points)
    with pytest.raises(ValueError, match='degenerate'):
        compare.align_poses(ref, ref)


def test_no_overlap():
    ref = trajectory([1, 2, 3], poses([[0, 0, 0]] * 3))
    est = trajectory([4, 5, 6], poses([[0, 0, 0]] * 3))
    with pytest.raises(ValueError, match='overlap'):
        compare.synchronize(ref, est)


def test_invalid_timestamp_and_calibration(tmp_path):
    path = tmp_path / 'bad.csv'
    write_rows(path, [['not a timestamp', 1, 0, 0, 0, 0, 0, 0, 1]])
    with pytest.raises(ValueError, match='timestamp'):
        compare.read_trajectory(path, 0.1)
    path.write_text('cam0: {}')
    with pytest.raises(ValueError, match='T_cam_imu'):
        compare.load_extrinsic(path)


def test_full_report(tmp_path):
    ref = poses([[0, 0, 0], [0.1, 0, 0], [0.1, 0.1, 0], [0, 0.1, 0]], [0, 10, 20, 30])
    times = [BASE + i * 20_000_000 for i in range(4)]
    for name in ('ref', 'est'):
        write_poses(tmp_path / f'{name}.csv', times, ref)
    (tmp_path / 'calib.yaml').write_text(yaml.safe_dump({'cam0': {'T_cam_imu': np.eye(4).tolist()}}))
    args = SimpleNamespace(reference=tmp_path / 'ref.csv', estimate=tmp_path / 'est.csv',
                           calib=tmp_path / 'calib.yaml', output_dir=tmp_path / 'out', max_gap_s=0.1)
    summary = compare.analyze(args)
    assert summary['matched_poses'] == 4
    assert summary['position_error_m']['rmse'] < 1e-12
    assert summary['translation_drift_percent'] is None
    assert json.loads((args.output_dir / 'summary.json').read_text()) == summary
    for name in ('trajectory_comparison.png', 'pose_errors.png', 'drift_1m.png'):
        assert (args.output_dir / name).read_bytes().startswith(b'\x89PNG')
    with (args.output_dir / 'aligned_trajectory.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert [int(row['timestamp_ns']) for row in rows] == times
    assert len((args.output_dir / 'drift_1m.csv').read_text().splitlines()) == 1
