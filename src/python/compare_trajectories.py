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

"""Compare metric world-from-body CSV poses without requiring ROS."""

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


POSE_FIELDS = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']


def read_trajectory(path, max_gap_s):
    """Keep the last valid duplicate; invalid-only timestamps split tracks."""
    poses = {}
    invalid_times = set()
    total = invalid = duplicates = 0
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        required = {'timestamp_ns', 'pose_valid', *POSE_FIELDS}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f'{path}: missing columns {sorted(required - set(reader.fieldnames or []))}')
        for line, row in enumerate(reader, 2):
            total += 1
            try:
                stamp = int(row['timestamp_ns'])
                if not 0 <= stamp <= np.iinfo(np.int64).max:
                    raise ValueError()
            except (TypeError, ValueError):
                raise ValueError(f'{path}:{line}: invalid integer timestamp_ns') from None
            try:
                values = np.array([float(row[k]) for k in POSE_FIELDS])
                norm = np.linalg.norm(values[3:])
                if (int(row['pose_valid']) != 1 or not np.isfinite(values).all()
                        or not np.isfinite(norm) or norm < 1e-12):
                    raise ValueError()
                transform = np.eye(4)
                transform[:3, :3] = Rotation.from_quat(values[3:] / norm).as_matrix()
                transform[:3, 3] = values[:3]
            except (TypeError, ValueError):
                invalid += 1
                invalid_times.add(stamp)
                continue
            duplicates += int(stamp in poses)
            poses[stamp] = transform
    if not poses:
        raise ValueError(f'{path}: no valid poses')
    stamps = np.array(sorted(poses), dtype=np.int64)
    breaks = np.array(sorted(invalid_times - poses.keys()), dtype=np.int64)
    segments = np.zeros(len(stamps), dtype=int)
    for i in range(1, len(stamps)):
        has_invalid = (np.searchsorted(breaks, stamps[i]) >
                       np.searchsorted(breaks, stamps[i - 1], side='right'))
        segments[i] = segments[i - 1] + int(
            stamps[i] - stamps[i - 1] > max_gap_s * 1e9 or has_invalid)
    return stamps, np.array([poses[int(t)] for t in stamps]), segments, {
        'rows': total, 'invalid_rows': invalid, 'duplicate_valid_rows': duplicates,
        'filtered_rows': invalid + duplicates, 'valid_poses': len(stamps),
        'continuous_segments': int(segments[-1] + 1),
    }


def load_extrinsic(path):
    """Return IMU-from-cam0; calibration input is cam0-from-IMU in mm."""
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    try:
        transform = np.asarray(data['cam0']['T_cam_imu'], dtype=float)
    except (KeyError, TypeError, ValueError):
        raise ValueError('calibration must contain cam0.T_cam_imu') from None
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-5)):
        raise ValueError('cam0.T_cam_imu must be a finite rigid 4x4 transform')
    transform = transform.copy()
    transform[:3, 3] /= 1000
    return np.linalg.inv(transform)


def interpolate_pose(a, b, fraction):
    """Linear translation and shortest-arc quaternion SLERP, including q/-q."""
    result = np.eye(4)
    result[:3, 3] = (1 - fraction) * a[:3, 3] + fraction * b[:3, 3]
    ra, rb = Rotation.from_matrix(a[:3, :3]), Rotation.from_matrix(b[:3, :3])
    result[:3, :3] = (ra * Rotation.from_rotvec(
        (ra.inv() * rb).as_rotvec() * fraction)).as_matrix()
    return result


def synchronize(reference, estimate):
    """Sample estimate on reference timestamps; never bridge discontinuities."""
    rt, rp, rs, _ = reference
    et, ep, es, _ = estimate
    stamps, refs, ests, groups = [], [], [], []
    previous = None
    group = -1
    for i, stamp in enumerate(rt):
        j = int(np.searchsorted(et, stamp))
        if j < len(et) and et[j] == stamp:
            pose, segment = ep[j], int(es[j])
        elif j == 0 or j == len(et) or es[j - 1] != es[j]:
            continue
        else:
            fraction = int(stamp - et[j - 1]) / int(et[j] - et[j - 1])
            pose = interpolate_pose(ep[j - 1], ep[j], fraction)
            segment = int(es[j])
        key = (int(rs[i]), segment)
        if previous is None or previous != (i - 1, key):
            group += 1
        previous = (i, key)
        stamps.append(stamp)
        refs.append(rp[i])
        ests.append(pose)
        groups.append(group)
    if len(stamps) < 3:
        raise ValueError('fewer than three synchronized poses (no overlap or insufficient valid data)')
    return np.array(stamps, dtype=np.int64), np.array(refs), np.array(ests), np.array(groups)


def align_poses(reference, estimate):
    """Fit reference = R @ estimate + t with unit scale, rejecting rank < 2."""
    ref, est = reference[:, :3, 3], estimate[:, :3, 3]
    rc, ec = ref - ref.mean(axis=0), est - est.mean(axis=0)
    u, singular, vt = np.linalg.svd(ec.T @ rc)
    if len(ref) < 3 or singular[0] < 1e-12 or singular[1] <= singular[0] * 1e-10:
        raise ValueError('degenerate position geometry: rigid alignment rotation is not uniquely determined')
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    transform = np.eye(4)
    transform[:3, :3] = vt.T @ correction @ u.T
    transform[:3, 3] = ref.mean(axis=0) - transform[:3, :3] @ est.mean(axis=0)
    return transform @ estimate, transform


def pose_errors(reference, estimate):
    position = np.linalg.norm(estimate[:, :3, 3] - reference[:, :3, 3], axis=1)
    relative = (Rotation.from_matrix(reference[:, :3, :3]).inv() *
                Rotation.from_matrix(estimate[:, :3, :3]))
    return position, np.rad2deg(relative.magnitude())


def statistics(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return {k: float(v) for k, v in {
        'rmse': np.sqrt(np.mean(values ** 2)), 'mean': np.mean(values),
        'median': np.median(values), 'p95': np.percentile(values, 95),
        'max': np.max(values),
    }.items()}


def meter_drift(stamps, reference, estimate, groups):
    """Nonoverlapping exact one-meter intervals within each continuous group."""
    rows = []
    total_distance = discarded_distance = 0.0
    for group in np.unique(groups):
        mask = groups == group
        times, ref, est = stamps[mask], reference[mask], estimate[mask]
        distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(ref[:, :3, 3], axis=0), axis=1))]
        total_distance += float(distance[-1])
        count = int(np.floor(distance[-1]))
        discarded_distance += float(distance[-1] - count)

        def at(boundary):
            if boundary == 0:
                return int(times[0]), ref[0], est[0]
            j = int(np.searchsorted(distance, boundary, side='left'))
            fraction = (boundary - distance[j - 1]) / (distance[j] - distance[j - 1])
            stamp = int(times[j - 1]) + round(int(times[j] - times[j - 1]) * fraction)
            return (stamp, interpolate_pose(ref[j - 1], ref[j], fraction),
                    interpolate_pose(est[j - 1], est[j], fraction))

        start = at(0)
        for meter in range(count):
            end = at(meter + 1)
            delta_ref = np.linalg.inv(start[1]) @ end[1]
            delta_est = np.linalg.inv(start[2]) @ end[2]
            error = np.linalg.inv(delta_ref) @ delta_est
            translation = float(np.linalg.norm(error[:3, 3]))
            angle = float(np.rad2deg(Rotation.from_matrix(error[:3, :3]).magnitude()))
            rows.append({'segment_id': int(group), 'start_distance_m': meter,
                         'end_distance_m': meter + 1, 'start_timestamp_ns': start[0],
                         'end_timestamp_ns': end[0], 'translation_error_m': translation,
                         'translation_drift_percent': translation * 100,
                         'rotation_error_deg': angle, 'rotation_drift_deg_per_m': angle})
            start = end
    return rows, total_distance, discarded_distance


def write_csv(path, fields, rows):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def save_plots(output, stamps, ref, est, groups, position, rotation, drift):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    for number, group in enumerate(np.unique(groups)):
        mask = groups == group
        for poses, label, color in ((ref, 'Reference (TagSLAM)', 'C0'),
                                    (est, 'Aligned estimate (OpenVINS)', 'C1')):
            ax.plot(*poses[mask, :3, 3].T, color=color, label=label if number == 0 else None)
    ax.set(xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]', title='Rigid alignment (scale = 1)')
    extent = np.ptp(np.concatenate((ref[:, :3, 3], est[:, :3, 3])), axis=0)
    ax.set_box_aspect(np.maximum(extent, max(float(extent.max()), 1e-6) * 0.01))
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / 'trajectory_comparison.png', dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    elapsed = (stamps - stamps[0]).astype(float) / 1e9
    for group in np.unique(groups):
        mask = groups == group
        axes[0].plot(elapsed[mask], position[mask], color='C0')
        axes[1].plot(elapsed[mask], rotation[mask], color='C1')
    axes[0].set_ylabel('Position error [m]')
    axes[1].set(ylabel='Rotation error [deg]', xlabel='Time from first match [s]')
    fig.tight_layout()
    fig.savefig(output / 'pose_errors.png', dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    if drift:
        indices = np.arange(1, len(drift) + 1)
        axes[0].plot(indices, [r['translation_drift_percent'] for r in drift], '.-')
        axes[1].plot(indices, [r['rotation_drift_deg_per_m'] for r in drift], '.-', color='C1')
    else:
        axes[0].text(0.5, 0.5, 'No complete 1 m intervals', ha='center', transform=axes[0].transAxes)
    axes[0].set_ylabel('Translation drift [%]')
    axes[1].set(ylabel='Rotation drift [deg/m]', xlabel='Complete 1 m interval index')
    fig.tight_layout()
    fig.savefig(output / 'drift_1m.png', dpi=160)
    plt.close(fig)


def analyze(args):
    if not np.isfinite(args.max_gap_s) or args.max_gap_s <= 0:
        raise ValueError('--max-gap-s must be finite and positive')
    reference = read_trajectory(args.reference, args.max_gap_s)
    estimate = read_trajectory(args.estimate, args.max_gap_s)
    extrinsic = load_extrinsic(args.calib)
    estimate = (estimate[0], estimate[1] @ extrinsic, estimate[2], estimate[3])
    stamps, ref, est, groups = synchronize(reference, estimate)
    aligned, alignment = align_poses(ref, est)
    position, rotation = pose_errors(ref, aligned)
    drift, distance, discarded = meter_drift(stamps, ref, aligned, groups)
    summary = {
        'inputs': {k: str(Path(getattr(args, k)).resolve()) for k in ('reference', 'estimate', 'calib')},
        'conventions': {'reference': 'TagSLAM, not ground truth', 'estimate': 'OpenVINS',
                        'comparison_body': 'cam0 = rig', 'pose': 'world_from_body',
                        'quaternion': 'xyzw', 'position_unit': 'm', 'scale': 1.0,
                        'time_offset_s': 0.0, 'max_gap_s': args.max_gap_s},
        'estimate_right_transform_imu_from_cam0_m': extrinsic.tolist(),
        'alignment_reference_world_from_estimate_world': alignment.tolist(),
        'reference_input': reference[3], 'estimate_input': estimate[3],
        'matched_poses': len(stamps), 'unmatched_reference_poses': len(reference[0]) - len(stamps),
        'start_timestamp_ns': int(stamps[0]), 'end_timestamp_ns': int(stamps[-1]),
        'time_span_s': int(stamps[-1] - stamps[0]) / 1e9,
        'continuous_segments': int(groups[-1] + 1),
        'valid_duration_s': float(sum(int(stamps[i] - stamps[i - 1]) / 1e9
                                     for i in range(1, len(stamps)) if groups[i] == groups[i - 1])),
        'reference_distance_m': distance, 'discarded_partial_distance_m': discarded,
        'position_error_m': statistics(position), 'rotation_error_deg': statistics(rotation),
        'complete_1m_intervals': len(drift),
        'translation_drift_percent': statistics([r['translation_drift_percent'] for r in drift]),
        'rotation_drift_deg_per_m': statistics([r['rotation_drift_deg_per_m'] for r in drift]),
        'definitions': {
            'rmse': 'sqrt(mean(error^2)), equally weighted synchronized samples',
            'rotation': 'shortest angle of inverse(R_reference) * R_estimate',
            'drift': 'E = inverse(inverse(T_ref_start)*T_ref_end) * (inverse(T_est_start)*T_est_end)',
            'translation_percent': '100 * norm(translation(E)) / 1m',
            'intervals': 'nonoverlapping reference arc-length intervals; reset at discontinuities; incomplete tails omitted',
            'duplicate_policy': 'last valid row wins; invalid-only timestamps break continuity',
        },
    }
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    fields = ['timestamp_ns', 'relative_time_s', 'segment_id']
    fields += [f'{prefix}_{field}' for prefix in ('reference', 'estimate') for field in POSE_FIELDS]
    fields += ['position_error_m', 'rotation_error_deg']
    ref_quat = Rotation.from_matrix(ref[:, :3, :3]).as_quat()
    est_quat = Rotation.from_matrix(aligned[:, :3, :3]).as_quat()
    rows = []
    for i, stamp in enumerate(stamps):
        values = [int(stamp), int(stamp - stamps[0]) / 1e9, int(groups[i])]
        values += [*ref[i, :3, 3], *ref_quat[i], *aligned[i, :3, 3], *est_quat[i], position[i], rotation[i]]
        rows.append(dict(zip(fields, values)))
    write_csv(output / 'aligned_trajectory.csv', fields, rows)
    write_csv(output / 'drift_1m.csv', [
        'segment_id', 'start_distance_m', 'end_distance_m', 'start_timestamp_ns',
        'end_timestamp_ns', 'translation_error_m', 'translation_drift_percent',
        'rotation_error_deg', 'rotation_drift_deg_per_m'], drift)
    save_plots(output, stamps, ref, aligned, groups, position, rotation, drift)
    print(f"Matched poses: {len(stamps)}; continuous segments: {groups[-1] + 1}")
    print(f"Position RMSE: {summary['position_error_m']['rmse']:.6f} m")
    print(f"Rotation RMSE: {summary['rotation_error_deg']['rmse']:.6f} deg")
    if drift:
        print(f"1 m intervals: {len(drift)}; translation drift mean / RMSE: "
              f"{summary['translation_drift_percent']['mean']:.6f}% / "
              f"{summary['translation_drift_percent']['rmse']:.6f}%")
        print(f"Rotation drift mean: {summary['rotation_drift_deg_per_m']['mean']:.6f} deg/m")
    else:
        print('1 m drift: 不可计算 (no complete 1 m intervals)')
    print(f'Reference is TagSLAM, not ground truth. Output: {output.resolve()}')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True, help='TagSLAM CSV, rig = cam0')
    parser.add_argument('--estimate', required=True, help='OpenVINS CSV, body = imu')
    parser.add_argument('--calib', required=True, help='YAML cam0.T_cam_imu, translation in mm')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--max-gap-s', type=float, default=0.1)
    args = parser.parse_args()
    try:
        analyze(args)
    except (ValueError, OSError, yaml.YAMLError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
