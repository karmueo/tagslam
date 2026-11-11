# TagSLAM: SLAM with Tags

TagSLAM is a ROS2-based package for Simultaneous Localization and
Mapping using Apriltag fiducial markers.

ROS2 is WORK IN PROGRESS. Documentation will follow.

## Platforms supported

At the moment TagSLAM requires ROS2 Rolling/Jazzy or newer.
Older versions of ROS2 lack some important rosbag2 features.

## How to build

The build instructions follow the standard procedure for ROS2. Set the following shell variables:

```bash
repo=tagslam
url=https://github.com/berndpfrommer/${repo}.git
```

and follow the ROS2 build instructions
[here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md).
However, you need to modify the instructions to compile the ``ros2`` branch.

Make sure to source your workspace's ``install/setup.bash`` afterwards.


## How to use


### Sync and detect

TagSLAM operates off of tags (and odometry messages, if provided). In a scenerio with multiple cameras or odometry it is important
that all sensors are synchronized, meaning that the sensor data has matching ROS header.stamp fields.
Sync_and_detect runs the apriltag detector across multiple cameras and emits synchronized messages with the decoded tags.
These messages in turn are used by TagSLAM. Note that ``sync_and_detect`` can also deal with odometry:
it drops all odometry messages except for the ones that coincide (approximately) with the camera images, and alters the header.stamp
field to match exactly the ones of the image messages.

You can run ``sync_and_detect`` either from a bag file, and write the
detected tags into another bag, or you can run it as a stand-alone (composable) node.
It will use the ``cameras.yaml`` file to determine what topics to read from the input bag, what image transport (raw vs compressed), what tag detector
(MIT vs UMich), and what output tag topics to use. The ``tagslam.yaml`` file is searched for bodies with odometry topics.


Here is how to run it from a bag:
```
ros2 run tagslam sync_and_detect_from_bag --ros-args -p "cameras:=./cameras.yaml" -p "tagslam_config:=./tagslam.yaml" -p "in_bag:=name_of_input_bag" -p "out_bag:=./tag_bag"
```

For online operation, launch a ``sync_and_detect`` node like this:
```
ros2 launch tagslam sync_and_detect.launch.py use_sim_time:=<True/False> cameras:=<path_to_cameras.yaml_file> tagslam_config:=<path_to_tagslam_config_file> use_approximate_sync:=<True/False>
```

### TagSLAM

TagSLAM can run off a rosbag, or as a node. When running off a bag, TagSLAM will automatically recognize when there are only
image topics, but no tag topics in the rosbag, and will start ``sync_and_detect`` to do tag detection.

Run TagSLAM from a rosbag like this:
```
ros2 run tagslam tagslam_from_bag --ros-args -p "cameras:=./cameras.yaml" -p "tagslam_config:=./tagslam.yaml" -p "camera_poses:=./camera_poses.yaml" -p "in_bag:=./bag_with_tags_and_odom" -p "out_bag:=./out_bag"
```

For online operation, launch a ``tagslam`` node like this:
```
ros2 launch tagslam tagslam.launch.py use_sim_time:=<True/False> cameras:=<path_to_cameras.yaml_file> camera_poses:=<path_to_camera_poses.yaml file> tagslam_config:=<path_to_tagslam_config_file> use_approximate_sync:=<True/False>
```

To view a TagSLAM output bag, source the ROS and workspace environments and
run the viewer from a graphical session:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/workspace/install/setup.bash
ros2 launch tagslam view_bag.launch.py in_bag:=/absolute/path/to/out_bag
# Optional: adjust playback speed and the segment gap in seconds.
ros2 launch tagslam view_bag.launch.py in_bag:=/absolute/path/to/out_bag rate:=1.0 max_gap:=0.03
```

Playback defaults to half speed and waits for RViz to subscribe. Poses with
consecutive timestamps more than `max_gap` seconds apart are shown as
separate segments; isolated poses are shown as points. The default gap is
0.03 seconds. RViz remains open after playback; closing it or pressing Ctrl+C
stops the launched processes. If RViz asks to save its configuration on close,
choose **Discard** to exit without modifying the installed configuration.

By default, pose estimation requires at least one usable tag ID
across all cameras in a synchronized frame. The same ID seen by multiple
cameras counts once. Hamming distance, squash and ID remapping are applied
before counting; tags rejected by the configured minimum area or
body-association rules do not count. Tags that
can be added to the configured default body count even before they are mapped.
Frames below the threshold are skipped entirely, including their odometry;
they produce no new pose or dynamic transform and may create gaps in the
trajectory. Tag detection output is unaffected.

Set the startup-only ROS parameter `minimum_tags_for_pose` to change the
threshold, or set it to `0` to disable this check and restore the previous
behavior. Negative values are invalid. To require three tags, add
`-p minimum_tags_for_pose:=3` after `--ros-args` for bag processing, or add
`minimum_tags_for_pose:=3` to the online launch command. The final frame summary
reports input, threshold-passing, skipped and solved frame counts separately.

### Estimate frame-aligned poses from a stereo MCAP

For MCAP files containing a side-by-side stereo `CompressedImage` topic,
run the complete offline workflow after sourcing ROS and this workspace:

```bash
ros2 launch tagslam estimate_from_mcap.launch.py \
  in_bag:=/absolute/path/to/job_004.mcap
```

The launch reads `calib.yaml` next to the MCAP by default; override it with
`calib:=/absolute/path/to/calib.yaml`. Calibration translations in
`T_cam1_cam0` are interpreted in millimeters. Both cameras must have the
same resolution and fisheye distortion model. The default topic is
`/stereo/image_raw`; set `image_topic:=/other/topic` if needed. The installed
configuration uses 144 mm AprilTags, tag 0 as the map origin, and the left
camera as the rig origin. Set `minimum_tags_for_pose:=0` to disable the
default one-tag filter. A frame can pass the filter and still lack a solved pose.

To fuse OpenVINS with AprilTag observations, pass the IMU-frame odometry
topic from the same MCAP.

```bash
ros2 launch tagslam estimate_from_mcap.launch.py \
  in_bag:=/absolute/path/to/output/job_004_openvins.mcap \
  calib:=/absolute/path/to/calib.yaml \
  vio_odom_topic:=/openvins/odom minimum_tags_for_pose:=0
```

The VIO topic must contain `nav_msgs/msg/Odometry` with consistent frame IDs
and header timestamps matching the stereo images. The pipeline preserves the
odometry messages and uses `cam0.T_cam_imu` from the calibration file
(IMU to left camera, with translation in millimeters). The override to
`0` allows synchronized frames with no usable tags to use VIO constraints.
Images without a matching odometry message remain in `frame_index.csv`.
The temporary bag replays images and VIO in header timestamp order. The frame
index retains the original image recording timestamps.
The fused odometry on `/tagslam/odom/body_rig` has `frame_id: map` and
`child_frame_id: rig`. The run summary includes VIO message and matched-frame
counts.

By default a new directory is created in `./tagslam_runs/`; set
`output_dir:=/absolute/path/to/run` to choose its location. An existing
`output_dir` is deleted and recreated, so repeat runs into the same path start
clean; the launch file refuses to replace a directory that contains the current
working directory, input MCAP, or calibration file (including symlink targets).
On success the directory contains `trajectory.txt`,
`out_bag/`, the actual `config/`, a `frame_index.csv`, and
`run_summary.json`; stage logs are `prepare.log`, `tagslam.log` and
`export.log`. The temporary split stereo bag is removed on success;
set `keep_intermediate:=true` to retain it. On failure the output directory
and intermediate data remain available for diagnosis.

The trajectory uses TUM format, with one space-separated line per valid pose
in input frame order and no header:

```text
timestamp tx ty tz qx qy qz qw
```

The timestamp is the original image header time in seconds with nine decimal
places, allowing each pose to be matched to an input frame. Each pose is the
final optimized `map` to `rig` pose, with position in meters and quaternion
in `x,y,z,w` order. Frames without a solved pose are omitted without
interpolation, so the TXT line count equals the number of valid poses. An
all-missing result is an empty TXT file. The separate `frame_index.csv` retains
every input frame, its zero-based index, original header timestamp in integer
nanoseconds, and MCAP record timestamp. A threshold-passing frame can also
lack a solved pose.
Repeated or decreasing image timestamps, or duplicated or unmatched output
pose timestamps, cause an error rather than ambiguous alignment. The result
bag can be inspected with `view_bag.launch.py` as described above.


### Compare TagSLAM and OpenVINS trajectory CSVs

The standalone Python 3 tool uses NumPy, SciPy, PyYAML and Matplotlib; it
requires no ROS runtime. After building, it is also available as
`ros2 run tagslam compare_trajectories.py` with the same arguments.
It accepts trajectory CSVs only and cannot directly read the TUM TXT output
of `estimate_from_mcap.launch.py`; previously generated CSVs remain usable.

```bash
python3 src/python/compare_trajectories.py \
  --reference /data/scl_data/slam/stereo_live_bags/EGO/400hz/tagslam_output/trajectory.csv \
  --estimate /data/scl_data/slam/stereo_live_bags/EGO/400hz/openvins_output/job_004.csv \
  --calib /data/scl_data/slam/stereo_live_bags/EGO/400hz/calib.yaml \
  --output-dir /tmp/tagslam_trajectory_comparison_job004
```

Both CSVs must contain `timestamp_ns`, `pose_valid`, `x,y,z,qx,qy,qz,qw`.
Poses are world-from-body, positions are in meters, and quaternions use
`x,y,z,w` order. TagSLAM is the reference (not ground truth), with `rig=cam0`;
OpenVINS poses describe the IMU. The tool converts OpenVINS poses to cam0 by
right-multiplying the inverse of `cam0.T_cam_imu` from the calibration YAML,
whose translation is in millimeters. It does not apply the calibration's
IMU time offset again or estimate a time offset or scale.

Integer nanosecond timestamps retain their precision. Invalid poses are
filtered, quaternions normalized, timestamps sorted, and the last valid row
at each duplicate timestamp retained. Invalid-only timestamps split the
trajectory. On overlapping reference timestamps, estimate positions are
linearly interpolated and orientations use shortest-arc SLERP. No
extrapolation or interpolation across breaks is allowed. Sampling gaps over
`--max-gap-s` (default `0.1`) also split either trajectory. Missing or malformed
integer timestamps fail explicitly because their break location is unknown.

All matched positions determine one least-squares rigid alignment, without
scale correction; its rotation also transforms orientations. Fewer than
three matches or degenerate position geometry (stationary/collinear tracks)
produce an error. Position RMSE is the root mean square of 3D position-error
norms; rotation RMSE uses shortest relative rotation angles in degrees. Each
matched sample has equal weight. Mean, median, P95 and maximum are also
reported.

One-meter drift uses nonoverlapping intervals along the reference polyline,
with both trajectories interpolated at exact meter boundaries. For each
interval, `delta = inverse(T_start) * T_end` and
`E = inverse(delta_reference) * delta_estimate`. Translation drift is
`100 * norm(translation(E)) / 1m` percent; rotation drift is the angle of
`rotation(E)` in degrees per meter. Counting restarts at every break;
incomplete tails are omitted and their total distance reported. If no
complete interval exists, drift is unavailable (`null` in JSON), while
absolute errors are still reported.

The output directory contains `summary.json` (inputs, conventions, transforms,
filter counts and statistics), `aligned_trajectory.csv` (matched reference
and aligned estimate poses, segment IDs and errors), `drift_1m.csv` (one row
per complete interval), and `trajectory_comparison.png`, `pose_errors.png`,
`drift_1m.png`. Plots work without a graphical desktop. Rerunning with the
same output directory replaces these generated files. In the drift CSV,
distances restart at zero for each `segment_id`; timestamp columns locate
segments in the original data.

### Rosbag

When playing from a ros2 bag it's important to pass ``use_sim_time:=True`` to all launch scripts, and to let the ros2 bag player drive the clock:
```
ros2 bag play --clock-topics-all my_bag/
```

## Trouble Shooting

### Nothing happens

- Check that the topics match. ``ros2 node info`` is your friend.
- If you have multiple cameras running, check that they are synchronized, i.e. that the ``header.stamp`` time stamps match between cameras. If they don't match, use an pproximate synchronizer.
- Is ``use_sim_time`` set consistently across all nodes?

### Jerky motions
- Check for image quality and that the tag detector works as it should. Use the apriltag_detector and in particular ``apriltag_draw`` from [this repo](https://github.com/ros-misc-utilities/apriltag_detector), which is available as installable apt package under ROS2.

### Large reprojection errors and SUBGRAPH ERROR warnings

- poor image quality
- bad calibration file
- bad camera pose file
- wrong tag size
- wrong tag pose specified



## License

This software and any future contributions to it are licensed under
the [Apache License 2.0](LICENSE).
