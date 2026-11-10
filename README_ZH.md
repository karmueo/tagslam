# TagSLAM：基于标记的 SLAM

TagSLAM 是一个基于 ROS 2 的软件包，使用 AprilTag 人工标记进行同步定位与建图。

ROS 2 版本仍在开发中，文档将陆续补充。

## 支持的平台

目前 TagSLAM 需要 ROS 2 Rolling、Jazzy 或更新版本。较早的 ROS 2 版本缺少一些重要的 rosbag2 功能。

## 构建

构建过程遵循 ROS 2 的标准流程。设置以下 shell 变量：

```bash
repo=tagslam
url=https://github.com/berndpfrommer/${repo}.git
```

然后按照[此处](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)的 ROS 2 构建说明操作，但需要调整说明以编译 `ros2` 分支。

完成后，务必加载工作空间的 `install/setup.bash`。

## 使用方法

### 同步与检测

TagSLAM 使用标记数据以及可选的里程计消息。在使用多个相机或里程计的场景中，所有传感器必须同步，即传感器数据的 ROS `header.stamp` 字段相匹配。

`sync_and_detect` 对多个相机运行 AprilTag 检测器，输出包含解码标记的同步消息，供 TagSLAM 使用。它也可以处理里程计：除了与相机图像大致同时到达的消息外，其他里程计消息都会被丢弃；保留消息的 `header.stamp` 会被修改为与图像消息完全一致。

可以从 bag 文件运行 `sync_and_detect` 并将检测到的标记写入另一个 bag，也可以将其作为独立的可组合节点运行。程序通过 `cameras.yaml` 确定输入 bag 的读取话题、图像传输方式（原始或压缩）、标记检测器（MIT 或 UMich）以及标记输出话题，并在 `tagslam.yaml` 中查找配置了里程计话题的物体。

从 bag 运行：

```bash
ros2 run tagslam sync_and_detect_from_bag --ros-args -p "cameras:=./cameras.yaml" -p "tagslam_config:=./tagslam.yaml" -p "in_bag:=name_of_input_bag" -p "out_bag:=./tag_bag"
```

在线运行时，按以下方式启动 `sync_and_detect` 节点：

```bash
ros2 launch tagslam sync_and_detect.launch.py use_sim_time:=<True/False> cameras:=<path_to_cameras.yaml_file> tagslam_config:=<path_to_tagslam_config_file> use_approximate_sync:=<True/False>
```

### TagSLAM

TagSLAM 可以处理 rosbag，也可以作为节点运行。处理 bag 时，如果 rosbag 中只有图像话题而没有标记话题，TagSLAM 会自动识别并启动 `sync_and_detect` 进行标记检测。

从 rosbag 运行 TagSLAM：

```bash
ros2 run tagslam tagslam_from_bag --ros-args -p "cameras:=./cameras.yaml" -p "tagslam_config:=./tagslam.yaml" -p "camera_poses:=./camera_poses.yaml" -p "in_bag:=./bag_with_tags_and_odom" -p "out_bag:=./out_bag"
```

在线运行时，按以下方式启动 `tagslam` 节点：

```bash
ros2 launch tagslam tagslam.launch.py use_sim_time:=<True/False> cameras:=<path_to_cameras.yaml_file> camera_poses:=<path_to_camera_poses.yaml file> tagslam_config:=<path_to_tagslam_config_file> use_approximate_sync:=<True/False>
```

要查看 TagSLAM 输出的 bag，请加载 ROS 和工作空间环境，然后在图形会话中启动查看器：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/workspace/install/setup.bash
ros2 launch tagslam view_bag.launch.py in_bag:=/absolute/path/to/out_bag
# 可选：调整播放速度和轨迹分段间隔（秒）。
ros2 launch tagslam view_bag.launch.py in_bag:=/absolute/path/to/out_bag rate:=1.0 max_gap:=0.03
```

默认以半速播放，并等待 RViz 订阅。相邻位姿时间戳间隔超过 `max_gap` 秒时，轨迹会显示为独立的段；孤立位姿显示为点。默认间隔为 0.03 秒。播放结束后 RViz 保持打开；关闭 RViz 或按 Ctrl+C 会停止启动的进程。如果关闭时 RViz 询问是否保存配置，请选择 **Discard**，以免修改已安装的配置。

默认情况下，位姿估计要求一个同步帧的所有相机中至少有三个可用且不同的标记 ID。多个相机看到的同一 ID 只计算一次。计数前会应用汉明距离、squash 和 ID 重映射规则；因配置的最小面积或物体关联规则而被拒绝的标记不计入。可以添加到配置的默认物体中的标记，即使尚未建图也会计入。低于阈值的帧及其里程计会被跳过，这可能导致轨迹和动态变换出现空缺。标记检测输出不受影响。

启动时可通过 ROS 参数 `minimum_tags_for_pose` 修改阈值，设为 `0` 则禁用此检查并恢复此前的行为。负值无效。处理 bag 时，在 `--ros-args` 后添加 `-p minimum_tags_for_pose:=3`；在线运行时，在启动命令中添加 `minimum_tags_for_pose:=3`。最终的帧统计会分别报告输入帧、通过阈值检查的帧、跳过的帧和已求解的帧数。

### 从双目 MCAP 估计与帧对齐的位姿

对于包含左右并排双目 `CompressedImage` 话题的 MCAP 文件，加载 ROS 和本工作空间后，运行完整的离线流程：

```bash
ros2 launch tagslam estimate_from_mcap.launch.py \
  in_bag:=/absolute/path/to/job_004.mcap
```

默认读取 MCAP 同目录下的 `calib.yaml`；可通过 `calib:=/absolute/path/to/calib.yaml` 指定其他文件。`T_cam1_cam0` 中的标定平移量以毫米为单位。两个相机必须具有相同的分辨率和鱼眼畸变模型。默认话题为 `/stereo/image_raw`；需要时可设置 `image_topic:=/other/topic`。安装的配置使用边长 144 毫米的 AprilTag、标记 0 作为地图原点，并以左相机作为相机组原点。此 MCAP 流程使用双目图像。设置 `minimum_tags_for_pose:=0` 可禁用过滤。帧即使通过阈值检查，也可能没有求解出位姿。

默认在 `./tagslam_runs/` 下创建新目录；可通过 `output_dir:=/absolute/new/directory` 指定位置。程序会拒绝已存在的输出目录。成功后，该目录包含 `trajectory.txt`、`out_bag/`、实际使用的 `config/`、`frame_index.csv` 和 `run_summary.json`；各阶段日志为 `prepare.log`、`tagslam.log` 和 `export.log`。成功后会删除临时拆分的双目 bag；设置 `keep_intermediate:=true` 可保留。失败时，输出目录和中间数据会保留以供诊断。

轨迹采用 TUM 格式，按输入帧顺序为每个有效位姿输出一行，以空格分隔，无表头：

```text
timestamp tx ty tz qx qy qz qw
```

时间戳是原始图像头的时间，以秒为单位，保留九位小数，可用于将位姿匹配到输入帧。每个位姿都是最终优化后的 `map` 到 `rig` 位姿，位置以米为单位，四元数按 `x,y,z,w` 排列。没有求解出位姿的帧会被省略，不进行插值，因此 TXT 行数等于有效位姿数。如果所有位姿均缺失，TXT 文件为空。单独的 `frame_index.csv` 保留每个输入帧、从零开始的索引、整数纳秒表示的原始头时间戳以及 MCAP 记录时间戳。通过阈值检查的帧也可能没有求解出位姿。

图像时间戳重复或递减，以及输出位姿时间戳重复或无法匹配时，程序会报错，避免产生有歧义的对齐结果。可按上述方法使用 `view_bag.launch.py` 查看结果 bag。

### 比较 TagSLAM 和 OpenVINS 轨迹 CSV

独立的 Python 3 工具依赖 NumPy、SciPy、PyYAML 和 Matplotlib，无需 ROS 运行环境。构建后也可以使用相同参数通过 `ros2 run tagslam compare_trajectories.py` 运行。该工具只接受轨迹 CSV，不能直接读取 `estimate_from_mcap.launch.py` 输出的 TUM TXT；此前生成的 CSV 仍可使用。

```bash
python3 src/python/compare_trajectories.py \
  --reference /data/scl_data/slam/stereo_live_bags/EGO/400hz/tagslam_output/trajectory.csv \
  --estimate /data/scl_data/slam/stereo_live_bags/EGO/400hz/openvins_output/job_004.csv \
  --calib /data/scl_data/slam/stereo_live_bags/EGO/400hz/calib.yaml \
  --output-dir /tmp/tagslam_trajectory_comparison_job004
```

两个 CSV 都必须包含 `timestamp_ns`、`pose_valid`、`x,y,z,qx,qy,qz,qw`。位姿表示从物体坐标系到世界坐标系的变换，位置以米为单位，四元数按 `x,y,z,w` 排列。TagSLAM 作为参考而非真值，其中 `rig=cam0`；OpenVINS 位姿描述 IMU。工具将 OpenVINS 位姿右乘标定 YAML 中 `cam0.T_cam_imu` 的逆，转换为 cam0 位姿；该标定平移量以毫米为单位。工具不会再次应用标定中的 IMU 时间偏移，也不会估计时间偏移或尺度。

整数纳秒时间戳保留完整精度。工具会过滤无效位姿、归一化四元数、对时间戳排序，并在时间戳重复时保留最后一个有效行。仅有无效位姿的时间戳会使轨迹分段。在重叠的参考时间戳处，估计位置采用线性插值，方向采用最短弧球面线性插值（SLERP）。不允许外推或跨断点插值。超过 `--max-gap-s`（默认为 `0.1`）的采样间隔也会使相应轨迹分段。缺失或格式错误的整数时间戳会导致明确报错，因为无法确定断点位置。

所有匹配位置共同确定一个最小二乘刚体对齐，不进行尺度校正；其旋转也用于变换方向。匹配数少于三个或位置几何退化（静止或共线轨迹）时会报错。位置 RMSE 是三维位置误差范数的均方根；旋转 RMSE 使用以度为单位的最短相对旋转角。每个匹配样本权重相同。此外还会报告平均值、中位数、P95 和最大值。

一米漂移使用参考轨迹折线上的不重叠区间计算，并在精确的整米边界对两条轨迹进行插值。对每个区间，`delta = inverse(T_start) * T_end`，`E = inverse(delta_reference) * delta_estimate`。平移漂移为 `100 * norm(translation(E)) / 1m` 百分比；旋转漂移为 `rotation(E)` 的角度，以度/米为单位。每个断点处重新开始计量；不完整的尾段会被省略，并报告其总距离。如果没有完整区间，漂移结果不可用（JSON 中为 `null`），但仍会报告绝对误差。

输出目录包含 `summary.json`（输入、约定、变换、过滤计数和统计信息）、`aligned_trajectory.csv`（匹配的参考位姿和对齐后的估计位姿、分段 ID 及误差）、`drift_1m.csv`（每个完整区间一行），以及 `trajectory_comparison.png`、`pose_errors.png`、`drift_1m.png`。绘图无需图形桌面。使用同一输出目录重新运行会替换这些生成的文件。在漂移 CSV 中，每个 `segment_id` 的距离从零重新开始；时间戳列用于定位原始数据中的轨迹段。

### Rosbag

播放 ROS 2 bag 时，必须向所有启动脚本传入 `use_sim_time:=True`，并让 ROS 2 bag 播放器驱动时钟：

```bash
ros2 bag play --clock-topics-all my_bag/
```

## 故障排查

### 没有任何反应

- 检查话题是否匹配。可以使用 `ros2 node info` 查看。
- 如果运行多个相机，请检查它们是否同步，即各相机的 `header.stamp` 时间戳是否匹配。如果不匹配，请使用近似同步器。
- 所有节点的 `use_sim_time` 设置是否一致？

### 运动不平滑

- 检查图像质量以及标记检测器是否正常工作。使用 [此仓库](https://github.com/ros-misc-utilities/apriltag_detector)中的 apriltag_detector，特别是 `apriltag_draw`；它可以作为 ROS 2 的 apt 包安装。

### 重投影误差较大并出现 SUBGRAPH ERROR 警告

- 图像质量差
- 标定文件有误
- 相机位姿文件有误
- 标记尺寸错误
- 指定的标记位姿错误

## 许可证

本软件及其未来的所有贡献均采用 [Apache License 2.0](LICENSE) 授权。
