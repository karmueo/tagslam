# TagSLAM：基于标记的 SLAM

TagSLAM 使用 AprilTag 进行同步定位与建图，支持离线 bag 和在线节点。ROS 2 版本仍在开发中，需要 Jazzy、Rolling 或更新版本。多相机和里程计数据须具有匹配的 ROS `header.stamp`；时间不完全一致时使用近似同步。

下文使用 Bash。构建从已加载 ROS 环境、且仓库位于 `src/tagslam`（`ros2` 分支）的工作空间根目录开始；运行前加载工作空间环境，准备相机标定 `cameras.yaml`、物体配置 `tagslam.yaml` 和需要时的 `camera_poses.yaml`。相对路径均相对于执行目录；`/absolute/path/`、`/path/to/` 为须替换的路径示例。

## 构建

以下步骤依据[仓库指南](AGENTS.md#构建与运行)，需具备安装系统依赖的权限。

```bash
# 导入工作空间源码依赖
vcs import \
  src \
  < src/tagslam/tagslam.repos
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `src`（位置参数） | 源码检出位置 | 输出目录 | 本流程显式指定工作空间的 `src` |
| `<`（Shell 重定向） | 从依赖清单读取标准输入 | 输入文件 | 本流程为 `src/tagslam/tagslam.repos`；依赖版本见[清单](tagslam.repos) |

```bash
# 安装工作空间缺少的系统依赖
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--from-paths` | 扫描包目录 | 输入目录 | 本流程为 `src` |
| `--ignore-src` | 跳过源码中已有的依赖包 | 行为控制项 | 未指定时不启用 |
| `-r` | 遇到错误继续处理其他依赖 | 行为控制项 | 未指定时不启用；结束后仍须处理报错 |
| `-y` | 自动同意安装提示 | 行为控制项 | 未指定时不启用 |

```bash
# 构建 TagSLAM 及其工作空间依赖
colcon build \
  --packages-up-to tagslam
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--packages-up-to` | 选择目标包及递归依赖 | 输入值 | 本流程为 `tagslam`；省略时不按此条件筛选包 |

```bash
# 加载构建后的工作空间环境
source \
  install/setup.bash
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 脚本路径（位置参数） | 将环境设置加载到当前终端 | 输入文件 | 必填，无默认值；新终端须先加载 ROS 环境，再加载此文件 |

## 使用方法

### 同步与检测

`cameras.yaml` 指定图像话题、传输方式、检测器和标记话题；`tagslam.yaml` 指定物体的里程计话题。近似同步只保留与图像匹配的里程计，将其头时间戳改为图像时间，其他里程计消息丢弃。

#### 离线检测

```bash
# 将输入 bag 的同步标记和里程计写入新 bag
ros2 run tagslam sync_and_detect_from_bag \
  --ros-args \
  -p "cameras:=./cameras.yaml" \
  -p "tagslam_config:=./tagslam.yaml" \
  -p "in_bag:=./input_bag" \
  -p "out_bag:=./tag_bag"
```

参数依据：[离线入口](src/sync_and_detect_from_bag.cpp)、[同步节点](src/sync_and_detect.cpp)。输出路径应使用新的 bag 路径。以下为项目参数；ROS 参数通过 `--ros-args` 后重复的 `-p 名称:=值` 传入。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--ros-args` | 开始 ROS 参数区 | 行为控制项 | 使用 `-p` 时指定 |
| `-p` / `--param` | 设置 ROS 参数，可重复 | 输入值 | 格式：`名称:=值` |
| `cameras` | 相机和检测配置 | 输入文件 | 默认空字符串；必须指定有效 YAML |
| `tagslam_config` | 物体和里程计配置 | 输入文件 | 默认空字符串；必须指定有效 YAML |
| `in_bag` | 输入 bag | 输入路径 | 默认空字符串；必须指定 |
| `out_bag` | 检测结果 bag | 输出路径 | 默认空字符串：只发布消息，不录包 |
| `max_number_of_frames` | 限制同步帧数 | 行为控制项 | 默认 `0`：不限；正整数；当前入口在计数大于该值后退出 |
| `use_approximate_sync` | 使用近似同步 | 行为控制项 | 默认 `true`；支持布尔值 |
| `publish` | 发布标记和同步里程计 | 行为控制项 | 默认 `true`；录包时保持开启 |
| `<图像话题>.image_transport` | 指定图像传输方式 | 输入值 | 默认取相机配置的 `image_transport`，未配置时为 `raw`；压缩输入可使用 `compressed` |

#### 在线检测

```bash
# 启动同步检测节点
ros2 launch tagslam sync_and_detect.launch.py \
  use_sim_time:=False \
  cameras:=./cameras.yaml \
  tagslam_config:=./tagslam.yaml \
  use_approximate_sync:=True
```

参数依据：[启动文件](launch/sync_and_detect.launch.py)。回放外部 bag 时改用 `use_sim_time:=True`，并按 [Rosbag](#rosbag) 启动播放器。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `cameras` | 相机配置 | 输入文件 | 默认 `cameras.yaml` |
| `tagslam_config` | 物体和里程计配置 | 输入文件 | 默认 `tagslam.yaml` |
| `use_sim_time` | 使用 ROS 仿真时钟 | 行为控制项 | 默认 `False`；支持 `True` / `False` |
| `use_approximate_sync` | 使用近似同步 | 行为控制项 | 默认 `True`；支持 `True` / `False` |

### TagSLAM

默认每个同步帧至少需要一个不同的可用标记 ID，多个相机看到同一 ID 只计一次。计数应用汉明距离、squash、ID 重映射、最小面积及物体关联规则；可加入默认物体的未建图标记也计入。低于阈值的帧及其里程计被跳过，可能造成轨迹空缺；检测输出不受影响。`minimum_tags_for_pose:=0` 禁用此检查，通过检查也不保证能求解位姿。结束统计分别报告输入、通过检查、跳过和已求解的帧数。

#### 离线估计

**输出警告：** `outbag` 指定的最终优化结果 bag 若已存在，会被删除并重建；请使用独立输出路径。`out_bag` 是另一个参数，用于录制发布消息，不能代替 `outbag`。依据：[输出实现](src/tagslam.cpp)、[离线入口](src/tagslam_from_bag.cpp)。

```bash
# 从标记或原始图像 bag 估计位姿并保存结果
ros2 run tagslam tagslam_from_bag \
  --ros-args \
  -p "cameras:=./cameras.yaml" \
  -p "tagslam_config:=./tagslam.yaml" \
  -p "camera_poses:=./camera_poses.yaml" \
  -p "in_bag:=./bag_with_tags_and_odom" \
  -p "out_bag:=./out_bag" \
  -p "outbag:=./optimized_bag" \
  -p max_number_of_frames:=2147483647
```

bag 缺少标记话题但包含所需图像时，会自动启动检测。当前离线入口中，帧数上限默认 `0` 会使播放循环首次迭代后退出；示例显式设为 32 位有符号整数上限，供通常的完整 bag 处理。总帧数须小于该上限，才能在读完 bag 后进入最终处理。依据：[循环与结束条件](src/tagslam_from_bag.cpp)。

ROS 参数写法见[离线检测](#离线检测)。下表列出离线入口及 [TagSLAM 节点](src/tagslam.cpp) 可核实的项目参数。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `cameras` | 相机配置 | 输入文件 | 默认空字符串；必须指定有效 YAML |
| `tagslam_config` | SLAM 和物体配置 | 输入文件 | 默认空字符串；必须指定有效 YAML |
| `camera_poses` | 相机位姿配置 | 输入文件 | 默认空字符串：不读取外部相机位姿文件 |
| `in_bag` | 输入 bag | 输入路径 | 默认空字符串；必须指定 |
| `out_bag` | 录制节点发布的消息 | 输出路径 | 默认空字符串：不录制 |
| `outbag` | 最终优化结果 bag | 输出路径 | 默认 `out.bag`；已有路径会被替换 |
| `output_directory` | 保存位姿、标定等结果 | 输出目录 | 默认 `.` |
| `max_number_of_frames` | 设置处理帧数上限 | 行为控制项 | 默认 `0`；此离线入口须显式设置；示例为 `2147483647` |
| `minimum_tags_for_pose` | 设置每帧最少可用标记数 | 行为控制项 | 默认 `1`；非负整数，`0` 禁用；仅启动时设置 |
| `playback_rate` | 最终重播速率 | 行为控制项 | 默认 `5.0`；数值 |
| `fixed_frame_id` | 固定坐标系名称 | 输入值 | 默认 `map`；字符串 |
| `publish_ack` | 发布处理确认消息 | 行为控制项 | 默认 `false`；布尔值 |

#### 在线估计

```bash
# 启动 TagSLAM 节点
ros2 launch tagslam tagslam.launch.py \
  use_sim_time:=False \
  cameras:=./cameras.yaml \
  camera_poses:=./camera_poses.yaml \
  tagslam_config:=./tagslam.yaml \
  use_approximate_sync:=True \
  minimum_tags_for_pose:=1
```

参数依据：[启动文件](launch/tagslam.launch.py)。需要至少三个可用标记时设为 `minimum_tags_for_pose:=3`。回放外部 bag 时设为 `use_sim_time:=True`，参见 [Rosbag](#rosbag)。无输出时核对配置话题、各传感器时间戳和所有节点的时钟设置；运动不平滑时检查图像质量和标记检测，可用 [apriltag_detector](https://github.com/ros-misc-utilities/apriltag_detector) 的 `apriltag_draw` 检查检测结果；重投影误差大或出现 `SUBGRAPH ERROR` 时核对图像质量、标定、相机位姿、标记尺寸和标记位姿。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `cameras` | 相机配置 | 输入文件 | 默认 `cameras.yaml` |
| `camera_poses` | 相机位姿配置 | 输入文件 | 默认 `camera_poses.yaml` |
| `tagslam_config` | SLAM 和物体配置 | 输入文件 | 默认 `tagslam.yaml` |
| `minimum_tags_for_pose` | 每帧最少可用标记数 | 行为控制项 | 默认 `1`；非负整数，`0` 禁用 |
| `use_sim_time` | 使用 ROS 仿真时钟 | 行为控制项 | 默认 `False`；支持 `True` / `False` |
| `use_approximate_sync` | 使用近似同步 | 行为控制项 | 默认 `True`；支持 `True` / `False` |

### 从双目 MCAP 估计与帧对齐的位姿

输入须为包含左右并排双目 `CompressedImage` 的 MCAP。两相机分辨率相同且使用鱼眼模型；标定文件的 `T_cam1_cam0` 平移量以毫米计。安装配置使用边长 144 毫米的 AprilTag、标记 0 作为地图原点、左相机作为相机组原点。依据：[准备脚本](src/python/estimate_from_mcap.py)、[配置模板](config/mcap_stereo_tagslam.yaml)。

**输出警告：** 显式传入 `output_dir` 时，已有目录会被递归删除并重建；不得指向需要保留的数据目录。程序拒绝替换包含当前工作目录、输入 MCAP 或标定文件的目录，并会解析符号链接检查实际输入路径。省略该参数时自动创建带时间戳的新目录。依据：[当前启动文件](launch/estimate_from_mcap.launch.py)。

```bash
# 从双目 MCAP 生成与输入帧对齐的最终轨迹
ros2 launch tagslam estimate_from_mcap.launch.py \
  in_bag:=/absolute/path/to/job_004.mcap
```

在已加载 ROS 和工作空间环境的终端执行。成功后输出目录包含 `trajectory.txt`、`out_bag/`、`config/`、`frame_index.csv`、`run_summary.json`，阶段日志为 `prepare.log`、`tagslam.log`、`export.log`；失败时保留输出及中间数据供诊断。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `in_bag` | 原始双目 MCAP | 输入文件 | 必填，无默认值；须为存在的 `.mcap` 文件 |
| `calib` | 双目标定 | 输入文件 | 默认输入 MCAP 同目录的 `calib.yaml` |
| `output_dir` | 保存整次运行结果 | 输出目录 | 默认自动生成 `./tagslam_runs/<MCAP文件名去扩展名>_<年月日_时分秒_微秒>/`；显式指定时替换已有目录 |
| `image_topic` | 双目压缩图像话题 | 输入值 | 默认 `/stereo/image_raw`；以 `/` 开头 |
| `vio_odom_topic` | IMU 坐标系 VIO 里程计话题 | 输入值 | 默认空字符串：不融合 VIO；指定时以 `/` 开头，类型为 `nav_msgs/msg/Odometry` |
| `minimum_tags_for_pose` | 每帧最少可用标记数 | 行为控制项 | 默认 `1`；范围 `0`～`2147483647`，`0` 禁用 |
| `keep_intermediate` | 成功后保留拆分双目 bag | 行为控制项 | 默认 `false`：成功后删除；支持 `true` / `false` |

融合 OpenVINS 时，里程计须在同一 MCAP 中、以 IMU 为跟踪坐标系，坐标系名称一致，头时间戳与双目图像匹配；标定须包含平移量以毫米计的 `cam0.T_cam_imu`。

```bash
# 融合同一 MCAP 中的 VIO 与 AprilTag 观测
ros2 launch tagslam estimate_from_mcap.launch.py \
  in_bag:=/absolute/path/to/output/job_004_openvins.mcap \
  calib:=/absolute/path/to/calib.yaml \
  vio_odom_topic:=/openvins/odom \
  minimum_tags_for_pose:=0
```

参数见上表。设为 `0` 后，无可用标记的同步帧也可使用 VIO。流程原样转发里程计，按标定将 IMU 转换到左相机坐标；融合输出为 `/tagslam/odom/body_rig`，父坐标系 `map`、子坐标系 `rig`。摘要记录里程计消息数与匹配帧数；未匹配里程计的图像仍保留在帧索引中。临时 bag 按头时间戳回放，帧索引保留原始 MCAP 记录时间。

`trajectory.txt` 无表头，每个有效位姿一行：`timestamp tx ty tz qx qy qz qw`。时间为原始图像头时间（秒，九位小数），位姿为最终优化后的 `map` 到 `rig`，位置单位为米。未求解帧省略且不插值；全部缺失时文件为空。`frame_index.csv` 保留所有输入帧、从零开始的索引、整数纳秒头时间戳及 MCAP 记录时间。图像时间戳重复或递减、输出位姿时间戳重复或无法匹配时会报错。查看结果使用下一节命令。

### 查看输出 bag

新终端须先加载 ROS，再加载工作空间；以下为 Jazzy 示例。

```bash
# 加载 ROS Jazzy 环境
source \
  /opt/ros/jazzy/setup.bash

# 加载本工作空间环境
source \
  /path/to/workspace/install/setup.bash
```

`source` 参数见[构建](#构建)。查看器需要图形会话，输入为包含 `metadata.yaml` 的结果 bag 目录。

```bash
# 在 RViz 中回放结果并显示轨迹
ros2 launch tagslam view_bag.launch.py \
  in_bag:=/absolute/path/to/out_bag
```

参数依据：[查看器启动文件](launch/view_bag.launch.py)。默认半速播放并等待 RViz 订阅；超过 `max_gap` 秒的轨迹分段，孤立位姿显示为点。播放结束后 RViz 保持打开；关闭 RViz 或按 Ctrl+C 停止进程。RViz 询问保存配置时选 **Discard**，避免修改已安装配置。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `in_bag` | TagSLAM 结果 bag | 输入目录 | 必填，无默认值；目录须包含 `metadata.yaml` |
| `rate` | 播放速度倍率 | 行为控制项 | 默认 `0.5`；有限正数 |
| `max_gap` | 轨迹分段时间间隔 | 输入值 | 默认 `0.03` 秒；有限正数 |
| `DISPLAY` / `WAYLAND_DISPLAY`（环境变量） | 指定图形会话 | 输入值 | 依赖桌面环境，至少一个已设置 |

```bash
# 以正常速度回放，并使用 0.03 秒轨迹分段间隔
ros2 launch tagslam view_bag.launch.py \
  in_bag:=/absolute/path/to/out_bag \
  rate:=1.0 \
  max_gap:=0.03
```

### 比较 TagSLAM 和 OpenVINS 轨迹 CSV

从仓库根目录执行，需 Python 3、NumPy、SciPy、PyYAML 和 Matplotlib，无需 ROS 或图形桌面。工具只接受 CSV，**不能直接读取上述流程输出的 TUM TXT**；可使用此前已有的 CSV。构建后也可将脚本入口替换为 `ros2 run tagslam compare_trajectories.py`，参数相同。

两个 CSV 都须包含 `timestamp_ns`、`pose_valid`、`x,y,z,qx,qy,qz,qw`。时间为整数纳秒；位姿从物体到世界坐标系，位置以米计。TagSLAM 为参考（`rig=cam0`），OpenVINS 为 IMU 位姿；标定须包含 `cam0.T_cam_imu`，平移量以毫米计。

**输出警告：** 使用同一输出目录重跑会替换生成的比较文件。

```bash
# 比较参考轨迹与 OpenVINS 轨迹并保存统计和图表
python3 src/python/compare_trajectories.py \
  --reference /absolute/path/to/tagslam/trajectory.csv \
  --estimate /absolute/path/to/openvins/job_004.csv \
  --calib /absolute/path/to/calib.yaml \
  --output-dir /absolute/path/to/trajectory_comparison
```

参数依据：[比较脚本](src/python/compare_trajectories.py)。工具将 IMU 位姿转换为 cam0 位姿，进行刚体对齐，不估计尺度或时间偏移，也不重复应用标定时间偏移。需至少三个匹配点且轨迹不能静止或共线；不跨断点插值、不外推。无效位姿和超过 `--max-gap-s` 的间隔会分段；时间戳缺失或格式错误会报错。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--reference` | TagSLAM 参考轨迹 | 输入文件 | 必填，无默认值；上述 CSV 格式 |
| `--estimate` | OpenVINS 估计轨迹 | 输入文件 | 必填，无默认值；上述 CSV 格式 |
| `--calib` | IMU 与 cam0 外参 | 输入文件 | 必填，无默认值；YAML |
| `--output-dir` | 比较结果位置 | 输出目录 | 必填，无默认值；已有生成文件会被替换 |
| `--max-gap-s` | 允许插值的最大采样间隔 | 输入值 | 默认 `0.1` 秒；有限正数 |
| `-h` / `--help` | 显示帮助并退出 | 行为控制项 | 未指定时正常运行 |

输出包括 `summary.json`、`aligned_trajectory.csv`、`drift_1m.csv`、`trajectory_comparison.png`、`pose_errors.png` 和 `drift_1m.png`。TagSLAM 参考并非真值；误差按等权匹配样本统计。一米漂移在每个连续轨迹段内计算，不完整尾段省略；无完整一米区间时 JSON 漂移为 `null`，绝对误差仍可报告。漂移 CSV 的 `segment_id` 内距离从零开始，时间戳可定位原始数据。

### Rosbag

仅在外部播放器向在线节点供数时使用；所有相关节点均须设 `use_sim_time:=True`。

```bash
# 回放 bag 并按所有话题驱动 ROS 时钟
ros2 bag play \
  --clock-topics-all \
  my_bag/
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| bag 路径（位置参数） | 指定回放数据 | 输入路径 | 必填，无默认值；`my_bag/` 为示例 |
| `--clock-topics-all` | 按所有话题消息发布时钟 | 行为控制项 | 未指定时不启用此时钟模式 |

## 许可证

本软件及未来贡献采用 [Apache License 2.0](LICENSE)。
