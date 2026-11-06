# 仓库指南

## 项目结构

TagSLAM 是用于基于人工标记进行 SLAM 的 ROS 2 包。C++ 实现位于 `src/`，公开头文件位于 `include/tagslam/`，GTSAM 畸变辅助代码位于 `include/gtsam_distortion/` 和 `src/gtsam_distortion/`。`factor/`、`measurements/` 和 `value/` 子目录按功能组织相关组件。Python 工具位于 `src/python/`，ROS 启动文件位于 `launch/`，示例标定配置和 RViz 资源位于 `example/` 和 `config/`。

## 构建与运行

使用 ROS 2 Jazzy、Rolling 或更新版本。在已加载 ROS 2 环境的工作空间根目录执行以下命令，假设本仓库位于 `src/tagslam`：

```bash
vcs import src < src/tagslam/tagslam.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to tagslam
source install/setup.bash
```

上述命令依次导入源码依赖、安装系统依赖、构建本包及其工作空间依赖，并加载构建结果。使用 `ros2 launch tagslam tagslam.launch.py` 运行在线节点，并根据实际配置提供 `cameras`、`camera_poses` 和 `tagslam_config` 启动参数。数据包处理命令及同步要求参见 `README.md`。

## 代码风格

使用 C++17 和仓库中的 `.clang-format` 配置：基于 Google 风格，使用两个空格缩进，每行最多 80 列。使用 `clang-format -i <file>` 格式化修改过的 C++ 文件。遵循现有命名方式：文件名使用 `snake_case`，类名使用 `PascalCase`，私有成员以下划线结尾。Python 启动文件使用四个空格缩进。构建时编译器警告会被视为错误。

## 测试

在工作空间根目录先运行 `colcon test --packages-select tagslam`，再运行 `colcon test-result --verbose` 查看结果。已配置的检查包括版权、cppcheck、cpplint、clang-format、CMake 和 XML 检查。目前没有注册单元测试，也没有覆盖率阈值；`src/rpp_test.cpp` 是独立的调试程序。新增行为时，请添加针对性测试（例如 `test/test_<feature>.cpp`），并使用 `ament_cmake_gtest` 在 CMake 中注册。

## 提交与拉取请求

近期提交标题通常使用简短、小写的英文祈使句，例如 `fix linter errors`。按照 `CONTRIBUTING.md` 的要求，使用 `git commit -s` 为每次提交添加签署信息；贡献内容采用 Apache 2.0 许可证。提交拉取请求时，说明行为变化和动机，关联相关议题，并列出执行过的构建和测试。必要时附上具有代表性的配置或输出。CI 会检查以 `ros2` 为目标分支的拉取请求。
