# RoboMaster assignment3 ROS2
这份仓库提供一个基础工程，供你在 Ubuntu 22.04 / ROS 2 Humble 上，基于海康机器人 MVS SDK 完成相机功能包。

目前只有最小节点和启动配置，连接相机、发布图像、参数设置及断线重连需要你完成。目录划分仅供参考，你可以根据需要调整。

## 开始

1. 点击 GitHub 页面右上角的 **Fork**，将仓库复制到你的账号下。
2. 在你的 Fork 页面点击 **Code**，复制地址并克隆到本地：

   ```bash
   # 将下面的地址替换为你的 Fork 地址
   git clone <你的 Fork 地址>
   cd robomaster-camera-assignment
   ```

3. 阅读 [ROS 2 教程](docs/ROS2Tutorial.md) 和 [作业要求](docs/assignment.md)，按下面的步骤构建并启动工程。
4. 在自己的仓库中完成开发，提交并推送改动，最后提交你的 GitHub 仓库链接。

[AGENTS.md](AGENTS.md) 用于约束 AI 助手的帮助范围：你可以用 AI 理解概念和分析问题，核心实现需要自己完成。

## 仓库结构

```text
robomaster-camera-assignment/          # 同时作为 colcon 工作空间
├── AGENTS.md                         # AI 助教规范
├── README.md
├── docs/ROS2Tutorial.md              # ROS 2 教程
├── docs/assignment.md                # 作业要求
└── src/hikrobot_camera/              # ROS 2 功能包
    ├── package.xml                   # 包信息与依赖
    ├── CMakeLists.txt                # 构建与安装配置
    ├── include/hikrobot_camera/
    │   └── camera_node.hpp          # 节点声明
    ├── src/
    │   ├── main.cpp                 # 程序入口
    │   └── camera_node.cpp          # 在这里开始实现
    ├── launch/camera.launch.py       # 启动文件
    ├── config/camera.yaml           # 参数配置
    ├── cmake/                       # 可按需添加 SDK 查找模块
    └── test/                        # 可按需添加测试
```

## 环境与依赖

先安装 ROS 2 Humble 与开发工具，确保 `ros2`、`colcon` 和 `rosdep` 可用。

工程目前没有接入 MVS SDK。你需要从 [海康机器人下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0) 下载适合系统架构的 SDK，阅读随附文档，并完成构建集成。ROS 和系统依赖可以通过 rosdep 安装，厂商 SDK 需要单独配置。

## 编译

在新终端中进入仓库根目录，运行：

```bash
source /opt/ros/humble/setup.bash
# 仅当系统尚未初始化 rosdep 时执行一次：sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select hikrobot_camera
```

本仓库本身就是工作空间，不需要再放到另一个工作空间的 `src` 中。如果你想使用已有工作空间，也可以只把 `src/hikrobot_camera` 放进去。

## 运行

另开终端，在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hikrobot_camera camera.launch.py
```

如果你使用 Zsh，将环境脚本的 `.bash` 换为 `.zsh`。

初始工程会输出 `Training scaffold only` 并保持运行，按 Ctrl+C 退出。此时尚未实现相机功能，没有图像话题是正常的。

你也可以指定自己的参数文件：

```bash
ros2 launch hikrobot_camera camera.launch.py params_file:=/absolute/path/to/camera.yaml
```

当前 YAML 只配置了 `use_sim_time`。相机相关参数需要你在代码中声明并实现后，再加入配置文件。

## 完成与提交

从 `camera_node.cpp` 的 TODO 开始，按 [作业要求](docs/assignment.md) 完成相机功能。你可以增加源文件或 SDK 封装类，并相应更新构建配置。

完成后：

- 更新 README，说明 SDK 及依赖的安装方式、如何编译启动、有哪些可配置参数。如果有未完成的功能或已知问题，简单注明即可。
- 将源代码、Launch 和参数配置推送到你的 Fork, 然后提交仓库链接到 2719850558@qq.com，格式为：第三次作业-班级-姓名（第三次作业-自动化2305-周湛昊）


---

# 第三次作业实现说明（周银夕）

## 实现功能

- **设备选择与连接**：枚举 GigE + USB3 设备，支持按序列号选择（`serial_number` 留空连接枚举到的第一台）；启动时回读并打印分辨率、曝光范围、增益、实际帧率；
- **图像采集与发布**：独立取流线程，以 `sensor_msgs/msg/Image` 发布到 `/image_raw`，默认 BEST_EFFORT QoS；
- **参数查看与修改**：曝光/增益/帧率声明为带描述与取值范围的 ROS 参数，支持 `ros2 param get/describe` 查看，支持运行时 `ros2 param set` 立即下发到相机；
- **断线重连与资源清理**：取流失败或掉线后自动释放句柄并按周期重连，重新插入自动恢复取流；退出时停线程、停取流、关设备、销毁句柄。

实测硬件：Hikrobot MV-CA016-10UC（USB3，1440×1080 @ 30 fps）。

## 依赖安装

1. ROS 2 Humble 与 ros-dev-tools（参考 docs/ROS2Tutorial.md）
2. 海康机器人 MVS SDK for Linux x86_64：从[海康机器人下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0)下载，解压后 `sudo ./setup.sh` 安装到 `/opt/MVS`，再执行 `sudo ldconfig`
3. ROS 依赖：`rosdep install --from-paths src --ignore-src -r -y --rosdistro humble`

## 编译与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hikrobot_camera
source install/setup.bash
ros2 launch hikrobot_camera camera.launch.py
# 或指定自己的参数文件
ros2 launch hikrobot_camera camera.launch.py params_file:=/absolute/path/to/camera.yaml
```

## 可配置参数（config/camera.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| serial_number | "" | 相机序列号，空 = 第一台（重启生效） |
| image_topic | /image_raw | 图像话题（重启生效） |
| frame_id | camera | 图像消息坐标系 |
| pixel_format | Mono8 | 像素格式（重启生效） |
| exposure_time | 10000.0 | 曝光时间（us），运行时可调 |
| gain | 0.0 | 增益（dB），运行时可调 |
| frame_rate | 30.0 | 采集帧率（fps），运行时可调 |
| grab_timeout_ms | 1000 | 单次取流超时 |
| reconnect_interval | 1.0 | 断线重连间隔（s） |
| image_qos_reliable | false | 图像 QoS：true=RELIABLE（重启生效） |

运行时调参示例（double 类型参数必须带小数点）：

```bash
ros2 param set /hikrobot_camera exposure_time 30000.0
ros2 param get /hikrobot_camera gain
ros2 param describe /hikrobot_camera frame_rate
ros2 param dump /hikrobot_camera    # 调好后可导回 yaml 固化
```

## 已知问题

- 暂不做 Bayer→RGB 转换，彩色相机以 Mono8 或 Bayer 原始格式发布（rviz2 均可直接显示）；
- 图像为 BEST_EFFORT QoS：rviz2 的 Image 显示需将 Reliability Policy 设为 Best Effort（或将 `image_qos_reliable` 设为 true 后重启）；
- `ros2 topic hz` 对 BEST_EFFORT 的大图像会低估帧率，实际流畅度以 rviz2 为准。
