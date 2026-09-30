# hikrobot_camera

基于海康机器人 MVS SDK 的 ROS 2 相机功能包：连接海康相机、采集图像，并以标准 `sensor_msgs/msg/Image` 消息发布到话题，供 rviz2 等 ROS 2 工具订阅显示。

## 功能特性

- 按序列号（或 IP）**发现、选择并打开相机**，支持网口与 USB 相机
- **采集图像并发布**；彩色相机的 Bayer 原始格式会在内部自动转换为 RGB8
- 专用抓图线程**逐帧取图**，不丢帧，帧率预设 100 Hz 时实测约 93 Hz
- 通过 ROS 2 参数动态调节**曝光、增益、帧率、像素格式**，越界自动校验
- **断线自动检测与重连**，退出时自动释放资源
- **错误反馈**（设备不存在、设备被占用等）

## 项目结构

```
ROS2-camera-lxy/                        # colcon 工作空间
├── src/hikrobot_camera/                # ROS 2 功能包
│   ├── CMakeLists.txt                  # 构建配置
│   ├── package.xml                     # 包信息与依赖
│   ├── cmake/FindMVS.cmake             # MVS SDK 查找模块
│   ├── include/hikrobot_camera/        # 头文件（类声明）
│   │   ├── camera_node.hpp             #   CameraNode（ROS 节点层）
│   │   └── mvs_camera.hpp              #   MvsCamera（SDK 封装层）、Frame 结构体
│   ├── src/                            # 源文件（实现）
│   │   ├── main.cpp                    #   入口
│   │   ├── camera_node.cpp             #   CameraNode 实现
│   │   └── mvs_camera.cpp              #   MvsCamera 实现
│   ├── launch/camera.launch.py         # 启动文件
│   ├── config/camera.yaml              # 参数配置
│   └── test/                           # 测试（占位）
├── docs/                               # 文档
├── build/  install/  log/              # 构建产物（.gitignore 忽略）
└── README.md                           # 本文件
```

### 分层设计

| 层 | 类 | 职责 |
|---|---|---|
| ROS 节点层 | `CameraNode` | 话题发布、参数管理、定时器与抓图线程、资源生命周期 |
| SDK 封装层 | `MvsCamera` | 设备枚举与连接、图像采集（含 Bayer→RGB 转换）、参数读写 |

`CameraNode` 仅通过 `MvsCamera` 的公开接口操作相机，不直接依赖 SDK 类型，便于替换底层驱动。

## 环境依赖

- Ubuntu 22.04 / ROS 2 Humble
- 海康机器人 MVS SDK（Linux 版，默认安装于 `/opt/MVS`）

### SDK 环境变量

运行前需设置以下环境变量（建议写入 `~/.zshrc` 或 `~/.bashrc` 持久化）：

```bash
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
```

> 注意：海康 SDK 自带的 `set_env_path.sh` 是 bash 脚本，在 zsh 下无法直接 `source`，需手动导出上述变量。

## 编译

```bash
source /opt/ros/humble/setup.zsh   # bash 用户换成 setup.bash
colcon build --symlink-install --packages-select hikrobot_camera
```

## 运行

```bash
source install/setup.zsh
ros2 launch hikrobot_camera camera.launch.py
```

另开一个终端打开 rviz2：

```bash
rviz2
```

在 rviz2 中：**Add → By topic → 选 `/image_raw`（类型 `sensor_msgs/msg/Image`）→ Image → OK**，即可看到相机画面。

## 可配置参数

所有参数都集中在 `config/camera.yaml` 中配置（改完不用重新编译，用 `ros2 launch` 启动即生效），也可运行时用 `ros2 param set` 动态修改：

```yaml
/hikrobot_camera:
  ros__parameters:
    serial_number: "DA0734524"   # 目标相机序列号
    image_topic: "image_raw"     # 图像发布话题名
    exposure_time: 10000.0       # 曝光时间，微秒
    gain: 0.0                    # 增益，dB
    frame_rate: 100.0            # 帧率，fps
    pixel_format: 0x02180014     # 像素格式（RGB8）
```

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| `serial_number` | string | `DA0734524` | - | 目标相机序列号 |
| `image_topic` | string | `image_raw` | - | 图像发布话题名 |
| `exposure_time` | double | `10000` | 微秒 | 曝光时间（10ms，帧率上限 ~100fps） |
| `gain` | double | `0` | dB | 增益 |
| `frame_rate` | double | `100` | fps | 帧率 |
| `pixel_format` | int | `0x02180014` | 枚举 | 像素格式（RGB8） |

### 参数真实范围（由相机查询）

节点连接后会自动查询各参数的真实支持范围，越界设置会被拒绝并报告边界。以下为 `MV-CS016-10UC` 实测值：

| 参数 | 支持范围 |
|---|---|
| `exposure_time` | [15, 9999813] 微秒 |
| `gain` | [0.0, 17.0] dB |
| `frame_rate` | [0.1, 100000.0] fps |

示例：

```bash
ros2 param set /hikrobot_camera exposure_time 200000.0   # 调亮
ros2 param set /hikrobot_camera gain 10.0                # 加增益
ros2 param get /hikrobot_camera exposure_time            # 读回
```

## 注意事项

- **同一时刻只能一个程序独占相机**：多个节点同时打开同一相机会报"设备被占用"。
- **真彩色相机输出 Bayer 格式**：节点已自动将 BayerRG8 转换为 RGB8 后发布，rviz2 可直接显示。
- **曝光时间与帧率的权衡**：曝光越短帧率越高、画面越暗。默认 10ms（约 100fps）画面偏暗，可加大增益或补光；若想画面更亮，可增大曝光（如 20ms，帧率降到约 50fps）。
- 测试前可用 `pkill -f "lib/hikrobot_camera/camera_node"` 清理残留节点进程。

## 测试结果

- 图像发布帧率约 **93 Hz**（抓图线程逐帧取图，接近预设 100fps 上限；ai 分析受 Bayer→RGB 转换的 CPU 开销拖累）
- 断线重连：拔插 USB 线后，节点几秒后可自动检测并重连
