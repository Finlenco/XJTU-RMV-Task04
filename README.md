# 海康威视相机 ROS2 驱动

基于 ROS2 Humble 与海康威视 MVS C++ SDK 的相机驱动。默认使用 `hik_camera_driver/config/camera_params.yaml` 作为参数来源，支持 RViz 可视化、帧率监控与参数信息打印。

## 功能特性

- ✅ **自动设备发现和连接**：支持通过IP地址或序列号连接相机
- ✅ **稳定图像采集**：高帧率图像数据采集和发布
- ✅ **参数动态配置**：支持曝光时间、增益、帧率、像素格式等参数调节
- ✅ **断线重连**：自动检测连接状态并重连
- ✅ **标准ROS2接口**：发布标准的 `sensor_msgs/msg/Image`消息
- ✅ **图像传输优化**：使用 `image_transport` 进行高效图像传输
- ✅ **RViz2可视化**：支持在RViz2中实时查看相机图像
- ✅ **实时帧率监控**：区分设定帧率和实际帧率
- ✅ **参数信息显示**：详细的参数说明和调优建议

## 代码结构

```
hik_camera_driver/
├── include/hik_camera_driver/
│   ├── hik_camera_driver.hpp      # 主驱动头文件
│   └── mvs_sdk_wrapper.hpp        # MVS SDK包装器
├── src/
│   ├── hik_camera_node.cpp        # 主相机节点
│   ├── fps_monitor_node.cpp       # 帧率监控节点
│   ├── param_info_node.cpp        # 参数信息节点
│   └── mvs_sdk_wrapper.cpp        # MVS SDK实现
├── launch/
│   └── hik_camera_system.launch.py # 完整系统启动文件
├── config/
│   ├── camera_params.yaml         # 参数配置文件
│   └── camera_display.rviz        # RViz配置文件
├── CMakeLists.txt                 # CMake配置
├── package.xml                    # 包清单
└── README.md                      # 说明文档
```

## 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- OpenCV 4.x
- 海康威视MVS C++ SDK

## 快速开始

1) 环境与 SDK 设置（一次性）

```bash
source /opt/ros/humble/setup.bash
echo 'export MVS_HOME=$HOME/opt/MVS' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$HOME/opt/MVS/lib/64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

```

2) 编译与启动

```bash
cd /home/fin/Code/RoboMaster/Task4/hik_camera_driver
colcon build --packages-select hik_camera_driver --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 launch hik_camera_driver hik_camera_system.launch.py use_rviz:=true monitor_fps:=true
```


3) 脚本启动（可选，便捷方式）

```bash
cd /home/fin/Code/RoboMaster/Task4/hik_camera_driver
chmod +x scripts/camera_manager.sh

# 使用 YAML 默认参数启动完整系统（等价于 launch 方法）
./scripts/camera_manager.sh start

# 仅启动，不打开 RViz 或帧率监控
./scripts/camera_manager.sh start --no-rviz --no-monitor

# 覆盖关键运行参数（命令行优先于 YAML）
./scripts/camera_manager.sh start \
  -s 00F26632041 \
  -i 192.168.1.100 \
  -t /image_raw \
  -f 30.0 \
  -e 2000.0 \
  -g 2.0 \
  -p bgr8
```

脚本参数说明：

- `-i, --ip`：相机 IP 地址（GigE）
- `-s, --serial`：相机序列号（USB）
- `-t, --topic`：图像话题名，默认 `/image_raw`
- `-f, --fps`：设定帧率（FPS）
- `-e, --exposure`：曝光时间（微秒）
- `-g, --gain`：增益
- `-p, --pixel`：像素格式（bgr8/rgb8/mono8 等）
- `--no-rviz`：不启动 RViz
- `--no-monitor`：不启动帧率监控

## 启动方式

- 推荐使用 Launch：`ros2 launch hik_camera_driver hik_camera_system.launch.py use_rviz:=true monitor_fps:=true`
- 所有运行参数默认来自 `config/camera_params.yaml`；需要临时覆盖时可用：

```bash
ros2 launch hik_camera_driver hik_camera_system.launch.py \
  --ros-args \
  -p hik_camera_driver.frame_rate:=30.0 \
  -p hik_camera_driver.exposure_time:=2000.0
```

## 系统功能详解

### 相机驱动节点 (hik_camera_driver)

- **功能**: 相机连接、图像采集、参数控制
- **话题**: 发布图像数据到指定话题
- **参数**: 支持动态参数调整
- **状态**: 显示连接状态和错误信息

### 帧率监控节点 (fps_monitor)

- **功能**: 实时监控实际帧率
- **输出**:
  - 平均帧率（从启动开始）
  - 当前帧率（基于最近几帧）
  - 最近帧率（最近1秒内）
- **输出**: 在日志中周期性打印平均/当前/最近帧率

### 参数信息节点 (param_info)

- **功能**: 显示系统参数和说明
- **输出**: 在日志中打印当前参数与说明

### RViz2 可视化

- **功能**: 图像显示和系统监控
- **配置**: 自动加载相机显示配置
- **用途**: 实时查看图像、调试系统

## 参数配置指南

### 连接参数

#### `camera_ip` (string, 默认: "")

- **功能**: 相机IP地址，用于网络相机连接
- **作用**: 指定要连接的相机的网络IP地址
- **使用场景**: GigE相机、网络相机
- **示例**: `"192.168.1.100"`
- **注意事项**:
  - 确保相机和主机在同一网段
  - 需要先配置相机的IP地址
  - 与camera_serial互斥，优先使用IP连接

#### `camera_serial` (string, 默认: "")

- **功能**: 相机序列号，用于USB相机连接
- **作用**: 通过序列号唯一标识相机设备
- **使用场景**: USB3.0相机、USB2.0相机
- **示例**: `"Vir81799215"`
- **注意事项**:
  - 序列号在相机标签上可以找到
  - 与camera_ip互斥，IP为空时使用序列号连接
  - 支持多相机时通过序列号区分

#### `auto_reconnect` (bool, 默认: true)

- **功能**: 是否启用自动重连机制
- **作用**: 网络断开或相机掉线时自动尝试重新连接
- **使用场景**: 网络不稳定环境、长时间运行
- **示例**: `true` / `false`
- **注意事项**:
  - 启用后会定期检查连接状态
  - 重连间隔由reconnect_interval控制

#### `reconnect_interval` (int, 默认: 5)

- **功能**: 自动重连间隔时间
- **作用**: 控制重连尝试的频率
- **单位**: 秒
- **示例**: `5` (5秒重连一次)
- **注意事项**:
  - 值太小会增加系统负载
  - 值太大可能错过重连时机
  - 建议范围: 3-10秒

### 相机参数

#### `frame_rate` (double, 默认: 30.0)

- **功能**: 设定帧率
- **作用**: 控制相机采集图像的频率
- **单位**: FPS (帧每秒)
- **示例**: `30.0` (30帧每秒)
- **注意事项**:
  - 受相机硬件限制
  - 高帧率需要更多带宽和计算资源
  - 实际帧率可能因网络或处理能力而降低
  - 常用值: 10, 15, 30, 60 FPS

#### `exposure_time` (double, 默认: 1000.0)

- **功能**: 曝光时间
- **作用**: 控制图像传感器的曝光时间，影响图像亮度
- **单位**: 微秒 (μs)
- **示例**: `1000.0` (1000微秒 = 1毫秒)
- **注意事项**:
  - 值越大图像越亮，但可能产生运动模糊
  - 值越小图像越暗，但运动模糊减少
  - 需要根据光照条件调整
  - 常用范围: 100-10000微秒

#### `gain` (double, 默认: 1.0)

- **功能**: 相机增益
- **作用**: 信号放大倍数，提高图像亮度
- **单位**: 倍数
- **示例**: `1.0` (无增益), `2.0` (2倍增益)
- **注意事项**:
  - 增益越高图像越亮，但噪声也增加
  - 建议优先调整曝光时间，再调整增益
  - 常用范围: 1.0-10.0
  - 高增益会降低图像质量

#### `pixel_format` (string, 默认: "bgr8")

- **功能**: 像素格式
- **作用**: 定义图像的颜色格式和编码方式
- **可选值**:
  - `"bgr8"`: 8位BGR彩色格式
  - `"rgb8"`: 8位RGB彩色格式
  - `"mono8"`: 8位灰度格式
  - `"mono16"`: 16位灰度格式
- **注意事项**:
  - 影响图像质量和数据大小
  - 彩色格式数据量是灰度的3倍
  - 16位格式提供更好的动态范围

### 图像参数

#### `topic_name` (string, 默认: "/image_raw")

- **功能**: 图像话题名称
- **作用**: 定义ROS2中图像数据的发布话题
- **示例**: `"/image_raw"`, `"/camera/image"`
- **注意事项**:
  - 必须是有效的ROS2话题名
  - 避免与其他话题冲突
  - 建议使用描述性名称

#### `image_width` (int, 默认: 640)

- **功能**: 图像宽度
- **作用**: 设置输出图像的宽度
- **单位**: 像素
- **示例**: `640`, `1280`, `1920`
- **注意事项**:
  - 受相机传感器分辨率限制
  - 影响数据传输量和处理负载
  - 常用分辨率: 640x480, 1280x720, 1920x1080

#### `image_height` (int, 默认: 480)

- **功能**: 图像高度
- **作用**: 设置输出图像的高度
- **单位**: 像素
- **示例**: `480`, `720`, `1080`
- **注意事项**:
  - 与image_width配合使用
  - 影响数据传输量和处理负载
  - 保持宽高比避免图像变形

## 参数调优指南

### 根据应用场景选择参数

#### 1. 高速运动检测

```yaml
frame_rate: 60.0        # 高帧率
exposure_time: 500.0    # 短曝光，减少运动模糊
gain: 2.0               # 适当增益补偿亮度
pixel_format: "mono8"   # 灰度格式，减少数据量
```

#### 2. 弱光环境

```yaml
frame_rate: 15.0        # 较低帧率
exposure_time: 4000.0   # 长曝光，增加亮度
gain: 5.0               # 高增益
pixel_format: "bgr8"    # 彩色格式
```

#### 3. 一般应用

```yaml
frame_rate: 30.0        # 标准帧率
exposure_time: 1000.0   # 标准曝光
gain: 1.0               # 无增益
pixel_format: "bgr8"    # 彩色格式
```

### 参数调整顺序

1. **先调整曝光时间** - 获得合适的图像亮度
2. **再调整增益** - 在曝光时间不够时增加亮度
3. **最后调整帧率** - 根据应用需求确定采集频率
4. **考虑像素格式** - 平衡图像质量和数据量

## 实际帧率 vs 设定帧率

### 设定帧率 (frame_rate)

- **定义**: 用户设置的期望帧率
- **作用**: 控制相机采集频率
- **限制**: 受相机硬件和网络条件限制

### 实际帧率 (actual_fps)

- **定义**: 系统实际达到的帧率
- **测量**: 通过帧率监控节点实时测量
- **影响因素**:
  - 网络带宽和延迟
  - 系统处理能力
  - 相机硬件性能
  - 图像分辨率和格式

### 帧率监控

系统提供实时帧率监控功能：

- **平均帧率**: 从启动到现在的平均帧率
- **当前帧率**: 基于最近几帧计算的帧率
- **最近帧率**: 最近1秒内的帧率

## 监控和调试

### 查看系统状态

```bash
# 查看所有节点
ros2 node list

# 查看话题列表
ros2 topic list

# 查看参数列表
ros2 param list /hik_camera_driver

# 查看话题信息
ros2 topic info /image_raw
```

### 常用命令

```bash
ros2 node list
ros2 param list /hik_camera_driver
ros2 param get /hik_camera_driver frame_rate
ros2 param set /hik_camera_driver frame_rate 30.0
```

### 实时监控

```bash
ros2 topic echo /image_raw --no-arr
ros2 topic hz /image_raw
```

### 性能分析

```bash
# 查看话题频率
ros2 topic hz /image_raw

# 查看话题带宽
ros2 topic bw /image_raw

# 查看节点资源使用
ros2 node info /hik_camera_driver
```

## 常见问题解决

### 1. 相机连接失败

```bash
# OpenDevice 失败（常见 -2147483133）：
# 1) 安装 udev 规则并触发
RULE_FILE=$(find "$HOME/opt/MVS" -maxdepth 7 -type f -iname '*hik*usb*rules' -o -iname '*mv*usb*rules' -o -iname '90-*.rules' | head -n1); \
[ -n "$RULE_FILE" ] && sudo cp "$RULE_FILE" /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
# 2) 确保未被 Viewer/虚拟相机窗口占用
pkill -f -i 'viewer|mvs|hik|virtual' || true
# 3) 检查 LD_LIBRARY_PATH 指向 $HOME/opt/MVS/lib/64
echo $LD_LIBRARY_PATH
```

### 2. 帧率不达标

```bash
# 检查网络带宽
ros2 topic bw /image_raw

# 降低图像分辨率
ros2 param set /hik_camera_driver image_width 640
ros2 param set /hik_camera_driver image_height 480

# 使用灰度格式
ros2 param set /hik_camera_driver pixel_format "mono8"
```

### 3. 图像质量问题

```bash
# 调整曝光时间
ros2 param set /hik_camera_driver exposure_time 2000.0

# 调整增益
ros2 param set /hik_camera_driver gain 3.0

# 检查像素格式
ros2 param get /hik_camera_driver pixel_format
```

## 性能优化建议

### 网络优化

- 使用千兆网络
- 确保网络稳定
- 避免网络拥塞

### 系统优化

- 使用SSD存储
- 增加系统内存
- 优化CPU使用

### 参数优化

- 根据应用需求调整帧率
- 平衡图像质量和性能
- 使用合适的像素格式

## 开发扩展

### 添加新功能

1. 在头文件中声明新方法
2. 在源文件中实现功能
3. 更新CMakeLists.txt
4. 重新编译和测试

### 自定义参数

1. 在构造函数中声明参数
2. 添加参数处理逻辑
3. 更新launch文件
4. 测试参数功能

### 集成其他节点

1. 创建新的launch文件
2. 配置节点依赖关系
3. 设置参数传递
4. 测试系统集成

## 验收要点（对照清单）

- ✅ **代码可通过 `colcon build` 编译，无严重警告/错误**
- ✅ **启动后可成功连接相机并采集图像**
- ✅ **在 RViz2 中能稳定查看 `/image_raw` 图像**
- ✅ **支持通过参数修改曝光、增益、帧率、像素格式并生效**
- ✅ **支持实时帧率监控和参数信息显示**
- ✅ **提供完整的管理脚本和启动方式**

## 故障排除

### 编译错误

```bash
# 清理构建目录
rm -rf build install log

# 重新编译
colcon build --packages-select hik_camera_driver
```

### 运行时错误

```bash
# 检查环境变量
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH

# 重新加载环境
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 权限问题

```bash
# 给脚本添加执行权限
chmod +x scripts/camera_manager.sh

# 检查相机设备权限
ls -la /dev/video*
```

## 技术架构

### 主要组件

1. **HikCameraDriver类**: 主要的相机驱动类
2. **MVSSDKWrapper类**: 海康威视SDK的封装
3. **参数管理**: 支持动态参数调整
4. **图像传输**: 基于image_transport的图像发布

### 关键特性

- 自动重连机制
- 动态参数调整
- 多线程图像采集
- 实时统计信息
- 异常处理机制

## 版本与注意事项

- 默认参数来源：`hik_camera_driver/config/camera_params.yaml`
- 可通过 `--ros-args -p hik_camera_driver.<param>:=<value>` 临时覆盖
- OpenCV 版本与 `cv_bridge` 可能提示冲突告警（无功能影响）；建议系统维护统一 OpenCV 版本

## 注意事项与建议

- **SDK配置**: 安装并正确配置海康 MVS C++ SDK（环境变量、库路径、udev 权限等）
- **运行权限**: USB 相机需 udev 规则，避免 root 运行
- **OpenCV版本**: OpenCV 版本与 `cv_bridge` 链接库可能出现提示，保持系统统一版本可避免冲突告警
- **网络配置**: 使用网络相机时，确保相机和主机在同一网段，并正确配置IP地址
- **权限设置**: USB相机可能需要设置udev规则，确保设备访问权限

---
