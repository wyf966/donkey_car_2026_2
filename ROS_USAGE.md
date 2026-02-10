# ROS集成使用说明

本文档说明如何使用donkeycar的ROS集成功能，包括ROS bag文件训练和实时控制。

## 功能概述

1. **ROS bag转Tub工具**：将ROS bag文件转换为donkeycar训练格式
2. **ROS订阅器Parts**：实时接收ROS话题数据（图像、IMU、里程计、激光雷达）
3. **ROS发布器Part**：发布控制指令到/cmd_vel话题

## 一、训练阶段：ROS bag文件转Tub格式

### 1.1 准备ROS bag文件

确保您的ROS bag文件包含以下话题（或根据实际情况调整话题名称）：
- `/usb_cam/image_raw` - 相机图像
- `/imu` - IMU数据（可选）
- `/odom` - 里程计数据
- `/scan` - 激光雷达数据（可选）
- `/cmd_vel` - 控制指令（作为训练标签）

### 1.2 转换bag文件

```bash
# 单个bag文件
python scripts/rosbag2tub.py --bags=newdata.bag --output=data/ros_tub

# 多个bag文件
python scripts/rosbag2tub.py --bags=newdata1.bag,newdata2.bag,newdata3.bag --output=data/ros_tub

# 自定义话题名称
python scripts/rosbag2tub.py \
    --bags=newdata.bag \
    --output=data/ros_tub \
    --image-topic=/camera/image_raw \
    --imu-topic=/imu_data \
    --odom-topic=/odometry \
    --scan-topic=/laser_scan \
    --cmd-vel-topic=/velocity_command \
    --sync-window=0.1
```

### 1.3 训练模型

使用标准donkeycar训练命令：

```bash
python manage.py train --tubs=data/ros_tub --model=models/ros_pilot.h5
```

## 二、运行时阶段：ROS实时控制

### 2.1 配置ROS环境

在使用ROS功能之前，确保已source ROS环境：

```bash
# ROS Noetic
source /opt/ros/noetic/setup.bash

# ROS Melodic
source /opt/ros/melodic/setup.bash
```

### 2.2 配置ROS驱动

复制并编辑ROS驱动配置文件：

```bash
cp donkeycar/templates/cfg_ros_drive.py mycar/cfg_ros_drive.py
```

编辑`cfg_ros_drive.py`，根据实际情况配置：
- ROS话题名称
- 最大速度和角速度
- 传感器启用/禁用

### 2.3 启动ROS驱动

```bash
python manage.py drive --myconfig=cfg_ros_drive.py --model=models/ros_pilot.h5
```

### 2.4 使用Web控制器

启动后，在浏览器中访问 `http://localhost:8887` 进行控制：
- **用户模式**：手动控制小车
- **本地模式**：AI自动驾驶

## 三、配置文件说明

### 3.1 ROS话题配置

在`cfg_ros_drive.py`中配置ROS话题：

```python
# 图像话题（必需）
ROS_IMAGE_TOPIC = '/usb_cam/image_raw'

# IMU话题（可选）
ROS_USE_IMU = False
ROS_IMU_TOPIC = '/imu'

# 里程计话题（必需）
ROS_ODOM_TOPIC = '/odom'

# 激光雷达话题（可选）
ROS_USE_LIDAR = False
ROS_SCAN_TOPIC = '/scan'

# 控制指令话题
ROS_CMD_VEL_TOPIC = '/cmd_vel'
ROS_MAX_SPEED = 1.0                    # 最大线速度（m/s）
ROS_MAX_ANGULAR_VELOCITY = 1.0         # 最大角速度（rad/s）
```

### 3.2 模型输入配置

根据使用的传感器配置模型输入：

- **仅相机+里程计**（默认）：
  ```python
  ROS_USE_IMU = False
  ROS_USE_LIDAR = False
  ```

- **相机+里程计+激光雷达**（避障）：
  ```python
  ROS_USE_IMU = False
  ROS_USE_LIDAR = True
  ```

- **相机+里程计+IMU**：
  ```python
  ROS_USE_IMU = True
  ROS_USE_LIDAR = False
  ```

## 四、数据流说明

### 4.1 训练数据流

```
ROS bag文件 → rosbag2tub.py → Tub数据集 → 训练脚本 → 模型文件
```

### 4.2 运行时数据流

```
ROS话题 → ROS订阅器 → Vehicle Memory → 模型推理 → ROS发布器 → ROS话题
```

## 五、故障排除

### 5.1 ROS包未安装

错误：`ROS包未安装，无法使用ROSCameraSubscriber`

解决：安装ROS Python包（参见`ROS_DEPENDENCIES.md`）

### 5.2 ROS环境未配置

错误：`未检测到ROS环境变量`

解决：source ROS环境
```bash
source /opt/ros/noetic/setup.bash
```

### 5.3 话题不存在

错误：`话题不存在或没有数据`

解决：
1. 检查ROS话题是否正在发布：`rostopic list`
2. 检查话题数据：`rostopic echo /topic_name`
3. 在配置文件中更新话题名称

### 5.4 图像转换错误

错误：`图像转换错误`

解决：
1. 确保图像话题消息类型为`sensor_msgs/Image`
2. 检查图像编码格式（通常为`rgb8`或`bgr8`）

## 六、示例工作流

### 完整工作流示例

```bash
# 1. 录制ROS bag文件（在小车上）
rosbag record -o newdata.bag /imu /usb_cam/image_raw /odom /scan /cmd_vel

# 2. 转换bag文件为Tub格式（在训练机器上）
source /opt/ros/noetic/setup.bash
python scripts/rosbag2tub.py --bags=newdata.bag --output=data/ros_tub

# 3. 训练模型
python manage.py train --tubs=data/ros_tub --model=models/ros_pilot.h5

# 4. 部署到小车并运行（在小车上）
source /opt/ros/noetic/setup.bash
python manage.py drive --myconfig=cfg_ros_drive.py --model=models/ros_pilot.h5
```

## 七、注意事项

1. **时间同步**：bag文件转换时，不同话题的消息会按时间戳对齐（默认窗口0.1秒）
2. **数据对齐**：确保训练数据中图像、传感器数据和标签时间对齐
3. **控制指令转换**：cmd_vel到steering/throttle的转换可能需要根据实际小车调整
4. **线程安全**：ROS订阅器使用线程安全设计，避免阻塞主循环
5. **ROS节点名称**：使用`anonymous=True`避免节点名称冲突

## 八、高级配置

### 8.1 自定义cmd_vel转换

如果需要自定义steering/throttle到cmd_vel的转换逻辑，可以修改`donkeycar/parts/ros_bridge.py`中的`ROSCmdVelPublisher.extract_cmd_vel_data()`方法。

### 8.2 添加自定义传感器

可以参考现有的ROS订阅器Parts，创建自定义传感器订阅器。

### 8.3 多bag文件训练

支持同时转换多个bag文件：
```bash
python scripts/rosbag2tub.py --bags=bag1.bag,bag2.bag,bag3.bag --output=data/combined_tub
```
