# ROS依赖项说明

本项目的ROS集成功能需要以下ROS Python包。这些包通常通过ROS系统包管理器安装，而不是通过pip。

## 必需的ROS包

### Ubuntu/Debian系统

```bash
# 对于ROS Noetic (Ubuntu 20.04)
sudo apt-get install python3-rosbag python3-cv-bridge python3-rospy

# 对于ROS Melodic (Ubuntu 18.04)
sudo apt-get install python-rosbag python-cv-bridge python-rospy

# 对于ROS Kinetic (Ubuntu 16.04)
sudo apt-get install python-rosbag python-cv-bridge python-rospy
```

### 依赖的ROS消息类型

- `sensor_msgs/Image` - 图像消息
- `sensor_msgs/Imu` - IMU消息
- `sensor_msgs/LaserScan` - 激光雷达消息
- `nav_msgs/Odometry` - 里程计消息
- `geometry_msgs/Twist` - 速度控制消息

这些消息类型通常包含在以下ROS包中：
- `ros-noetic-sensor-msgs` (Noetic)
- `ros-noetic-nav-msgs` (Noetic)
- `ros-noetic-geometry-msgs` (Noetic)

## 环境配置

在使用ROS功能之前，需要source ROS环境：

```bash
# ROS Noetic
source /opt/ros/noetic/setup.bash

# ROS Melodic
source /opt/ros/melodic/setup.bash

# ROS Kinetic
source /opt/ros/kinetic/setup.bash
```

## 验证安装

运行以下命令验证ROS包是否正确安装：

```python
python3 -c "import rospy; import rosbag; from cv_bridge import CvBridge; print('ROS packages installed successfully')"
```

## 注意事项

1. ROS Python包必须与系统安装的ROS版本匹配
2. 确保ROS环境变量（如`ROS_DISTRO`）已正确设置
3. 如果使用conda环境，可能需要额外配置以使用系统ROS包
4. `cv_bridge`需要OpenCV，确保已安装`python3-opencv`或`python-opencv`
