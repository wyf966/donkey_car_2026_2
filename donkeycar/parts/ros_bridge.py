"""
ROS桥接Parts：用于donkeycar与ROS系统之间的数据交换

包含：
- ROS订阅器Parts：从ROS话题接收传感器数据
- ROS发布器Part：向ROS话题发布控制指令
"""

import threading
import time
import numpy as np
import logging

logger = logging.getLogger(__name__)

try:
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, Imu, LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    ROS_AVAILABLE = True
except ImportError as e:
    logger.warning(f"ROS包未安装: {e}。ROS功能将不可用。")
    ROS_AVAILABLE = False


class ROSCameraSubscriber(object):
    """
    订阅ROS图像话题，将图像数据写入donkeycar Memory
    
    订阅话题: /usb_cam/image_raw (sensor_msgs/Image)
    输出: cam/image_array (numpy array, H×W×3, uint8)
    """
    
    def __init__(self, topic_name='/usb_cam/image_raw', node_name='donkey_camera_sub'):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS包未安装，无法使用ROSCameraSubscriber")
        
        self.topic_name = topic_name
        self.node_name = node_name
        self.bridge = CvBridge()
        self.image_array = None
        self.lock = threading.Lock()
        self.running = False
        
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.get_node_uri():
            rospy.init_node(node_name, anonymous=True)
        
        # 订阅图像话题
        self.subscriber = rospy.Subscriber(topic_name, Image, self.image_callback)
        logger.info(f"ROSCameraSubscriber: 订阅话题 {topic_name}")
    
    def image_callback(self, msg):
        """图像消息回调函数"""
        try:
            # 将ROS图像消息转换为numpy数组
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            with self.lock:
                self.image_array = cv_image
        except Exception as e:
            logger.error(f"图像转换错误: {e}")
    
    def run_threaded(self):
        """线程安全地返回最新图像"""
        with self.lock:
            return self.image_array.copy() if self.image_array is not None else None
    
    def run(self):
        """非线程版本（不推荐）"""
        return self.run_threaded()
    
    def shutdown(self):
        """关闭订阅器"""
        if hasattr(self, 'subscriber'):
            self.subscriber.unregister()
        self.running = False


class ROSIMUSubscriber(object):
    """
    订阅ROS IMU话题，提取加速度和角速度数据
    
    订阅话题: /imu (sensor_msgs/Imu)
    输出: imu/acl_x, imu/acl_y, imu/acl_z, imu/gyr_x, imu/gyr_y, imu/gyr_z (float)
    """
    
    def __init__(self, topic_name='/imu', node_name='donkey_imu_sub'):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS包未安装，无法使用ROSIMUSubscriber")
        
        self.topic_name = topic_name
        self.node_name = node_name
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.lock = threading.Lock()
        
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.get_node_uri():
            rospy.init_node(node_name, anonymous=True)
        
        # 订阅IMU话题
        self.subscriber = rospy.Subscriber(topic_name, Imu, self.imu_callback)
        logger.info(f"ROSIMUSubscriber: 订阅话题 {topic_name}")
    
    def imu_callback(self, msg):
        """IMU消息回调函数"""
        with self.lock:
            self.accel_x = msg.linear_acceleration.x
            self.accel_y = msg.linear_acceleration.y
            self.accel_z = msg.linear_acceleration.z
            self.gyro_x = msg.angular_velocity.x
            self.gyro_y = msg.angular_velocity.y
            self.gyro_z = msg.angular_velocity.z
    
    def run_threaded(self):
        """线程安全地返回IMU数据"""
        with self.lock:
            return (self.accel_x, self.accel_y, self.accel_z,
                   self.gyro_x, self.gyro_y, self.gyro_z)
    
    def run(self):
        """非线程版本"""
        return self.run_threaded()
    
    def shutdown(self):
        """关闭订阅器"""
        if hasattr(self, 'subscriber'):
            self.subscriber.unregister()


class ROSOdomSubscriber(object):
    """
    订阅ROS里程计话题，提取速度数据
    
    订阅话题: /odom (nav_msgs/Odometry)
    输出: enc/speed (float, 从twist.linear.x提取)
    """
    
    def __init__(self, topic_name='/odom', node_name='donkey_odom_sub'):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS包未安装，无法使用ROSOdomSubscriber")
        
        self.topic_name = topic_name
        self.node_name = node_name
        self.speed = 0.0
        self.lock = threading.Lock()
        
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.get_node_uri():
            rospy.init_node(node_name, anonymous=True)
        
        # 订阅里程计话题
        self.subscriber = rospy.Subscriber(topic_name, Odometry, self.odom_callback)
        logger.info(f"ROSOdomSubscriber: 订阅话题 {topic_name}")
    
    def odom_callback(self, msg):
        """里程计消息回调函数"""
        with self.lock:
            self.speed = msg.twist.twist.linear.x
    
    def run_threaded(self):
        """线程安全地返回速度"""
        with self.lock:
            return self.speed
    
    def run(self):
        """非线程版本"""
        return self.run_threaded()
    
    def shutdown(self):
        """关闭订阅器"""
        if hasattr(self, 'subscriber'):
            self.subscriber.unregister()


class ROSScanSubscriber(object):
    """
    订阅ROS激光雷达话题，提取距离数组
    
    订阅话题: /scan (sensor_msgs/LaserScan)
    输出: lidar/dist_array (numpy array)
    """
    
    def __init__(self, topic_name='/scan', node_name='donkey_scan_sub'):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS包未安装，无法使用ROSScanSubscriber")
        
        self.topic_name = topic_name
        self.node_name = node_name
        self.dist_array = None
        self.lock = threading.Lock()
        
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.get_node_uri():
            rospy.init_node(node_name, anonymous=True)
        
        # 订阅激光雷达话题
        self.subscriber = rospy.Subscriber(topic_name, LaserScan, self.scan_callback)
        logger.info(f"ROSScanSubscriber: 订阅话题 {topic_name}")
    
    def scan_callback(self, msg):
        """激光雷达消息回调函数"""
        try:
            ranges = np.array(msg.ranges)
            # 将inf和nan替换为最大距离
            ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)
            with self.lock:
                self.dist_array = ranges
        except Exception as e:
            logger.error(f"激光雷达数据处理错误: {e}")
    
    def run_threaded(self):
        """线程安全地返回距离数组"""
        with self.lock:
            return self.dist_array.copy() if self.dist_array is not None else None
    
    def run(self):
        """非线程版本"""
        return self.run_threaded()
    
    def shutdown(self):
        """关闭订阅器"""
        if hasattr(self, 'subscriber'):
            self.subscriber.unregister()


class ROSCmdVelPublisher(object):
    """
    从donkeycar Memory读取steering和throttle，发布到ROS cmd_vel话题
    
    输入: steering, throttle (从Memory读取)
    发布话题: /cmd_vel (geometry_msgs/Twist)
    
    转换逻辑:
    - linear.x = throttle * max_speed
    - angular.z = steering * max_angular_velocity
    """
    
    def __init__(self, topic_name='/cmd_vel', node_name='donkey_cmd_vel_pub',
                 max_speed=1.0, max_angular_velocity=1.0):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS包未安装，无法使用ROSCmdVelPublisher")
        
        self.topic_name = topic_name
        self.node_name = node_name
        self.max_speed = max_speed
        self.max_angular_velocity = max_angular_velocity
        
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.get_node_uri():
            rospy.init_node(node_name, anonymous=True)
        
        # 创建发布器
        self.publisher = rospy.Publisher(topic_name, Twist, queue_size=1)
        logger.info(f"ROSCmdVelPublisher: 发布话题 {topic_name}")
        logger.info(f"  最大速度: {max_speed} m/s")
        logger.info(f"  最大角速度: {max_angular_velocity} rad/s")
        
        # 等待订阅者连接
        time.sleep(0.5)
    
    def run(self, steering, throttle):
        """
        发布控制指令
        
        Args:
            steering: 转向角，范围[-1, 1]，-1为左转，1为右转
            throttle: 油门，范围[-1, 1]，-1为倒车，1为前进
        """
        if steering is None or throttle is None:
            return
        
        # 创建Twist消息
        twist = Twist()
        
        # 转换steering和throttle到cmd_vel
        # 对于差速驱动模型：
        # - linear.x: 线速度（前进/后退）
        # - angular.z: 角速度（转向）
        twist.linear.x = float(np.clip(throttle * self.max_speed, -self.max_speed, self.max_speed))
        twist.angular.z = float(np.clip(steering * self.max_angular_velocity, 
                                        -self.max_angular_velocity, self.max_angular_velocity))
        
        # 发布消息
        if not rospy.is_shutdown():
            self.publisher.publish(twist)
    
    def run_threaded(self, steering, throttle):
        """线程版本"""
        return self.run(steering, throttle)
    
    def shutdown(self):
        """关闭发布器"""
        if hasattr(self, 'publisher'):
            # 发送停止指令
            twist = Twist()
            self.publisher.publish(twist)
            time.sleep(0.1)
