#!/usr/bin/env python3
"""
将ROS bag文件转换为donkeycar Tub格式的训练数据

Usage:
    rosbag2tub.py --bags=<bag_files> --output=<output_path> [--image-topic=<topic>] [--imu-topic=<topic>] [--odom-topic=<topic>] [--scan-topic=<topic>] [--cmd-vel-topic=<topic>] [--sync-window=<seconds>]

Options:
    --bags=<bag_files>          逗号分隔的ROS bag文件路径列表
    --output=<output_path>       输出Tub目录路径
    --image-topic=<topic>        图像话题名称 [default: /usb_cam/image_raw]
    --imu-topic=<topic>          IMU话题名称 [default: /imu]
    --odom-topic=<topic>         里程计话题名称 [default: /odom]
    --scan-topic=<topic>         激光雷达话题名称 [default: /scan]
    --cmd-vel-topic=<topic>      控制指令话题名称 [default: /cmd_vel]
    --sync-window=<seconds>      时间同步窗口（秒） [default: 0.1]
"""

import os
import sys
import time
import numpy as np
from docopt import docopt
from progress.bar import IncrementalBar

try:
    import rospy
    import rosbag
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, Imu, LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
except ImportError as e:
    print(f"错误: 需要安装ROS Python包。请确保已source ROS环境: {e}")
    print("安装方法: sudo apt-get install python3-rosbag python3-cv-bridge")
    sys.exit(1)

from donkeycar.parts.tub_v2 import Tub
from PIL import Image as PILImage
import cv2


class ROSBagConverter:
    """将ROS bag文件转换为Tub格式"""
    
    def __init__(self, bag_files, output_path, topics_config, sync_window=0.1):
        self.bag_files = bag_files
        self.output_path = output_path
        self.topics_config = topics_config
        self.sync_window = sync_window
        self.bridge = CvBridge()
        
        # 定义Tub的输入和类型
        self.inputs = ['cam/image_array']
        self.types = ['image_array']
        
        # 根据话题配置添加输入
        if topics_config['imu']:
            self.inputs.extend(['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
                               'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z'])
            self.types.extend(['float', 'float', 'float', 'float', 'float', 'float'])
        
        if topics_config['odom']:
            self.inputs.append('enc/speed')
            self.types.append('float')
        
        if topics_config['scan']:
            self.inputs.append('lidar/dist_array')
            self.types.append('nparray')
        
        # 添加标签（如果有cmd_vel）
        if topics_config['cmd_vel']:
            self.inputs.extend(['user/angle', 'user/throttle'])
            self.types.extend(['float', 'float'])
    
    def convert_image(self, ros_image_msg):
        """将ROS Image消息转换为numpy数组"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, "rgb8")
            return cv_image
        except Exception as e:
            print(f"图像转换错误: {e}")
            return None
    
    def extract_imu_data(self, imu_msg):
        """从IMU消息中提取数据"""
        linear_accel = imu_msg.linear_acceleration
        angular_vel = imu_msg.angular_velocity
        return (
            linear_accel.x, linear_accel.y, linear_accel.z,
            angular_vel.x, angular_vel.y, angular_vel.z
        )
    
    def extract_odom_data(self, odom_msg):
        """从里程计消息中提取速度"""
        return odom_msg.twist.twist.linear.x
    
    def extract_scan_data(self, scan_msg):
        """从激光雷达消息中提取距离数组"""
        ranges = np.array(scan_msg.ranges)
        # 将inf和nan替换为最大距离
        ranges = np.nan_to_num(ranges, nan=scan_msg.range_max, posinf=scan_msg.range_max)
        return ranges
    
    def extract_cmd_vel_data(self, twist_msg):
        """从cmd_vel消息中提取steering和throttle"""
        # 假设使用差速模型：angular.z对应steering，linear.x对应throttle
        # 需要根据实际小车配置调整转换逻辑
        max_angular = 1.0  # 最大角速度，可根据配置调整
        max_linear = 1.0   # 最大线速度，可根据配置调整
        
        steering = twist_msg.angular.z / max_angular if max_angular > 0 else 0.0
        throttle = twist_msg.linear.x / max_linear if max_linear > 0 else 0.0
        
        # 限制范围到[-1, 1]
        steering = np.clip(steering, -1.0, 1.0)
        throttle = np.clip(throttle, -1.0, 1.0)
        
        return steering, throttle
    
    def find_nearest_message(self, target_time, message_list):
        """找到最接近目标时间的消息"""
        if not message_list:
            return None
        
        min_diff = float('inf')
        nearest_msg = None
        
        for msg_time, msg_data in message_list:
            diff = abs(msg_time - target_time)
            if diff < min_diff and diff <= self.sync_window:
                min_diff = diff
                nearest_msg = (msg_time, msg_data)
        
        return nearest_msg
    
    def read_bag_messages(self, bag_path):
        """读取bag文件中的所有消息"""
        print(f"读取bag文件: {bag_path}")
        messages = {
            'image': [],
            'imu': [],
            'odom': [],
            'scan': [],
            'cmd_vel': []
        }
        
        try:
            bag = rosbag.Bag(bag_path, 'r')
            
            for topic, msg, t in bag.read_messages():
                timestamp = t.to_sec()
                
                if topic == self.topics_config['image']:
                    messages['image'].append((timestamp, msg))
                elif topic == self.topics_config['imu']:
                    messages['imu'].append((timestamp, msg))
                elif topic == self.topics_config['odom']:
                    messages['odom'].append((timestamp, msg))
                elif topic == self.topics_config['scan']:
                    messages['scan'].append((timestamp, msg))
                elif topic == self.topics_config['cmd_vel']:
                    messages['cmd_vel'].append((timestamp, msg))
            
            bag.close()
            
            # 打印统计信息
            print(f"  图像消息: {len(messages['image'])}")
            print(f"  IMU消息: {len(messages['imu'])}")
            print(f"  里程计消息: {len(messages['odom'])}")
            print(f"  激光雷达消息: {len(messages['scan'])}")
            print(f"  控制指令消息: {len(messages['cmd_vel'])}")
            
        except Exception as e:
            print(f"读取bag文件错误: {e}")
            return None
        
        return messages
    
    def convert(self):
        """执行转换"""
        print(f"开始转换ROS bag文件到Tub格式...")
        print(f"输出路径: {self.output_path}")
        
        # 创建输出目录
        os.makedirs(self.output_path, exist_ok=True)
        
        # 创建Tub
        tub = Tub(self.output_path, inputs=self.inputs, types=self.types)
        
        total_records = 0
        
        # 处理每个bag文件
        for bag_file in self.bag_files:
            if not os.path.exists(bag_file):
                print(f"警告: bag文件不存在: {bag_file}")
                continue
            
            messages = self.read_bag_messages(bag_file)
            if messages is None:
                continue
            
            # 使用图像时间戳作为主时间线
            image_messages = messages['image']
            if not image_messages:
                print(f"警告: bag文件中没有图像消息: {bag_file}")
                continue
            
            print(f"处理 {len(image_messages)} 个图像帧...")
            bar = IncrementalBar('转换进度', max=len(image_messages))
            
            for img_time, img_msg in image_messages:
                # 转换图像
                img_array = self.convert_image(img_msg)
                if img_array is None:
                    bar.next()
                    continue
                
                # 构建记录
                record = {
                    'cam/image_array': img_array
                }
                
                # 同步其他传感器数据
                if self.topics_config['imu'] and messages['imu']:
                    imu_msg = self.find_nearest_message(img_time, messages['imu'])
                    if imu_msg:
                        imu_data = self.extract_imu_data(imu_msg[1])
                        record['imu/acl_x'] = imu_data[0]
                        record['imu/acl_y'] = imu_data[1]
                        record['imu/acl_z'] = imu_data[2]
                        record['imu/gyr_x'] = imu_data[3]
                        record['imu/gyr_y'] = imu_data[4]
                        record['imu/gyr_z'] = imu_data[5]
                
                if self.topics_config['odom'] and messages['odom']:
                    odom_msg = self.find_nearest_message(img_time, messages['odom'])
                    if odom_msg:
                        speed = self.extract_odom_data(odom_msg[1])
                        record['enc/speed'] = speed
                
                if self.topics_config['scan'] and messages['scan']:
                    scan_msg = self.find_nearest_message(img_time, messages['scan'])
                    if scan_msg:
                        dist_array = self.extract_scan_data(scan_msg[1])
                        record['lidar/dist_array'] = dist_array
                
                # 如果有cmd_vel，作为标签
                if self.topics_config['cmd_vel'] and messages['cmd_vel']:
                    cmd_msg = self.find_nearest_message(img_time, messages['cmd_vel'])
                    if cmd_msg:
                        steering, throttle = self.extract_cmd_vel_data(cmd_msg[1])
                        record['user/angle'] = steering
                        record['user/throttle'] = throttle
                
                # 写入记录
                try:
                    tub.write_record(record)
                    total_records += 1
                except Exception as e:
                    print(f"\n写入记录错误: {e}")
                
                bar.next()
            
            bar.finish()
        
        tub.close()
        print(f"\n转换完成! 共生成 {total_records} 条记录")
        print(f"Tub路径: {self.output_path}")


def main():
    args = docopt(__doc__)
    
    # 解析bag文件列表
    bag_files = [f.strip() for f in args['--bags'].split(',')]
    
    # 话题配置
    topics_config = {
        'image': args['--image-topic'],
        'imu': args['--imu-topic'],
        'odom': args['--odom-topic'],
        'scan': args['--scan-topic'],
        'cmd_vel': args['--cmd-vel-topic']
    }
    
    sync_window = float(args['--sync-window'])
    output_path = args['--output']
    
    # 检查ROS环境
    if 'ROS_DISTRO' not in os.environ:
        print("警告: 未检测到ROS环境变量。请确保已source ROS setup.bash")
        print("例如: source /opt/ros/noetic/setup.bash")
    
    # 执行转换
    converter = ROSBagConverter(bag_files, output_path, topics_config, sync_window)
    converter.convert()


if __name__ == '__main__':
    main()
