#!/usr/bin/env python3
"""
datastore_to_bag_no_rospy.py

将 DonkeyCar v5 datastore (catalog + images) 导出为 ROS bag，
可用 `rosbag play donkey_data.bag --clock` 自动播放控制小车。
"""

import os
import json
import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import rospy

# --------------- 配置 ---------------
DATA_DIR = os.path.expanduser("~/Desktop/donkey1/project/mycar/data")
BAG_FILE = os.path.join(DATA_DIR, "donkey_data.bag")

CAM_TOPIC = "/camera/image_raw"
CMD_TOPIC = "/cmd_vel"

# 每条消息间隔时间，单位秒
TIME_INTERVAL = 0.05

# --------------- 工具 ---------------
bridge = CvBridge()

def catalog_files(data_dir):
    """返回 catalog 文件列表"""
    files = sorted([
        f for f in os.listdir(data_dir)
        if f.startswith("catalog") and f.endswith(".catalog")
    ])
    return files

def load_catalog(catalog_path):
    """读取 catalog 文件，返回记录列表"""
    with open(catalog_path, "r") as f:
        records = []
        for line in f:
            try:
                rec = json.loads(line)
                records.append(rec)
            except:
                continue
    return records

# --------------- 主流程 ---------------
if __name__ == "__main__":
    all_records = []

    # 读取所有 catalog 文件
    catalogs = catalog_files(DATA_DIR)
    print(f"Found {len(catalogs)} catalog files.")

    for cfile in catalogs:
        cpath = os.path.join(DATA_DIR, cfile)
        records = load_catalog(cpath)
        all_records.extend(records)

    print(f"Total records: {len(all_records)}")

    # 创建 ROS bag
    with rosbag.Bag(BAG_FILE, 'w') as bag:
        for i, rec in enumerate(all_records):
            # ---- 处理相机图像 ----
            img_path = os.path.join(DATA_DIR, "images", os.path.basename(rec['cam/image_array']))
            if not os.path.exists(img_path):
                continue

            cv_img = cv2.imread(img_path)
            if cv_img is None:
                continue

            ros_img = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")

            # 时间戳
            stamp = rospy.Time.from_sec(i * TIME_INTERVAL)
            ros_img.header.stamp = stamp

            bag.write(CAM_TOPIC, ros_img, t=stamp)

            # ---- 处理控制指令 ----
            twist = Twist()
            twist.linear.x = float(rec.get('user/throttle', 0.0))
            twist.angular.z = float(rec.get('user/angle', 0.0))
            bag.write(CMD_TOPIC, twist, t=stamp)

    print(f"ROS bag saved to: {BAG_FILE}")

