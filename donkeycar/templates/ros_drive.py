#!/usr/bin/env python3
"""
ROS驱动模板：使用ROS话题进行数据采集和控制

Usage:
    manage.py drive --myconfig=cfg_ros_drive.py [--model=<model>] [--type=(linear|categorical)]

Options:
    --model=<model>              模型文件路径
    --type=<type>                模型类型
"""

from docopt import docopt
import donkeycar as dk
from donkeycar.parts.transform import TriggeredCallback, DelayedTrigger
from donkeycar.parts.tub_v2 import TubWriter
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.controller import LocalWebController
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.behavior import BehaviorPart
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.launch import AiLaunch
from donkeycar.parts.explode import ExplodeDict
from donkeycar.parts.transform import Lambda
from donkeycar.parts.pipe import Pipe
from donkeycar.utils import *
import logging
import time
import os

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def drive(cfg, model_path=None, model_type=None, meta=[]):
    """
    使用ROS话题构建donkeycar车辆驱动循环
    
    从ROS话题接收：
    - 图像: /usb_cam/image_raw
    - IMU: /imu (可选)
    - 里程计: /odom
    - 激光雷达: /scan (可选)
    
    向ROS话题发布：
    - 控制指令: /cmd_vel
    """
    logger.info(f'PID: {os.getpid()}')
    
    # 初始化车辆
    V = dk.vehicle.Vehicle()
    
    # 初始化日志
    if cfg.HAVE_CONSOLE_LOGGING:
        logger.setLevel(logging.getLevelName(cfg.LOGGING_LEVEL))
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter(cfg.LOGGING_FORMAT))
        logger.addHandler(ch)
    
    #
    # 添加ROS传感器订阅器
    #
    from donkeycar.parts.ros_bridge import (
        ROSCameraSubscriber, ROSIMUSubscriber, 
        ROSOdomSubscriber, ROSScanSubscriber
    )
    
    # ROS相机订阅器
    logger.info("添加ROS相机订阅器...")
    ros_camera = ROSCameraSubscriber(
        topic_name=cfg.ROS_IMAGE_TOPIC,
        node_name='donkey_camera_sub'
    )
    V.add(ros_camera, inputs=[], outputs=['cam/image_array'], threaded=True)
    
    # ROS里程计订阅器（必需）
    logger.info("添加ROS里程计订阅器...")
    ros_odom = ROSOdomSubscriber(
        topic_name=cfg.ROS_ODOM_TOPIC,
        node_name='donkey_odom_sub'
    )
    V.add(ros_odom, inputs=[], outputs=['enc/speed'], threaded=True)
    
    # ROS IMU订阅器（可选）
    if cfg.ROS_USE_IMU:
        logger.info("添加ROS IMU订阅器...")
        ros_imu = ROSIMUSubscriber(
            topic_name=cfg.ROS_IMU_TOPIC,
            node_name='donkey_imu_sub'
        )
        V.add(ros_imu, inputs=[], 
              outputs=['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
                      'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z'],
              threaded=True)
    
    # ROS激光雷达订阅器（可选，用于避障）
    if cfg.ROS_USE_LIDAR:
        logger.info("添加ROS激光雷达订阅器...")
        ros_scan = ROSScanSubscriber(
            topic_name=cfg.ROS_SCAN_TOPIC,
            node_name='donkey_scan_sub'
        )
        V.add(ros_scan, inputs=[], outputs=['lidar/dist_array'], threaded=True)
    
    #
    # 添加用户控制器（Web界面）
    #
    if cfg.CONTROLLER_TYPE == "web":
        ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT, mode=cfg.WEB_INIT_MODE)
        V.add(ctr,
              inputs=['cam/image_array'],
              outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
              threaded=True)
    
    # 转换user/steering到user/angle（向后兼容）
    V.add(Pipe(), inputs=['user/steering'], outputs=['user/angle'])
    
    # 展开按钮输入
    V.add(ExplodeDict(V.mem, "web/"), inputs=['web/buttons'])
    
    # 油门过滤器
    th_filter = ThrottleFilter()
    V.add(th_filter, inputs=['user/throttle'], outputs=['user/throttle'])
    
    #
    # 用户模式和自动驾驶模式条件
    #
    from donkeycar.parts.controller import UserPilotCondition
    V.add(UserPilotCondition(show_pilot_image=getattr(cfg, 'SHOW_PILOT_IMAGE', False)),
          inputs=['user/mode', "cam/image_array"],
          outputs=['run_user', "run_pilot", "ui/image_array"])
    
    #
    # 加载模型
    #
    if model_path:
        kl = dk.utils.get_model_by_type(model_type or cfg.DEFAULT_MODEL_TYPE, cfg)
        
        def load_model(kl, model_path):
            start = time.time()
            logger.info(f'加载模型: {model_path}')
            kl.load(model_path)
            logger.info(f'模型加载完成，耗时: {time.time() - start:.2f}秒')
        
        # 加载模型
        load_model(kl, model_path)
        
        # 模型文件监控和重载
        V.add(FileWatcher(model_path, verbose=True),
              outputs=['modelfile/modified'])
        V.add(FileWatcher(model_path), outputs=['modelfile/dirty'],
              run_condition="run_pilot")
        V.add(DelayedTrigger(100), inputs=['modelfile/dirty'],
              outputs=['modelfile/reload'], run_condition="run_pilot")
        V.add(TriggeredCallback(model_path, lambda f: load_model(kl, f)),
              inputs=["modelfile/reload"], run_condition="run_pilot")
        
        #
        # 收集模型输入
        #
        if cfg.TRAIN_BEHAVIORS:
            bh = BehaviorPart(cfg.BEHAVIOR_LIST)
            V.add(bh, outputs=['behavior/state', 'behavior/label', "behavior/one_hot_state_array"])
            inputs = ['cam/image_array', "behavior/one_hot_state_array"]
        elif cfg.ROS_USE_LIDAR:
            inputs = ['cam/image_array', 'lidar/dist_array']
        elif cfg.ROS_USE_IMU:
            class Vectorizer:
                def run(self, *components):
                    return components
            
            V.add(Vectorizer, inputs=['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
                                      'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z'],
                  outputs=['imu_array'])
            inputs = ['cam/image_array', 'imu_array']
        else:
            # 默认：图像 + 里程计
            inputs = ['cam/image_array', 'enc/speed']
        
        # 模型输出
        outputs = ['pilot/angle', 'pilot/throttle']
        if cfg.TRAIN_LOCALIZER:
            outputs.append("pilot/loc")
        
        # 添加图像变换
        if hasattr(cfg, 'TRANSFORMATIONS') or hasattr(cfg, 'POST_TRANSFORMATIONS'):
            from donkeycar.parts.image_transformations import ImageTransformations
            logger.info("添加推理时的图像变换")
            V.add(ImageTransformations(cfg, 'TRANSFORMATIONS', 'POST_TRANSFORMATIONS'),
                  inputs=['cam/image_array'], outputs=['cam/image_array_trans'])
            inputs = ['cam/image_array_trans'] + inputs[1:]
        
        # 添加模型推理
        V.add(kl, inputs=inputs, outputs=outputs, run_condition='run_pilot')
        
        # AI启动加速
        aiLauncher = AiLaunch(cfg.AI_LAUNCH_DURATION, cfg.AI_LAUNCH_THROTTLE, 
                              cfg.AI_LAUNCH_KEEP_ENABLED)
        V.add(aiLauncher,
              inputs=['user/mode', 'pilot/throttle'],
              outputs=['pilot/throttle'])
    
    #
    # 记录控制（可选）
    #
    if cfg.AUTO_RECORD_ON_THROTTLE or cfg.RECORD_DURING_AI:
        from donkeycar.parts.controller import ToggleRecording
        recording_control = ToggleRecording(cfg.AUTO_RECORD_ON_THROTTLE, cfg.RECORD_DURING_AI)
        V.add(recording_control, inputs=['user/mode', "recording"], outputs=["recording"])
    
    #
    # 驱动模式选择：用户控制或AI控制
    #
    from donkeycar.parts.controller import DriveMode
    drive_mode = DriveMode(cfg.AI_THROTTLE_MULT)
    V.add(drive_mode,
          inputs=['user/mode', 'user/angle', 'user/throttle',
                 'pilot/angle', 'pilot/throttle'],
          outputs=['steering', 'throttle'])
    
    #
    # 添加ROS控制指令发布器
    #
    from donkeycar.parts.ros_bridge import ROSCmdVelPublisher
    logger.info("添加ROS控制指令发布器...")
    ros_cmd_vel = ROSCmdVelPublisher(
        topic_name=cfg.ROS_CMD_VEL_TOPIC,
        node_name='donkey_cmd_vel_pub',
        max_speed=cfg.ROS_MAX_SPEED,
        max_angular_velocity=cfg.ROS_MAX_ANGULAR_VELOCITY
    )
    V.add(ros_cmd_vel, inputs=['steering', 'throttle'], outputs=[])
    
    #
    # 数据记录（可选）
    #
    if cfg.AUTO_RECORD_ON_THROTTLE or cfg.RECORD_DURING_AI:
        inputs = ['cam/image_array', 'enc/speed']
        types = ['image_array', 'float']
        
        if cfg.ROS_USE_IMU:
            inputs.extend(['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
                          'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z'])
            types.extend(['float', 'float', 'float', 'float', 'float', 'float'])
        
        if cfg.ROS_USE_LIDAR:
            inputs.append('lidar/dist_array')
            types.append('nparray')
        
        if cfg.RECORD_DURING_AI:
            inputs.extend(['pilot/angle', 'pilot/throttle'])
            types.extend(['float', 'float'])
        else:
            inputs.extend(['user/angle', 'user/throttle'])
            types.extend(['float', 'float'])
        
        tub_path = TubHandler(path=cfg.DATA_PATH).create_tub_path() if \
            cfg.AUTO_CREATE_NEW_TUB else cfg.DATA_PATH
        meta += getattr(cfg, 'METADATA', [])
        tub_writer = TubWriter(tub_path, inputs=inputs, types=types, metadata=meta)
        
        V.add(tub_writer, inputs=inputs, outputs=["tub/num_records"], 
              run_condition='recording')
    
    #
    # 启动车辆循环
    #
    logger.info("启动ROS驱动循环...")
    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        logger.info("接收到中断信号，正在关闭...")
    finally:
        # 清理ROS资源
        logger.info("清理ROS资源...")
        if 'ros_camera' in locals():
            ros_camera.shutdown()
        if 'ros_odom' in locals():
            ros_odom.shutdown()
        if cfg.ROS_USE_IMU and 'ros_imu' in locals():
            ros_imu.shutdown()
        if cfg.ROS_USE_LIDAR and 'ros_scan' in locals():
            ros_scan.shutdown()
        if 'ros_cmd_vel' in locals():
            ros_cmd_vel.shutdown()
        logger.info("ROS驱动已关闭")


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    model_path = args.get('--model', None)
    model_type = args.get('--type', None)
    
    drive(cfg, model_path=model_path, model_type=model_type)
