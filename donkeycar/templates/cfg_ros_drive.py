"""
ROS驱动配置文件

用于配置donkeycar与ROS系统的集成，包括ROS话题名称、控制参数等。

使用示例:
    python manage.py drive --myconfig=cfg_ros_drive.py --model=models/my_model.h5
"""

import os

# PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

# VEHICLE
DRIVE_LOOP_HZ = 20      # 车辆循环频率（Hz）
MAX_LOOPS = None         # 最大循环次数，None表示无限循环

# CAMERA (ROS图像话题)
IMAGE_W = 160            # 图像宽度（像素）
IMAGE_H = 120            # 图像高度（像素）
IMAGE_DEPTH = 3          # 图像深度（RGB=3，灰度=1）

# ROS话题配置
# 图像话题
ROS_IMAGE_TOPIC = '/usb_cam/image_raw'  # ROS图像话题名称

# IMU话题（可选）
ROS_USE_IMU = False                      # 是否使用IMU数据
ROS_IMU_TOPIC = '/imu'                   # ROS IMU话题名称

# 里程计话题（必需）
ROS_ODOM_TOPIC = '/odom'                 # ROS里程计话题名称

# 激光雷达话题（可选，用于避障）
ROS_USE_LIDAR = False                    # 是否使用激光雷达数据
ROS_SCAN_TOPIC = '/scan'                 # ROS激光雷达话题名称

# 控制指令话题
ROS_CMD_VEL_TOPIC = '/cmd_vel'           # ROS控制指令话题名称
ROS_MAX_SPEED = 1.0                      # 最大线速度（m/s）
ROS_MAX_ANGULAR_VELOCITY = 1.0           # 最大角速度（rad/s）

# LOGGING
HAVE_CONSOLE_LOGGING = True
LOGGING_LEVEL = 'INFO'                   # 'NOTSET' / 'DEBUG' / 'INFO' / 'WARNING' / 'ERROR' / 'FATAL' / 'CRITICAL'
LOGGING_FORMAT = '%(message)s'

# TRAINING
DEFAULT_AI_FRAMEWORK = 'tensorflow'      # 'tensorflow' 或 'pytorch'
DEFAULT_MODEL_TYPE = 'linear'            # 'linear'|'categorical'|'rnn'|'imu'|'behavior'|'3d'|'localizer'|'latent'
CREATE_TF_LITE = True                    # 训练时自动创建tflite模型
CREATE_TENSOR_RT = False                 # 训练时自动创建tensorrt模型
SAVE_MODEL_AS_H5 = False                 # 是否使用旧的keras h5格式
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8
MAX_EPOCHS = 100
SHOW_PLOT = True
VERBOSE_TRAIN = True
USE_EARLY_STOP = True
EARLY_STOP_PATIENCE = 5
MIN_DELTA = .0005
PRINT_MODEL_SUMMARY = True
OPTIMIZER = None                         # 'adam', 'sgd', 'rmsprop'等，None使用默认
LEARNING_RATE = 0.001                    # 仅在指定OPTIMIZER时使用
LEARNING_RATE_DECAY = 0.0                # 仅在指定OPTIMIZER时使用
CACHE_IMAGES = True                      # 训练时缓存图像以加速
PRUNE_CNN = False
PRUNE_PERCENT_TARGET = 75
PRUNE_PERCENT_PER_ITERATION = 20
PRUNE_VAL_LOSS_DEGRADATION_LIMIT = 0.2
PRUNE_EVAL_PERCENT_OF_DATASET = .05

# 模型迁移选项
FREEZE_LAYERS = False
NUM_LAST_LAYERS_TO_TRAIN = 7

# 分类模型的最大油门范围
MODEL_CATEGORICAL_MAX_THROTTLE_RANGE = 0.8

# RNN或3D模型
SEQUENCE_LENGTH = 3

# 数据增强和变换
AUGMENTATIONS = []
TRANSFORMATIONS = []
AUG_BRIGHTNESS_RANGE = 0.2
AUG_BLUR_RANGE = (0, 3)
ROI_CROP_TOP = 45
ROI_CROP_BOTTOM = 0
ROI_CROP_RIGHT = 0
ROI_CROP_LEFT = 0
ROI_TRAPEZE_LL = 0
ROI_TRAPEZE_LR = 160
ROI_TRAPEZE_UL = 20
ROI_TRAPEZE_UR = 140
ROI_TRAPEZE_MIN_Y = 60
ROI_TRAPEZE_MAX_Y = 120

# RECORD OPTIONS
RECORD_DURING_AI = False                 # 是否在AI模式下记录数据
AUTO_CREATE_NEW_TUB = False              # 记录时是否创建新的tub目录
AUTO_RECORD_ON_THROTTLE = False          # 油门非零时自动记录（ROS模式下通常禁用）

# WEB CONTROL
CONTROLLER_TYPE = 'web'                  # 使用Web控制器
WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))
WEB_INIT_MODE = "user"                   # 'user'|'local_angle'|'local'

# DRIVING
AI_THROTTLE_MULT = 1.0                   # AI油门倍数，用于缩放所有模型输出的油门值
AI_LAUNCH_DURATION = 1.0                 # AI启动加速持续时间（秒）
AI_LAUNCH_THROTTLE = 0.5                 # AI启动时的油门值
AI_LAUNCH_KEEP_ENABLED = False           # AI启动后是否保持启用

# BEHAVIOR (可选)
TRAIN_BEHAVIORS = False
BEHAVIOR_LIST = []

# LOCALIZER (可选)
TRAIN_LOCALIZER = False

# SHOW PILOT IMAGE
SHOW_PILOT_IMAGE = False                 # 在自动驾驶模式下显示用于推理的图像
