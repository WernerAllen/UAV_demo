# 文件: backend/simulation_config.py
# 描述: 存放所有全局配置和仿真参数 (增加了方向持续时间配置)

# 环境边界 (单位: 米)
MAX_X = 600
MAX_Y = 600
MAX_Z = 100

# 无人机移动参数
MIN_SPEED = 3  # 最低速度 (米/秒)
MAX_SPEED = 5  # 最高速度 (米/秒)
UAV_MOVE_STEP_SIZE = 5  # 无人机每步移动距离 (米)

# ## **** MODIFICATION START: 新增方向持续时间配置 **** ##
# 无人机将沿一个方向飞行的最短和最长时间 (秒)
MIN_DIRECTION_TIME = 2.0  # 最少沿一个方向飞2秒
MAX_DIRECTION_TIME = 10.0  # 最多沿一个方向飞10秒
# ## **** MODIFICATION END **** ##

# 仿真默认参数
DEFAULT_NUM_UAVS = 150
DEFAULT_TIME_INCREMENT = 1  # 每个仿真步长代表0.1秒

# 无人机通信
UAV_COMMUNICATION_RANGE = 100  # 通信范围 (米)

# PRR网格化配置
GRID_ROWS = 3
GRID_COLS = 3
PRR_GRID_MAP = [
    [0.95, 0.90, 0.95],
    [0.85, 0.60, 0.85],
    [0.95, 0.90, 0.95]
]

# 重传与干扰模型配置
MAX_RETRANSMISSIONS = 20
TRANSMIT_POWER_WATTS = 0.1
PATH_LOSS_EXPONENT = 2.7
NOISE_POWER_WATTS = 1e-12
SINR_THRESHOLD = 10


DB_CONFIG = {
    "host": "localhost",
    "user": "root",
    "password": "123456",
    "database": "uav_simulation"
}

# API 相关
API_DEFAULT_PORT = 5001
