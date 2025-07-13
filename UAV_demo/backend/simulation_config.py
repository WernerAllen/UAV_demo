# 文件: backend/simulation_config.py
# 描述: 存放所有全局配置和仿真参数 (增加了方向持续时间配置)

# 环境边界 (单位: 米)
MAX_X = 600
MAX_Y = 600
MAX_Z = 0

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
DEFAULT_TIME_INCREMENT = 0.1  # 每个仿真步长代表的秒数

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

# DHyTP模型开关，True则使用新模型计算路径权重，False则使用旧的地理距离
USE_DHYTP_ROUTING_MODEL = True

# 并发区域检测阈值 
CONCURRENCY_DISTANCE_THRESHOLD = 30.0  # (d_max) 判断并发的最大距离 (米)
CONCURRENCY_ANGLE_THRESHOLD = 25.0   # (theta_max) 判断并发的最大夹角 (度)

# EoD模型所需参数
AVG_ONE_HOP_DISTANCE = 80.0 # (l_h^ave) 平均单跳通信距离，可以略小于通信范围
GRID_TRANSMISSION_ERROR = {   # (delta^t(z)) 每个网格的累积传输时间误差(秒)
    (0, 0): 0.01, (0, 1): 0.02, (0, 2): 0.01,
    (1, 0): 0.03, (1, 1): 0.10, (1, 2): 0.03,
    (2, 0): 0.01, (2, 1): 0.02, (2, 2): 0.01,
}

# 重传与干扰模型配置
MAX_RETRANSMISSIONS = 100

# ## **** MODIFICATION START: 位置变动检测配置 **** ##
# 位置变动检测阈值 (米)
POSITION_CHANGE_THRESHOLD = 2.5  # 检测下一跳节点位置变动的阈值
# ## **** MODIFICATION END **** ##



DB_CONFIG = {
    "host": "localhost",
    "user": "root",
    "password": "123456",
    "database": "uav_simulation"
}

# API 相关
API_DEFAULT_PORT = 5001
