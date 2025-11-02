# 文件: backend/simulation_config.py
# 描述: 存放所有全局配置和仿真参数 (增加了方向持续时间配置)

# ==================== 随机种子配置 ====================
# 用于固定无人机分布和移动模式，降低测试误差，提高实验可重复性
RANDOM_SEED_ENABLED = False  # 是否启用固定随机种子
RANDOM_SEED = 24  # 随机种子值
                  # 可选值：任意整数（如：42, 123, 2024等）
                  
# 使用说明：
# 1. RANDOM_SEED_ENABLED = True: 每次实验使用相同的初始分布
#    - 优点：实验结果可重复，便于对比不同协议/参数的性能差异
#    - 适用：性能对比测试、参数调优
# 2. RANDOM_SEED_ENABLED = False: 每次实验使用完全随机的分布
#    - 优点：测试更广泛的场景，避免过拟合特定分布
#    - 适用：鲁棒性测试、多场景验证
# 
# 影响范围：
# - 无人机初始位置 (x, y, z坐标)
# - 无人机初始飞行方向
# - 无人机方向改变的时间点
# - 所有协议中使用random的地方

# 环境边界 (单位: 米)
MAX_X = 600
MAX_Y = 600
MAX_Z = 0

# 无人机移动参数
MIN_SPEED = 18  # 最低速度 (米/秒)
MAX_SPEED = 20  # 最高速度 (米/秒)
UAV_MOVE_STEP_SIZE = 5  # 无人机每步移动距离 (米)

## **** MODIFICATION START: 新增方向持续时间配置 **** ##
# 无人机将沿一个方向飞行的最短和最长时间 (秒)
MIN_DIRECTION_TIME = 1.0  # 最少沿一个方向飞x秒
MAX_DIRECTION_TIME = 5.0  # 最多沿一个方向飞y秒
## **** MODIFICATION END **** ##

# 仿真默认参数
DEFAULT_NUM_UAVS = 200
DEFAULT_TIME_INCREMENT = 0.1  # 每个仿真步长代表的秒数

# 无人机通信
UAV_COMMUNICATION_RANGE = 100  # 通信范围 (米)

# PRR网格化配置
GRID_ROWS = 3
GRID_COLS = 3
PRR_GRID_MAP = [
    [0.75, 0.90, 0.90],
    [0.85, 0.95, 0.85],
    [0.80, 0.90, 0.85]
]

# PTP协议PRR自定义配置
PTP_GRID_ROWS = 10  # PTP协议使用的网格行数
PTP_GRID_COLS = 10  # PTP协议使用的网格列数
PRR_MIN = 0.5  # PRR最小值（全局通用）
PRR_MAX = 0.9  # PRR最大值（全局通用）
PTP_USE_RANDOM_PRR = True  # 是否使用随机PRR，True则使用随机值，False则使用PRR_GRID_MAP

# PRR丢包判定开关，True则启用PRR丢包/重传，False则只用EoD，不再PRR丢包
USE_PRR_FAILURE_MODEL = True

# 路由模型选择: "DHYTP", "MTP", "PTP", "NONE"
ROUTING_MODEL = "PTP"

# 兼容性变量 - 基于ROUTING_MODEL自动设置
USE_DHYTP_ROUTING_MODEL = ROUTING_MODEL == "DHYTP"
USE_MTP_ROUTING_MODEL = ROUTING_MODEL == "MTP"
USE_PTP_ROUTING_MODEL = ROUTING_MODEL == "PTP"



# 并发区域检测阈值 
CONCURRENCY_DISTANCE_THRESHOLD = 20.0  # (d_max) 判断并发的最大距离 (米)
CONCURRENCY_ANGLE_THRESHOLD = 25.0   # (theta_max) 判断并发的最大夹角 (度)

# EoD模型所需参数
AVG_ONE_HOP_DISTANCE = 80.0 # (l_h^ave) 平均单跳通信距离，可以略小于通信范围
GRID_TRANSMISSION_ERROR = {   # (delta^t(z)) 每个网格的累积传输时间误差(秒)
    (0, 0): 0.01, (0, 1): 0.02, (0, 2): 0.01,
    (1, 0): 0.03, (1, 1): 0.05, (1, 2): 0.03,
    (2, 0): 0.01, (2, 1): 0.02, (2, 2): 0.01,
}

# 重传次数上限
MAX_RETRANSMISSIONS = 10

# 位置变动检测阈值 (米)
POSITION_CHANGE_THRESHOLD = 1.5  # 检测下一跳节点位置变动的阈值


COLLECT_ENERGY_STATS = True             # 是否收集能耗统计信息

# 能源成本模型配置
# 基础能耗参数
ENERGY_UNIT_SEND = 0.5      # 每发送一个数据包的基础能耗
ENERGY_UNIT_RECEIVE = 0.1   # 每接收一个数据包的基础能耗
ENERGY_RETRANSMISSION_PENALTY = 1.5    # 重传能耗惩罚系数


# 协议特定能耗配置
PROTOCOL_ENERGY_CONFIG = {
    "PTP": {
        "ROUTE_DISCOVERY": 0.8,        # 每次路由发现的能耗/数据包（点对点路径计算）
    },
    "MTP": {
        "TREE_CREATION": 1.0,          # 创建树结构的初始能耗/数据包
        "TREE_MAINTENANCE": 0.5,       # 每次树维护操作的能耗
    },
    "DHYTP": {
        "TREE_CREATION": 1.0,          # 创建树结构的初始能耗/数据包
        "TREE_MAINTENANCE": 0.5,       # 每次树维护操作的能耗（与MTP保持一致）
        "PHASE_TRANSITION": 0.01,       # 从PTP阶段过渡到MTP阶段的能耗/数据包
    }
}


# 能耗计算方法:
# 1. 数据包传输能耗 = ENERGY_UNIT_SEND (固定值)
# 2. 数据包接收能耗 = ENERGY_UNIT_RECEIVE (固定值)
# 3. 重传能耗 = (传输能耗 * ENERGY_RETRANSMISSION_PENALTY)
# 4. 协议操作能耗 = 对应协议操作的能耗值 (来自PROTOCOL_ENERGY_CONFIG)
#    - PTP: 每个数据包需要ROUTE_DISCOVERY能耗（点对点路由发现）
#    - MTP: 每个数据包需要TREE_CREATION能耗 + 分摊的TREE_MAINTENANCE能耗
#    - DHYTP: 每个数据包需要TREE_CREATION + PHASE_TRANSITION能耗 + 分摊的TREE_MAINTENANCE能耗
#
# 总能耗计算方式:
# - 每个数据包在Packet类中添加energy_consumed属性，记录传输过程中累积的能耗
# - 在每次跳转、计算和协议操作时累加相应的能耗

# 树剪枝机制配置参数
TREE_PRUNING_ENABLED = True  # 是否启用树剪枝机制

# 椭圆参数配置（影响剪枝效果和性能）
ELLIPSE_ECCENTRICITY = 0.7   # 椭圆偏心率 (0 < e < 1) 
                             # 0.6=保守 | 0.7=平衡 | 0.8=激进
ELLIPSE_EXPANSION_FACTOR = 1.15  # 椭圆扩展因子，用于扩大搜索范围
                                 # 1.1=激进 | 1.15=平衡 | 1.3=保守
ELLIPSE_BOUNDARY_TOLERANCE = 4.0  # 椭圆边界容差(米)，用于判断节点是否在椭圆内
                                  # 3.0=严格 | 4.0=平衡 | 8.0=宽松

# 更新频率配置（影响实时性和计算开销）
PRUNING_UPDATE_INTERVAL = 0.4  # ETX更新间隔时间(秒)
                               # 0.3=高频 | 0.4=平衡 | 0.6=低频

# 树剪枝能耗节省配置
PRUNING_ENERGY_SAVING = 0.8  # 树剪枝带来的能耗节省比例（0-1之间）
                             # 0.5=保守 | 0.8=平衡 | 0.95=激进
                             # 计算方式：节省 = 剪枝率 × 基础能耗 × 节省比例
                             # 注意：节省的能耗不会超过 (剪枝率 × 基础能耗)

# 路径合并优化配置
PATH_MERGE_ENABLED = False  # 是否启用路径合并优化
PATH_MERGE_DISTANCE_THRESHOLD = 30.0  # 相邻路径段的平均距离阈值(米)
PATH_MERGE_MIN_SEGMENT_LENGTH = 1  # 可合并的最小路径段长度（跳数）
PATH_MERGE_ENERGY_SAVING = 0.5  # 路径合并带来的能耗节省系数（相对于树维护能耗）
PATH_MERGE_MAX_SEGMENT_LENGTH = 20  # 最大路径段长度（跳数），限制计算复杂度
PATH_MERGE_MAX_MERGES = 20  # 最大合并数量，避免过度合并
PATH_MERGE_AVERAGE_PATHS_PER_GROUP = 2.5  # 每个合并群组平均包含的路径段数量（用于能耗计算）

# 群组数随机化配置（基于实验规模）
PATH_MERGE_GROUP_COUNT_ENABLED = True  # 是否启用群组数随机化
PATH_MERGE_GROUP_COUNT_UAV_RATIO_MIN = 0.05  # 群组数占UAV数的最小比例（群组/UAV）
PATH_MERGE_GROUP_COUNT_UAV_RATIO_MAX = 0.1  # 群组数占UAV数的最大比例（群组/UAV）
PATH_MERGE_GROUP_COUNT_PACKET_RATIO_MIN = 0.2  # 群组数占数据包数的最小比例（群组/包）
PATH_MERGE_GROUP_COUNT_PACKET_RATIO_MAX = 1  # 群组数占数据包数的最大比例（群组/包）
PATH_MERGE_GROUP_COUNT_WEIGHT_UAV = 0.6  # UAV数量的权重（0-1之间）
PATH_MERGE_GROUP_COUNT_WEIGHT_PACKET = 0.4  # 数据包数量的权重（0-1之间）
                                          

# - 仿真结束后计算: 总能耗 / 成功传输的数据包数 = 平均每包能耗





DB_CONFIG = {
    "host": "localhost",
    "user": "root",
    "password": "123456",
    "database": "uav_simulation"
}

# API 相关
API_DEFAULT_PORT = 5001
