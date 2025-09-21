# 文件: backend/core/uav.py
# 描述: 移除了不再需要的MAC层退避计时器

import random
import math
from collections import deque
from models.mobility_model import RandomWaypointMobility
from simulation_config import MAX_X, MAX_Y, MIN_SPEED, MAX_SPEED, MIN_DIRECTION_TIME, MAX_DIRECTION_TIME, TREE_PRUNING_ENABLED, ELLIPSE_ECCENTRICITY, ELLIPSE_EXPANSION_FACTOR, ELLIPSE_BOUNDARY_TOLERANCE


class UAV:
    _id_counter = 0

    @classmethod
    def _generate_id(cls):
        cls._id_counter += 1
        return cls._id_counter

    @classmethod
    def reset_id_counter(cls):
        cls._id_counter = 0

    def __init__(self, color):
        self.id = self._generate_id()
        self.x = random.uniform(0, MAX_X)
        self.y = random.uniform(0, MAX_Y)
        self.z = random.uniform(20, 80)
        self.color = color
        self.tx_queue = deque()
        self.is_transmitting_now = False

        self.current_angle = 0.0
        self.time_to_change_direction = 0.0
        self._set_new_direction()

        # ## **** MODIFICATION: 移除了 mac_backoff_timer **** ##
        # 新的冲突处理机制不再需要单个无人机的退避计时器
        
    def _set_new_direction(self):
        """设置一个新的随机飞行方向和持续时间"""
        self.current_angle = random.uniform(0, 2 * math.pi)
        self.time_to_change_direction = random.uniform(MIN_DIRECTION_TIME, MAX_DIRECTION_TIME)

    def update_state(self, time_increment):
        # ## **** MODIFICATION: 移除了退避计时器的更新逻辑 **** ##
        
        self.time_to_change_direction -= time_increment
        if self.time_to_change_direction <= 0:
            self._set_new_direction()

        speed = random.uniform(MIN_SPEED, MAX_SPEED)
        move_dist = speed * time_increment

        delta_x = move_dist * math.cos(self.current_angle)
        delta_y = move_dist * math.sin(self.current_angle)

        self.x += delta_x
        self.y += delta_y

        if self.x <= 0 or self.x >= MAX_X:
            self.x = max(0, min(self.x, MAX_X))
            self.current_angle = math.pi - self.current_angle
        if self.y <= 0 or self.y >= MAX_Y:
            self.y = max(0, min(self.y, MAX_Y))
            self.current_angle = -self.current_angle

        self.current_angle %= (2 * math.pi)

    def add_packet_to_queue(self, packet):
        self.tx_queue.append(packet)

    def get_data_for_api(self):
        return {
            "id": self.id,
            "x": self.x, "y": self.y, "z": self.z,
            "color": self.color, "tx_queue_size": len(self.tx_queue)
        }
    
    def is_within_ellipse_region(self, source_uav, destination_uav):
        """
        判断当前UAV是否在源节点和目标节点定义的椭圆区域内
        
        Args:
            source_uav: 源节点UAV对象
            destination_uav: 目标节点UAV对象
            
        Returns:
            bool: True表示在椭圆区域内，False表示在区域外
        """
        if not TREE_PRUNING_ENABLED:
            return True  # 如果未启用树剪枝，所有节点都参与树构建
            
        # 计算椭圆参数
        # 焦点距离 = 源节点到目标节点的距离
        focal_distance = math.sqrt(
            (source_uav.x - destination_uav.x) ** 2 + 
            (source_uav.y - destination_uav.y) ** 2 + 
            (source_uav.z - destination_uav.z) ** 2
        )
        
        # 如果焦点距离为0，说明源节点和目标节点重合，返回True
        if focal_distance < 1e-6:
            return True
            
        # 椭圆半长轴 a = focal_distance / (2 * eccentricity)
        # 椭圆半短轴 b = a * sqrt(1 - e^2)
        a = focal_distance / (2 * ELLIPSE_ECCENTRICITY)
        b = a * math.sqrt(1 - ELLIPSE_ECCENTRICITY ** 2)
        
        # 应用扩展因子
        a *= ELLIPSE_EXPANSION_FACTOR
        b *= ELLIPSE_EXPANSION_FACTOR
        
        # 计算当前节点到两个焦点的距离之和
        dist_to_source = math.sqrt(
            (self.x - source_uav.x) ** 2 + 
            (self.y - source_uav.y) ** 2 + 
            (self.z - source_uav.z) ** 2
        )
        dist_to_destination = math.sqrt(
            (self.x - destination_uav.x) ** 2 + 
            (self.y - destination_uav.y) ** 2 + 
            (self.z - destination_uav.z) ** 2
        )
        
        # 椭圆定义：到两个焦点距离之和 <= 2a
        # 添加边界容差
        ellipse_boundary = 2 * a + ELLIPSE_BOUNDARY_TOLERANCE
        
        return (dist_to_source + dist_to_destination) <= ellipse_boundary
    
    def calculate_ellipse_utility(self, source_uav, destination_uav):
        """
        计算当前节点在椭圆区域内的效用值，用于树构建时的优先级排序
        
        Args:
            source_uav: 源节点UAV对象
            destination_uav: 目标节点UAV对象
            
        Returns:
            float: 效用值，值越大表示越适合作为树节点
        """
        if not self.is_within_ellipse_region(source_uav, destination_uav):
            return 0.0  # 不在椭圆区域内，效用为0
            
        # 计算到源节点和目标节点的距离
        dist_to_source = math.sqrt(
            (self.x - source_uav.x) ** 2 + 
            (self.y - source_uav.y) ** 2 + 
            (self.z - source_uav.z) ** 2
        )
        dist_to_destination = math.sqrt(
            (self.x - destination_uav.x) ** 2 + 
            (self.y - destination_uav.y) ** 2 + 
            (self.z - destination_uav.z) ** 2
        )
        
        # 效用值计算：距离源节点和目标节点越近，效用越高
        # 使用调和平均数来平衡两个距离
        if dist_to_source + dist_to_destination < 1e-6:
            return 1.0
            
        utility = 2.0 / (dist_to_source + dist_to_destination)
        return utility
    
    # 添加比较方法以支持排序操作，避免递归深度错误
    def __lt__(self, other):
        """小于比较，基于UAV ID"""
        if not isinstance(other, UAV):
            return NotImplemented
        return self.id < other.id
    
    def __le__(self, other):
        """小于等于比较，基于UAV ID"""
        if not isinstance(other, UAV):
            return NotImplemented
        return self.id <= other.id
    
    def __gt__(self, other):
        """大于比较，基于UAV ID"""
        if not isinstance(other, UAV):
            return NotImplemented
        return self.id > other.id
    
    def __ge__(self, other):
        """大于等于比较，基于UAV ID"""
        if not isinstance(other, UAV):
            return NotImplemented
        return self.id >= other.id
    
    def __eq__(self, other):
        """相等比较，基于UAV ID"""
        if not isinstance(other, UAV):
            return NotImplemented
        return self.id == other.id
    
    def __ne__(self, other):
        """不等比较，基于UAV ID"""
        if not isinstance(other, UAV):
            return NotImplemented
        return self.id != other.id
    
    def __hash__(self):
        """哈希方法，基于UAV ID"""
        return hash(self.id)