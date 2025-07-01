# 文件: backend/core/uav.py
# 描述: 移除了不再需要的MAC层退避计时器

import random
import math
from collections import deque
from models.mobility_model import RandomWaypointMobility
from simulation_config import MAX_X, MAX_Y, MIN_SPEED, MAX_SPEED, MIN_DIRECTION_TIME, MAX_DIRECTION_TIME


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
