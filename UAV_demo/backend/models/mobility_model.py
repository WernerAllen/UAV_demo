# backend/models/mobility_model.py
# 封装移动逻辑
# 只关心如何计算移动。

import random
import math
from simulation_config import MAX_X, MAX_Y, MAX_Z, UAV_MOVE_STEP_SIZE


class BaseMobilityModel:
    def __init__(self, uav_instance):
        self.uav = uav_instance

    def move(self, time_increment):
        raise NotImplementedError("Subclasses must implement this method")


class RandomWaypointMobility(BaseMobilityModel):
    def __init__(self, uav_instance):
        super().__init__(uav_instance)
        self.target_x: float = 0.0
        self.target_y: float = 0.0
        self.target_z: float = 0.0
        self._get_new_waypoint()

    def _get_new_waypoint(self):
        self.target_x = random.uniform(0, MAX_X)
        self.target_y = random.uniform(0, MAX_Y)
        self.target_z = random.uniform(0, MAX_Z)  # 简单处理Z轴

    def move(self, time_increment):  # time_increment 目前在这个简单模型中未使用，但保留接口
        # 简化：直接向目标点小幅度移动，如果到达则选择新路点
        # 在实际模型中，会基于速度和 time_increment 计算移动距离

        current_step_size = random.uniform(UAV_MOVE_STEP_SIZE / 2, UAV_MOVE_STEP_SIZE)

        # 简单向目标移动，实际应用中需要基于速度和时间增量
        if self.target_x is not None:
            dx = self.target_x - self.uav.x
            dy = self.target_y - self.uav.y
            dz = self.target_z - self.uav.z
            distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

            if distance < current_step_size:  # 到达或非常接近目标点
                self.uav.x = self.target_x
                self.uav.y = self.target_y
                self.uav.z = self.target_z
                self._get_new_waypoint()
            else:  # 向目标点移动
                self.uav.x += (dx / distance) * current_step_size
                self.uav.y += (dy / distance) * current_step_size
                self.uav.z += (dz / distance) * current_step_size
        else:  # 第一次移动或目标点未设置
            self._get_new_waypoint()

        # 确保不出界 (虽然移动逻辑本身会尝试趋向边界内的目标点)
        # 确保坐标是数值类型
        x_coord = float(self.uav.x) if self.uav.x is not None else 0.0
        y_coord = float(self.uav.y) if self.uav.y is not None else 0.0
        z_coord = float(self.uav.z) if self.uav.z is not None else 0.0
        
        self.uav.x = max(0.0, min(x_coord, float(MAX_X)))
        self.uav.y = max(0.0, min(y_coord, float(MAX_Y)))
        self.uav.z = max(0.0, min(z_coord, float(MAX_Z)))


class StationaryMobility(BaseMobilityModel):  # 示例：静止模型
    def move(self, time_increment):
        pass  # Do nothing, UAV stays in place