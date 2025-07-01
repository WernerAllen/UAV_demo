# 文件: backend/models/communication_model.py
# 描述: 实现PRR, 移动性, 干扰等通信失败模型

import random
import math
from simulation_config import *


class CommunicationModel:
    def check_prr_failure(self, receiver_uav):
        if not (0 <= receiver_uav.x < MAX_X and 0 <= receiver_uav.y < MAX_Y):
            return True
        cell_width = MAX_X / GRID_COLS
        cell_height = MAX_Y / GRID_ROWS
        col_index = min(int(receiver_uav.x / cell_width), GRID_COLS - 1)
        row_index = min(int(receiver_uav.y / cell_height), GRID_ROWS - 1)
        regional_prr = PRR_GRID_MAP[row_index][col_index]
        return random.random() > regional_prr

    def check_mobility_failure(self, sender_uav, receiver_uav):
        distance_sq = (sender_uav.x - receiver_uav.x) ** 2 + (sender_uav.y - receiver_uav.y) ** 2 + (
                    sender_uav.z - receiver_uav.z) ** 2
        return distance_sq > UAV_COMMUNICATION_RANGE ** 2

    def _calculate_path_loss_gain(self, distance):
        if distance == 0: return 1.0
        distance = max(1.0, distance)
        return distance ** -PATH_LOSS_EXPONENT

    def calculate_sinr(self, sender_uav, receiver_uav, interfering_uavs):
        distance_to_receiver = math.sqrt((sender_uav.x - receiver_uav.x) ** 2 + (sender_uav.y - receiver_uav.y) ** 2)
        signal_gain = self._calculate_path_loss_gain(distance_to_receiver)
        signal_power = TRANSMIT_POWER_WATTS * signal_gain

        total_interference_power = 0.0
        for interferer in interfering_uavs:
            distance_to_interferer = math.sqrt(
                (interferer.x - receiver_uav.x) ** 2 + (interferer.y - receiver_uav.y) ** 2)
            interference_gain = self._calculate_path_loss_gain(distance_to_interferer)
            total_interference_power += TRANSMIT_POWER_WATTS * interference_gain

        return signal_power / (total_interference_power + NOISE_POWER_WATTS)

    def check_interference_failure(self, sender_uav, receiver_uav, concurrent_transmitters):
        interferers = [uav for uav in concurrent_transmitters if uav.id != sender_uav.id]
        sinr = self.calculate_sinr(sender_uav, receiver_uav, interferers)
        return sinr < SINR_THRESHOLD