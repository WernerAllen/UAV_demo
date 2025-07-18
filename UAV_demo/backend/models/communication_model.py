# 文件: backend/models/communication_model.py
# 描述: 实现PRR, 移动性, 干扰等通信失败模型

import random
import math
from simulation_config import *
import numpy as np


class CommunicationModel:
    def check_prr_failure(self, receiver_uav):
        import simulation_config
        if not getattr(simulation_config, 'USE_PRR_FAILURE_MODEL', True):
            return False
        if not (0 <= receiver_uav.x < MAX_X and 0 <= receiver_uav.y < MAX_Y):
            return True
        cell_width = MAX_X / GRID_COLS
        cell_height = MAX_Y / GRID_ROWS
        col_index = min(int(receiver_uav.x / cell_width), GRID_COLS - 1)
        row_index = min(int(receiver_uav.y / cell_height), GRID_COLS - 1)
        regional_prr = PRR_GRID_MAP[row_index][col_index]
        return random.random() > regional_prr

    


# ## **** MODIFICATION START: 新增路由模型估算类 **** ##
class RoutingModel:
    """
    实现论文中提出的PTP相关估算模型
    """
    def __init__(self, uav_map):
        self.uav_map = uav_map

    def get_grid_cell(self, x, y):
        """根据坐标获取所在网格的索引"""
        if not (0 <= x < MAX_X and 0 <= y < MAX_Y):
            return None, None
        cell_width = MAX_X / GRID_COLS
        cell_height = MAX_Y / GRID_ROWS
        col = min(int(x / cell_width), GRID_COLS - 1)
        row = min(int(y / cell_height), GRID_COLS - 1)
        return row, col

    def calculate_eod_for_grid(self, distance_in_grid, grid_row, grid_col):
        """计算单个网格的EoD (公式16)"""
        if AVG_ONE_HOP_DISTANCE == 0 or PRR_GRID_MAP[grid_row][grid_col] == 0:
            return float('inf')
        
        prr = PRR_GRID_MAP[grid_row][grid_col]
        error_term = GRID_TRANSMISSION_ERROR.get((grid_row, grid_col), 0.01)

        # EoD^t(z, u_i, u_j) = l(z) / (l_h^ave * prr(z)) + delta(z)
        estimated_time = (distance_in_grid / (AVG_ONE_HOP_DISTANCE * prr)) + error_term
        return estimated_time

    def get_link_base_delay(self, uav1, uav2):
        """计算一条链路的基础延迟（不含并发）"""
        # 简单实现：将链路看作直线，估算其经过每个网格的延迟并求和
        # 注意：这是一个简化实现，精确计算直线在网格中的长度会更复杂
        dist = math.sqrt((uav1.x - uav2.x)**2 + (uav1.y - uav2.y)**2)
        row1, col1 = self.get_grid_cell(uav1.x, uav1.y)
        row2, col2 = self.get_grid_cell(uav2.x, uav2.y)

        if row1 is None or row2 is None:
            return float('inf')

        # 简化：如果跨网格，我们取两个网格延迟的平均值乘以总距离
        # 实际应用中可能需要更精确的几何计算
        delay1 = self.calculate_eod_for_grid(dist, row1, col1)
        if (row1, col1) != (row2, col2):
            delay2 = self.calculate_eod_for_grid(dist, row2, col2)
            return (delay1 + delay2) / 2
        return delay1

    def are_vectors_concurrent(self, p1, q1, p2, q2):
        """判断两个向量是否并发 (距离和角度)"""
        p1, q1, p2, q2 = map(np.array, [p1, q1, p2, q2])

        # 检查距离 (使用简化的线段中心点距离，实际应用可用更精确的线段最短距离算法)
        center1 = (p1 + q1) / 2
        center2 = (p2 + q2) / 2
        min_dist = np.linalg.norm(center1 - center2)

        if min_dist >= CONCURRENCY_DISTANCE_THRESHOLD:
            return False

        # 检查角度
        u = q1 - p1
        v = q2 - p2
        cos_theta = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
        angle = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))

        return angle < CONCURRENCY_ANGLE_THRESHOLD

    # ## **** MODIFICATION START: 新增辅助函数并重写并发延迟计算 **** ##
    def _get_grids_and_lengths_for_line(self, p1, p2):
        """
        获取一条线段(p1 -> p2)穿过的所有网格及其在每个网格内的实际穿越距离。
        返回: dict {(row, col): length_in_grid}
        """
        cell_width = MAX_X / GRID_COLS
        cell_height = MAX_Y / GRID_ROWS
        x1, y1 = p1
        x2, y2 = p2
        grids = {}
        # 采样足够密集，累加每个网格内的距离
        total_dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(total_dist / min(cell_width, cell_height) * 10), 1)
        prev_row, prev_col = None, None
        prev_x, prev_y = x1, y1
        for i in range(1, steps + 1):
            t = i / steps
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            row, col = self.get_grid_cell(x, y)
            if row is not None:
                if (row, col) not in grids:
                    grids[(row, col)] = 0.0
                # 累加上一步到这一步的距离
                seg_len = math.hypot(x - prev_x, y - prev_y)
                grids[(row, col)] += seg_len
            prev_x, prev_y = x, y
        return grids

    def calculate_concurrent_region_delay(self, vec1_p1, vec1_q1, vec2_p2, vec2_q2):
        """
        估算并发区域的延迟 (公式17的实现)。
        """
        # 1. 找出两个向量各自穿过的网格集合及长度
        grids1 = self._get_grids_and_lengths_for_line(vec1_p1, vec1_q1)
        grids2 = self._get_grids_and_lengths_for_line(vec2_p2, vec2_q2)
        # 2. 求交集，得到并发区域的网格 z
        concurrent_grids = set(grids1.keys()).intersection(grids2.keys())
        if not concurrent_grids:
            return 0.0
        total_concurrent_delay = 0.0
        # 3. 遍历并发区域的每个网格z，累加其EoD（用实际穿越距离）
        for row, col in concurrent_grids:
            length = min(grids1[(row, col)], grids2[(row, col)])  # 保守估计，取较小穿越长度
            total_concurrent_delay += self.calculate_eod_for_grid(length, row, col)
        return total_concurrent_delay