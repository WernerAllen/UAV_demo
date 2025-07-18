# 文件: backend/protocols/ptp_protocol.py
# 描述: PTP 路由协议实现 (原 RoutingModel)

import math
import numpy as np
from simulation_config import *

class RoutingModel:
    """
    实现论文中提出的PTP相关估算模型
    """
    def __init__(self, uav_map):
        self.uav_map = uav_map

    def get_grid_cell(self, x, y):
        if not (0 <= x < MAX_X and 0 <= y < MAX_Y):
            return None, None
        cell_width = MAX_X / GRID_COLS
        cell_height = MAX_Y / GRID_ROWS
        col = min(int(x / cell_width), GRID_COLS - 1)
        row = min(int(y / cell_height), GRID_ROWS - 1)
        return row, col

    def calculate_eod_for_grid(self, distance_in_grid, grid_row, grid_col):
        if AVG_ONE_HOP_DISTANCE == 0 or PRR_GRID_MAP[grid_row][grid_col] == 0:
            return float('inf')
        prr = PRR_GRID_MAP[grid_row][grid_col]
        error_term = GRID_TRANSMISSION_ERROR.get((grid_row, grid_col), 0.01)
        estimated_time = (distance_in_grid / (AVG_ONE_HOP_DISTANCE * prr)) + error_term
        return estimated_time

    def get_link_base_delay(self, uav1, uav2):
        dist = math.sqrt((uav1.x - uav2.x)**2 + (uav1.y - uav2.y)**2)
        row1, col1 = self.get_grid_cell(uav1.x, uav1.y)
        row2, col2 = self.get_grid_cell(uav2.x, uav2.y)
        if row1 is None or row2 is None:
            return float('inf')
        delay1 = self.calculate_eod_for_grid(dist, row1, col1)
        if (row1, col1) != (row2, col2):
            delay2 = self.calculate_eod_for_grid(dist, row2, col2)
            return (delay1 + delay2) / 2
        return delay1

    def are_vectors_concurrent(self, p1, q1, p2, q2):
        p1, q1, p2, q2 = map(np.array, [p1, q1, p2, q2])
        center1 = (p1 + q1) / 2
        center2 = (p2 + q2) / 2
        min_dist = np.linalg.norm(center1 - center2)
        if min_dist >= CONCURRENCY_DISTANCE_THRESHOLD:
            return False
        u = q1 - p1
        v = q2 - p2
        cos_theta = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
        angle = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))
        return angle < CONCURRENCY_ANGLE_THRESHOLD

    def _get_grids_and_lengths_for_line(self, p1, p2):
        cell_width = MAX_X / GRID_COLS
        cell_height = MAX_Y / GRID_ROWS
        x1, y1 = p1
        x2, y2 = p2
        grids = {}
        total_dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(total_dist / min(cell_width, cell_height) * 10), 1)
        prev_x, prev_y = x1, y1
        for i in range(1, steps + 1):
            t = i / steps
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            row, col = self.get_grid_cell(x, y)
            if row is not None:
                if (row, col) not in grids:
                    grids[(row, col)] = 0.0
                seg_len = math.hypot(x - prev_x, y - prev_y)
                grids[(row, col)] += seg_len
            prev_x, prev_y = x, y
        return grids

    def calculate_concurrent_region_delay(self, vec1_p1, vec1_q1, vec2_p2, vec2_q2):
        grids1 = self._get_grids_and_lengths_for_line(vec1_p1, vec1_q1)
        grids2 = self._get_grids_and_lengths_for_line(vec2_p2, vec2_q2)
        concurrent_grids = set(grids1.keys()).intersection(grids2.keys())
        if not concurrent_grids:
            return 0.0
        total_concurrent_delay = 0.0
        for row, col in concurrent_grids:
            length = min(grids1[(row, col)], grids2[(row, col)])
            total_concurrent_delay += self.calculate_eod_for_grid(length, row, col)
        return total_concurrent_delay 

    def calculate_expected_transmission_time(self, from_uav, to_uav, dest_uav, all_sending_vectors=None):
        """
        计算from_uav到to_uav的EoD+并发区域延迟，dest_uav为目标节点。
        all_sending_vectors: [(p1, q1), ...] 当前网络所有正在发送的向量
        """
        # 1. 基础EoD
        base_eod = self.get_link_base_delay(from_uav, dest_uav)
        # 2. 并发区域延迟
        delta_pred = 0.0
        if all_sending_vectors is not None:
            my_vec = ((from_uav.x, from_uav.y), (to_uav.x, to_uav.y))
            for other_vec in all_sending_vectors:
                if other_vec == my_vec:
                    continue
                if self.are_vectors_concurrent(my_vec[0], my_vec[1], other_vec[0], other_vec[1]):
                    delta_pred += self.calculate_concurrent_region_delay(my_vec[0], my_vec[1], other_vec[0], other_vec[1])
        return base_eod + delta_pred

    def select_next_hop_with_utility(self, current_uav, dest_uav, all_uavs, all_sending_vectors=None):
        """
        选择utility最大的邻居作为下一跳，utility=EoD(current, dest)-EoD(neighbor, dest)。
        候选集需满足mobility/interference约束（这里只排除强并发）。
        返回(best_neighbor, max_utility)
        """
        candidate_neighbors = []
        T = 0.4  # 预测时长（秒）
        for neighbor in all_uavs:
            if neighbor.id == current_uav.id:
                continue
            # 通信范围约束（当前）
            dist = math.sqrt((current_uav.x - neighbor.x) ** 2 + (current_uav.y - neighbor.y) ** 2)
            if dist > UAV_COMMUNICATION_RANGE:
                continue
            # mobility约束：预测T秒后距离
            future_x1 = getattr(current_uav, 'x', 0) + getattr(current_uav, 'vx', 0) * T
            future_y1 = getattr(current_uav, 'y', 0) + getattr(current_uav, 'vy', 0) * T
            future_x2 = getattr(neighbor, 'x', 0) + getattr(neighbor, 'vx', 0) * T
            future_y2 = getattr(neighbor, 'y', 0) + getattr(neighbor, 'vy', 0) * T
            future_dist = math.sqrt((future_x1 - future_x2) ** 2 + (future_y1 - future_y2) ** 2)
            if future_dist > UAV_COMMUNICATION_RANGE:
                continue
            # 干扰约束：排除与当前发送向量强并发的邻居
            if all_sending_vectors is not None:
                my_vec = ((current_uav.x, current_uav.y), (neighbor.x, neighbor.y))
                concurrent = False
                for other_vec in all_sending_vectors:
                    if other_vec == my_vec:
                        continue
                    if self.are_vectors_concurrent(my_vec[0], my_vec[1], other_vec[0], other_vec[1]):
                        concurrent = True
                        break
                if concurrent:
                    continue
            candidate_neighbors.append(neighbor)
        # 计算utility
        max_utility = -float('inf')
        best_neighbor = None
        eod_current = self.get_link_base_delay(current_uav, dest_uav)
        for neighbor in candidate_neighbors:
            eod_neighbor = self.get_link_base_delay(neighbor, dest_uav)
            utility = eod_current - eod_neighbor
            if utility > max_utility:
                max_utility = utility
                best_neighbor = neighbor
        return best_neighbor, max_utility 