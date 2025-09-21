# 文件: backend/protocols/ptp_protocol.py
# 描述: PTP 路由协议实现 (原 RoutingModel)

import math
import random
try:
    import numpy as np
except ImportError:
    np = None
from simulation_config import *

class PTPRoutingModel:
    """
    实现论文中提出的PTP相关估算模型
    """
    def __init__(self, uav_map):
        self.uav_map = uav_map
        # 初始化PRR网格（用于快速查询通信质量）
        from simulation_config import PTP_GRID_ROWS, PTP_GRID_COLS, PTP_USE_RANDOM_PRR
        # 直接导入全局PRR参数
        from simulation_config import PRR_MIN, PRR_MAX
        
        # 初始化PTP的PRR网格
        self.ptp_prr_grid = []
        if hasattr(globals(), 'PTP_USE_RANDOM_PRR') and PTP_USE_RANDOM_PRR:
            self._initialize_random_prr_grid()
            
        # 初始化PRR缓存
        self._prr_cache = {}

    def _initialize_random_prr_grid(self):
        """初始化随机PRR网格"""
        from simulation_config import PTP_GRID_ROWS, PTP_GRID_COLS, PRR_MIN, PRR_MAX
        
        rows = PTP_GRID_ROWS
        cols = PTP_GRID_COLS
        
        self.ptp_prr_grid = []
        for _ in range(rows):
            row = []
            for _ in range(cols):
                row.append(random.uniform(PRR_MIN, PRR_MAX))
            self.ptp_prr_grid.append(row)
        
        print(f"已初始化PTP随机PRR网格 ({rows}x{cols})，PRR范围: {PRR_MIN}-{PRR_MAX}")

    def get_grid_cell(self, x, y):
        if not (0 <= x < MAX_X and 0 <= y < MAX_Y):
            return None, None
            
        # 使用PTP专用网格尺寸
        from simulation_config import PTP_GRID_ROWS, PTP_GRID_COLS
        
        cell_width = MAX_X / PTP_GRID_COLS
        cell_height = MAX_Y / PTP_GRID_ROWS
        col = min(int(x / cell_width), PTP_GRID_COLS - 1)
        row = min(int(y / cell_height), PTP_GRID_ROWS - 1)
        return row, col

    def calculate_eod_for_grid(self, distance_in_grid, grid_row, grid_col):
        # 获取PRR值，优先使用PTP专用随机PRR网格
        if self.ptp_prr_grid is not None:
            rows = len(self.ptp_prr_grid)
            cols = len(self.ptp_prr_grid[0]) if rows > 0 else 0
            
            if 0 <= grid_row < rows and 0 <= grid_col < cols:
                prr = self.ptp_prr_grid[grid_row][grid_col]
            else:
                prr = 0.7  # 默认值
        else:
            # 使用全局PRR网格
            if grid_row >= len(PRR_GRID_MAP) or grid_col >= len(PRR_GRID_MAP[0]):
                prr = 0.7  # 默认值
            else:
                prr = PRR_GRID_MAP[grid_row][grid_col]
        
        if AVG_ONE_HOP_DISTANCE == 0 or prr == 0:
            return float('inf')
            
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
        # 使用PTP专用网格尺寸（如果定义了）
        rows = getattr(globals(), 'PTP_GRID_ROWS', GRID_ROWS)
        cols = getattr(globals(), 'PTP_GRID_COLS', GRID_COLS)
        
        cell_width = MAX_X / cols
        cell_height = MAX_Y / rows
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
        
        # 导入PRR配置信息
        from simulation_config import PRR_MIN, PRR_MAX
        
        # 计算PRR权重系数，在低PRR环境下增大权重
        avg_prr = (PRR_MIN + PRR_MAX) / 2
        # 根据环境PRR动态调整权重，PRR越低权重越大
        if avg_prr < 0.3:
            prr_weight = 0.5  # 低PRR环境下较高权重
        else:
            prr_weight = 0.3  # 正常PRR环境下较低权重
            
        for neighbor in candidate_neighbors:
            eod_neighbor = self.get_link_base_delay(neighbor, dest_uav)
            
            # 计算基本效用
            base_utility = eod_current - eod_neighbor
            
            # 获取当前节点到邻居的PRR值
            if hasattr(self, '_get_prr'):
                prr = self._get_prr(current_uav, neighbor)
            else:
                # 如果没有_get_prr方法，使用基本PRR计算逻辑
                row, col = self.get_grid_cell(neighbor.x, neighbor.y)
                prr = self.calculate_eod_for_grid(1.0, row, col)
                prr = min(max(prr, 0.05), 0.95)  # 确保PRR在合理范围内
            
            # 修改后的效用计算 - 增加PRR权重
            utility = base_utility + prr_weight * prr
            
            if utility > max_utility:
                max_utility = utility
                best_neighbor = neighbor
                
        return best_neighbor, max_utility 

    def _get_prr(self, uav1, uav2):
        """
        获取uav1到uav2的PRR值
        使用网格化的方式和配置的PRR范围
        添加缓存以提高性能
        """
        # 计算距离
        dist = math.sqrt((uav1.x - uav2.x) ** 2 + (uav1.y - uav2.y) ** 2 + (uav1.z - uav2.z) ** 2)
        
        # 使用距离区间作为键
        if not hasattr(self, '_prr_cache'):
            self._prr_cache = {}
            
        # 为了避免随机值在每次调用时都不同，我们对距离进行离散化处理
        dist_key = int(dist * 10)  # 0.1的精度
        
        if dist_key in self._prr_cache:
            return self._prr_cache[dist_key]
        
        # 导入PRR配置
        from simulation_config import PRR_MIN, PRR_MAX
        
        # 根据距离计算PRR
        range_size = PRR_MAX - PRR_MIN  # 上下限差值
        
        if dist <= 10:
            # 距离最近，使用最高PRR区间 (75%-100%范围)
            prr_min = PRR_MIN + range_size * 0.75
            prr_max = PRR_MAX
            prr = random.uniform(prr_min, prr_max)
        elif dist <= 30:
            # 距离较近，使用较高PRR区间 (50%-75%范围)
            prr_min = PRR_MIN + range_size * 0.5
            prr_max = PRR_MIN + range_size * 0.75
            prr = random.uniform(prr_min, prr_max)
        elif dist <= 60:
            # 距离中等，使用中等PRR区间 (25%-50%范围)
            prr_min = PRR_MIN + range_size * 0.25
            prr_max = PRR_MIN + range_size * 0.5
            prr = random.uniform(prr_min, prr_max)
        elif dist <= 100:
            # 距离较远，使用较低PRR区间 (0%-25%范围)
            prr_min = PRR_MIN
            prr_max = PRR_MIN + range_size * 0.25
            prr = random.uniform(prr_min, prr_max)
        else:
            # 超出范围返回0
            prr = 0
        
        # 限制缓存大小
        if len(self._prr_cache) > 1000:
            self._prr_cache.clear()
        
        # 存储结果
        self._prr_cache[dist_key] = prr
        
        return prr 