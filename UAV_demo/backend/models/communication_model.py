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
            
        # 根据当前使用的路由模型选择合适的网格配置
        if getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False):
            # 使用PTP专用网格配置
            rows = getattr(simulation_config, 'PTP_GRID_ROWS', GRID_ROWS)
            cols = getattr(simulation_config, 'PTP_GRID_COLS', GRID_COLS)
            
            # 如果启用了随机PRR，则动态计算PRR值
            if getattr(simulation_config, 'PTP_USE_RANDOM_PRR', False):
                prr_min = getattr(simulation_config, 'PTP_PRR_MIN', 0.5)
                prr_max = getattr(simulation_config, 'PTP_PRR_MAX', 0.9)
                
                # 计算网格位置
                cell_width = MAX_X / cols
                cell_height = MAX_Y / rows
                col_index = min(int(receiver_uav.x / cell_width), cols - 1)
                row_index = min(int(receiver_uav.y / cell_height), rows - 1)
                
                # 使用随机数生成器，但为了确保同一位置PRR值一致，使用位置作为种子
                seed = hash(f"{row_index}_{col_index}") % 10000
                r = random.Random(seed)
                regional_prr = prr_min + r.random() * (prr_max - prr_min)
            else:
                # 使用全局PRR网格
                cell_width = MAX_X / cols
                cell_height = MAX_Y / rows
                col_index = min(int(receiver_uav.x / cell_width), cols - 1)
                row_index = min(int(receiver_uav.y / cell_height), rows - 1)
                
                # 如果索引超出范围，使用默认值
                if row_index >= len(PRR_GRID_MAP) or col_index >= len(PRR_GRID_MAP[0]):
                    regional_prr = 0.7  # 默认值
                else:
                    regional_prr = PRR_GRID_MAP[row_index][col_index]
        else:
            # 使用默认配置
            cell_width = MAX_X / GRID_COLS
            cell_height = MAX_Y / GRID_ROWS
            col_index = min(int(receiver_uav.x / cell_width), GRID_COLS - 1)
            row_index = min(int(receiver_uav.y / cell_height), GRID_ROWS - 1)
            regional_prr = PRR_GRID_MAP[row_index][col_index]
            
        return random.random() > regional_prr

    

