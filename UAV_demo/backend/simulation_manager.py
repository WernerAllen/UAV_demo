# 文件: backend/simulation_manager.py
# 描述: 编排所有模块，管理实时仿真的状态和流程 (已更新为Dijkstra寻路算法)

from collections import deque
import random
import math
import os
import heapq # 导入heapq以实现优先队列
from core.uav import UAV
from core.packet import Packet
from mac_layer.mac import MACLayer
from simulation_config import *


class SimulationManager:
    def __init__(self):
        self.uavs = []
        self.simulation_time = 0.0
        self.is_running = False
        self.packets_in_network = []
        self.uav_graph = {}
        self.colors = ["blue", "green", "purple", "orange", "cyan", "magenta", "lime", "maroon", "navy", "olive",
                       "teal", "silver", "gold", "indigo", "violet", "pink", "khaki", "turquoise", "salmon", "tan"] * 5
        self.mac_layer = MACLayer(self.uavs)
        self.last_uav_count = DEFAULT_NUM_UAVS

    def start_simulation(self, num_uavs=None):
        random.seed(os.urandom(16))

        if num_uavs is not None:
            self.last_uav_count = num_uavs
        else:
            num_uavs = self.last_uav_count

        UAV.reset_id_counter()
        Packet.reset_id_counter()
        self.uavs.clear()
        self.packets_in_network.clear()
        self.simulation_time = 0.0
        
        self.mac_layer.reset_counters()

        for i in range(num_uavs):
            self.uavs.append(UAV(color=self.colors[i % len(self.colors)]))

        self.mac_layer.update_uav_list(self.uavs)
        self._build_uav_graph()
        self.is_running = True
        return f"{len(self.uavs)} UAVs created. Simulation started/reset."

    def step_simulation(self, time_increment=None):
        if not self.is_running: return "Error: Simulation not started."
        dt = time_increment if time_increment is not None else DEFAULT_TIME_INCREMENT
        self.simulation_time += dt
        for uav in self.uavs: uav.update_state(dt)
        self.mac_layer.process_transmissions(self.simulation_time)
        self._build_uav_graph()
        return f"Simulation stepped to {self.simulation_time:.2f}."

    def stop_simulation(self):
        self.is_running = False
        return "Manual simulation stopped."

    def initiate_data_transfer(self, source_id, destination_id, packet_count=1):
        path_ids, msg = self.get_shortest_path(source_id, destination_id)
        if not path_ids: return [], msg

        source_uav = self.mac_layer.uav_map.get(source_id)
        if not source_uav: return [], f"Source UAV {source_id} not found."

        created_packets = []
        for _ in range(packet_count):
            packet = Packet(source_id, destination_id, self.simulation_time)
            packet.path = path_ids
            packet.status = "in_transit"
            self.packets_in_network.append(packet)
            source_uav.add_packet_to_queue(packet)
            created_packets.append(packet)
        message = f"{packet_count} packet(s) created from UAV {source_id} to {destination_id}."
        return created_packets, message
        
    def generate_random_pairs_and_paths(self, pair_count):
        if len(self.uavs) < 2 or pair_count <= 0:
            return [], "UAV数量不足或源-目标对数量无效。"

        if pair_count * 2 > len(self.uavs):
            return [], "请求的源-目标对总数超过了可用的无人机数量。"

        all_uav_ids = [uav.id for uav in self.uavs]
        
        selected_ids = random.sample(all_uav_ids, pair_count * 2)
        
        pairs = []
        for i in range(pair_count):
            source_id = selected_ids[i*2]
            dest_id = selected_ids[i*2 + 1]
            pairs.append({"source": source_id, "destination": dest_id})
        
        pairs_with_paths = []
        for pair in pairs:
            path_ids, msg = self.get_shortest_path(pair["source"], pair["destination"])
            if path_ids:
                pairs_with_paths.append({
                    "source": pair["source"],
                    "destination": pair["destination"],
                    "path": path_ids
                })
        
        if not pairs_with_paths:
            return [], "成功生成源-目标对，但当前网络拓扑下未能找到任何有效路径。"

        return pairs_with_paths, f"成功生成 {len(pairs_with_paths)}/{pair_count} 个带初始路径的源-目标对。"

    # ## **** MODIFICATION START: 使用Dijkstra算法并修复类型报错 **** ##
    def get_shortest_path(self, source_uav_id, target_uav_id):
        """
        使用Dijkstra算法计算两个无人机之间的最短地理距离路径。
        """
        if not self.uavs: return None, "Simulation not active."
        
        uav_map = self.mac_layer.uav_map
        if source_uav_id not in uav_map or target_uav_id not in uav_map:
            return None, "Source/Target not found."
        
        if source_uav_id == target_uav_id: return [source_uav_id], "Source and target are the same."

        # 记录到各节点的最短距离
        distances = {uav_id: float('inf') for uav_id in uav_map}
        distances[source_uav_id] = 0
        
        # 引入一个计数器来确保元组的唯一性，避免比较后续元素
        entry_count = 0
        # 优先队列，存储 (距离, 计数器, 当前节点ID, 路径列表)
        # BUG FIX: 将初始距离从 0 改为 0.0 以确保类型一致性
        pq = [(0.0, entry_count, source_uav_id, [source_uav_id])]

        while pq:
            dist, _, current_id, path = heapq.heappop(pq)

            # 如果已经找到更短的路径，则跳过
            if dist > distances[current_id]:
                continue
            
            # 如果到达目标，则返回路径
            if current_id == target_uav_id:
                return path, f"Path found with total distance: {dist:.2f}m."

            current_uav = uav_map[current_id]
            for neighbor_id in self.uav_graph.get(current_id, []):
                neighbor_uav = uav_map[neighbor_id]
                
                # 计算边权重（两无人机间的距离）
                weight = math.sqrt(
                    (current_uav.x - neighbor_uav.x)**2 +
                    (current_uav.y - neighbor_uav.y)**2 +
                    (current_uav.z - neighbor_uav.z)**2
                )
                
                new_dist = dist + weight
                
                # 如果找到了更短的路径
                if new_dist < distances[neighbor_id]:
                    distances[neighbor_id] = new_dist
                    entry_count += 1
                    heapq.heappush(pq, (new_dist, entry_count, neighbor_id, path + [neighbor_id]))
                    
        return None, "No path found."
    # ## **** MODIFICATION END **** ##

    def _build_uav_graph(self):
        self.uav_graph = {uav.id: [] for uav in self.uavs}
        for i in range(len(self.uavs)):
            for j in range(i + 1, len(self.uavs)):
                uav1, uav2 = self.uavs[i], self.uavs[j]
                dist_sq = (uav1.x - uav2.x) ** 2 + (uav1.y - uav2.y) ** 2 + (uav1.z - uav2.z) ** 2
                if dist_sq <= UAV_COMMUNICATION_RANGE ** 2:
                    self.uav_graph[uav1.id].append(uav2.id)
                    self.uav_graph[uav2.id].append(uav1.id)

    def get_simulation_state(self):
        status_text = "idle"
        if self.is_running:
            status_text = "running"
        elif self.uavs:
            status_text = "paused/stopped"

        return {
            "status": status_text,
            "time": self.simulation_time,
            "uavs": [uav.get_data_for_api() for uav in self.uavs],
            "packets": [p.__dict__ for p in self.packets_in_network],
            "mac_packet_status": self.mac_layer.get_packet_status_snapshot(),
            'grid_config': {
                'rows': GRID_ROWS, 'cols': GRID_COLS, 'prr_map': PRR_GRID_MAP,
                'width': MAX_X, 'height': MAX_Y
            }
        }
