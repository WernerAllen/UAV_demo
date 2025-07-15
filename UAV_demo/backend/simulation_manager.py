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
from models.communication_model import RoutingModel

class SimulationManager:
    def __init__(self):
        self.uavs = []
        self.simulation_time = 0.0
        self.is_running = False
        self.packets_in_network = []
        self.uav_graph = {}
        self.colors = ["blue", "green", "purple", "orange", "cyan", "magenta", "lime", "maroon", "navy", "olive",
                       "teal", "silver", "gold", "indigo", "violet", "pink", "khaki", "turquoise", "salmon", "tan"] * 5
        
        # ## **** MODIFICATION START: 将sim_manager实例传给MAC层 **** ##
        self.mac_layer = MACLayer(self.uavs, self)
        # ## **** MODIFICATION END **** ##

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
            
            # ## **** MODIFICATION START: 记录下一跳节点位置 **** ##
            # 记录路径中每个下一跳节点的初始位置
            for i in range(len(path_ids) - 1):
                next_hop_id = path_ids[i + 1]
                next_hop_uav = self.mac_layer.uav_map.get(next_hop_id)
                if next_hop_uav:
                    packet.record_next_hop_position(next_hop_id, next_hop_uav.x, next_hop_uav.y, next_hop_uav.z)
            # ## **** MODIFICATION END **** ##
            
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

    # ## **** MODIFICATION START: 使用PTP模型重构寻路算法 **** ##
    def get_shortest_path(self, source_uav_id, target_uav_id):
        """
        使用Dijkstra算法计算路径。
        根据配置，权重可以是地理距离，也可以是DHyTP估算延迟。
        """
        if not self.uavs: return None, "Simulation not active."
        
        uav_map = self.mac_layer.uav_map
        if source_uav_id not in uav_map or target_uav_id not in uav_map:
            return None, "Source/Target not found."
        
        if source_uav_id == target_uav_id: return [source_uav_id], "Source and target are the same."

        # 初始化路由模型（如果需要）
        routing_model = RoutingModel(uav_map) if USE_PTP_ROUTING_MODEL else None

        # Dijkstra算法初始化
        distances = {uav_id: float('inf') for uav_id in uav_map}
        distances[source_uav_id] = 0
        entry_count = 0
        pq = [(0.0, entry_count, source_uav_id, [source_uav_id])]

        while pq:
            dist, _, current_id, path = heapq.heappop(pq)

            if dist > distances[current_id]:
                continue
            
            if current_id == target_uav_id:
                unit = "s" if USE_PTP_ROUTING_MODEL else "m"
                return path, f"Path found with total weight: {dist:.2f}{unit}."

            current_uav = uav_map[current_id]
            for neighbor_id in self.uav_graph.get(current_id, []):
                neighbor_uav = uav_map[neighbor_id]
                
                # --- 核心修改：计算边的权重 ---
                edge_weight = 0.0
                if USE_PTP_ROUTING_MODEL and routing_model:
                    # 使用PTP模型计算估算延迟作为权重
                    
                    # 1. 计算基础链路延迟 (EoD之和)
                    edge_weight = routing_model.get_link_base_delay(current_uav, neighbor_uav)

                    # 2. 计算并发区域带来的额外延迟
                    # 检查此链路是否与网络中其他数据包的当前跳有并发
                    concurrent_delay = 0.0
                    link_p1 = (current_uav.x, current_uav.y)
                    link_q1 = (neighbor_uav.x, neighbor_uav.y)

                    for pkt in self.packets_in_network:
                        # 排除发往此邻居的包或已送达的包
                        if pkt.status == 'delivered' or pkt.get_next_hop_id() == neighbor_id:
                            continue
                        
                        holder = self.mac_layer.uav_map.get(pkt.current_holder_id)
                        next_hop = self.mac_layer.uav_map.get(pkt.get_next_hop_id())

                        if holder and next_hop:
                            pkt_p2 = (holder.x, holder.y)
                            pkt_q2 = (next_hop.x, next_hop.y)
                            if routing_model.are_vectors_concurrent(link_p1, link_q1, pkt_p2, pkt_q2):
                                # 如果并发，累加惩罚性延迟
                                concurrent_delay += routing_model.calculate_concurrent_region_delay(
                                    link_p1, link_q1, pkt_p2, pkt_q2
                                )
                    edge_weight += concurrent_delay

                else:
                    # 使用旧的地理距离作为权重
                    edge_weight = math.sqrt(
                        (current_uav.x - neighbor_uav.x)**2 +
                        (current_uav.y - neighbor_uav.y)**2 +
                        (current_uav.z - neighbor_uav.z)**2
                    )
                # --- 权重计算结束 ---

                new_dist = dist + edge_weight
                
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

    # ## **** MODIFICATION START: 添加打印数据包事件历史的方法 **** ##
    def print_packet_event_history(self):
        """打印所有数据包的事件历史"""
        print("\n" + "="*80)
        print("仿真完成！所有数据包的事件历史：")
        print("="*80)
        
        for pkt in self.packets_in_network:
            print(f"\n数据包 {pkt.id} ({pkt.source_id} -> {pkt.destination_id}):")
            print(f"  状态: {pkt.status}")
            print(f"  实际路径: {pkt.actual_hops}")
            if hasattr(pkt, 'delivery_time') and pkt.delivery_time is not None:
                print(f"  送达时间: {pkt.delivery_time:.2f}秒")
            
            if pkt.event_history:
                print(f"  事件历史 ({len(pkt.event_history)} 个事件):")
                for i, event in enumerate(pkt.event_history, 1):
                    print(f"    {i}. [{event['sim_time']:.2f}s] {event['event']} - {event['info']}")
            else:
                print("  事件历史: 无")
        
        # 统计位置变动事件
        position_change_count = 0
        for pkt in self.packets_in_network:
            for event in pkt.event_history:
                if event['event'] == 'position_change':
                    position_change_count += 1
        
        print(f"\n" + "="*80)
        print(f"位置变动事件统计: 共 {position_change_count} 次")
        print("="*80)
    # ## **** MODIFICATION END **** ##
