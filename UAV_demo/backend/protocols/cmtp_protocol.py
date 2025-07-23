# 文件: backend/protocols/cmtp_protocol.py
# 描述: CMTP 路由协议实现（拥塞感知多层树协议）

import math
import numpy as np
from simulation_config import *

class CMTPRoutingModel:
    """
    拥塞感知多层树协议（Congestion-aware Multi-Layer Tree Protocol, CMTP）骨架
    论文MTP增强：支持能效树更新、路径合并、动态拥塞延迟
    """
    # 论文MTP增强参数
    ETX_UPDATE_THRESHOLD = 0.3  # ETX变化阈值，超过才更新树
    MERGE_DISTANCE_THRESHOLD = 30  # 目标节点合并树的距离阈值

    def __init__(self, uav_map):
        self.uav_map = uav_map
        self.virtual_trees = None
        self.root_nodes = None
        self.root_groups = None  # 论文MTP增强：合并树的分组
        self.last_etx_to_root = {}  # 论文MTP增强：记录上次ETX

    def build_virtual_tree_structures(self, destination_ids=None):
        """
        构建多层虚拟树结构，选择RootNodes。
        论文MTP增强：支持目标节点合并树
        """
        if destination_ids is None:
            destination_ids = list(self.uav_map.keys())
        self.root_nodes = []
        self.virtual_trees = {}
        # 论文MTP增强：路径合并机制
        self.root_groups = self._group_roots_by_distance(destination_ids)
        for group in self.root_groups:
            # 以第一个目标为主树根
            root_id = group[0]
            self.root_nodes.append(root_id)
            tree = self._build_tree_for_root(root_id)
            # 合并组内其他目标节点
            for other_id in group[1:]:
                tree = self._merge_tree(tree, self._build_tree_for_root(other_id))
            self.virtual_trees[root_id] = tree

    def _group_roots_by_distance(self, destination_ids):
        """论文MTP增强：将距离较近的目标节点分为一组，返回分组列表。"""
        groups = []
        used = set()
        for i, id1 in enumerate(destination_ids):
            if id1 in used:
                continue
            group = [id1]
            uav1 = self.uav_map[id1]
            for j, id2 in enumerate(destination_ids):
                if i == j or id2 in used:
                    continue
                uav2 = self.uav_map[id2]
                dist = math.sqrt((uav1.x - uav2.x) ** 2 + (uav1.y - uav2.y) ** 2 + (uav1.z - uav2.z) ** 2)
                if dist < self.MERGE_DISTANCE_THRESHOLD:
                    group.append(id2)
                    used.add(id2)
            used.add(id1)
            groups.append(group)
        return groups

    def _merge_tree(self, tree1, tree2):
        """论文MTP增强：合并两棵树，优先保留ETX更小的父节点。"""
        merged = dict(tree1)
        for node_id, parent_id in tree2.items():
            if node_id not in merged:
                merged[node_id] = parent_id
            else:
                # 选择ETX更小的父节点
                uav = self.uav_map.get(node_id)
                p1 = self.uav_map.get(merged[node_id]) if merged[node_id] else None
                p2 = self.uav_map.get(parent_id) if parent_id else None
                etx1 = self.get_link_base_delay(uav, p1) if p1 else float('inf')
                etx2 = self.get_link_base_delay(uav, p2) if p2 else float('inf')
                if etx2 < etx1:
                    merged[node_id] = parent_id
        return merged

    def _build_tree_for_root(self, root_id):
        """以root_id为根，递归建立虚拟树，返回{node_id: parent_id}映射。"""
        tree = {root_id: None}  # 根节点无父节点
        visited = set([root_id])
        queue = [root_id]
        while queue:
            current_id = queue.pop(0)
            current_uav = self.uav_map[current_id]
            for neighbor in self._get_neighbors(current_uav):
                if neighbor.id not in visited:
                    # 选择ETX最小的父节点
                    min_etx = float('inf')
                    best_parent = None
                    for parent in self._get_neighbors(neighbor):
                        etx = self.get_link_base_delay(neighbor, parent)
                        if etx < min_etx:
                            min_etx = etx
                            best_parent = parent
                    if best_parent:
                        tree[neighbor.id] = best_parent.id
                        visited.add(neighbor.id)
                        queue.append(neighbor.id)
        return tree

    def update_congestion_info(self):
        """
        更新拥塞感知信息，收集所有虚拟树的链路，找出重叠（并发）链路集合。
        结果保存在self.congestion_links: {link_tuple: [root_id, ...]}
        """
        self.congestion_links = {}
        # 遍历所有虚拟树，统计每条链路出现在哪些树中
        for root_id, tree in (self.virtual_trees or {}).items():
            for node_id, parent_id in tree.items():
                if parent_id is None:
                    continue
                link = tuple(sorted([node_id, parent_id]))  # 无向链路
                if link not in self.congestion_links:
                    self.congestion_links[link] = []
                self.congestion_links[link].append(root_id)

    def calculate_expected_transmission_time(self, from_uav, to_uav, layer=0, packet=None, sim_time=None):
        """
        论文MTP增强：Δ_pred动态计算（基于PRR和链路利用率）
        """
        etx = self.get_link_base_delay(from_uav, to_uav)
        congestion_delay = 0.0
        if hasattr(self, 'congestion_links'):
            from_id, to_id = from_uav.id, to_uav.id
            link = tuple(sorted([from_id, to_id]))
            for other_link, roots in self.congestion_links.items():
                if other_link == link:
                    continue
                if link in self.congestion_links and set(self.congestion_links[link]) & set(roots):
                    # 论文MTP增强：Δ_pred动态计算
                    prr = self._get_prr(self.uav_map[other_link[0]], self.uav_map[other_link[1]])
                    # 假设链路利用率为0.5（可根据实际流量统计）
                    utilization = 0.5
                    delta_pred = (1.0 / prr) * utilization if prr > 0 else 0.2
                    congestion_delay += delta_pred
        ett = etx + congestion_delay
        # 事件历史记录
        if packet is not None and hasattr(packet, 'add_event'):
            info = f"from={from_uav.id}, to={to_uav.id}, etx={etx:.3f}, congestion_delay={congestion_delay:.3f}, ett={ett:.3f}"
            packet.add_event("mtp_ett_calc", getattr(from_uav, 'id', None), getattr(packet, 'current_hop_index', None), sim_time if sim_time is not None else 0, info)
        return ett

    def select_next_hop(self, current_uav, candidate_neighbors, layer=0, packet=None, sim_time=None):
        """
        在候选邻居中选择期望传输时间（ETT）最短的节点作为下一跳。
        返回: (best_neighbor, min_ett)
        """
        min_ett = float('inf')
        best_neighbor = None
        ett_map = {}
        for neighbor in candidate_neighbors:
            ett = self.calculate_expected_transmission_time(current_uav, neighbor, layer, packet=packet, sim_time=sim_time)
            ett_map[neighbor.id] = ett
            if ett < min_ett:
                min_ett = ett
                best_neighbor = neighbor
        # 事件历史记录
        if packet is not None and hasattr(packet, 'add_event'):
            candidates_str = ', '.join([f"{nid}:{ett_map[nid]:.3f}" for nid in ett_map])
            info = f"candidates=[{candidates_str}], selected={getattr(best_neighbor, 'id', None)}, ett={min_ett:.3f}"
            packet.add_event("cmtp_path_select", getattr(current_uav, 'id', None), getattr(packet, 'current_hop_index', None), sim_time if sim_time is not None else 0, info)
        
        # 注释掉这里的输出，将在数据包传输成功后显示
        # if best_neighbor:
        #     # 计算进度值：ETT越小，进度越高，最大为1.0
        #     if min_ett > 0:
        #         progress = min(1.0, 1.0 / min_ett)  # ETT的倒数，限制最大值为1.0
        #     else:
        #         progress = 1.0
        #     print(f"【CMTP】 UAV-{current_uav.id}→{best_neighbor.id} 选择→UAV-{best_neighbor.id} 进度:{progress:.2f}")
            
        return best_neighbor, min_ett

    def get_link_base_delay(self, uav1, uav2=None, root_id=None):
        """
        计算单跳ETX或到RootNode的ETX。
        - 若uav2不为None，则返回uav1到uav2的单跳ETX。
        - 若uav2为None且root_id不为None，则递归计算uav1到root的最小ETX。
        """
        if uav2 is not None:
            # 单跳ETX: 1 / PRR(x, y)
            prr = self._get_prr(uav1, uav2)
            if prr == 0:
                return float('inf')
            return 1.0 / prr
        elif root_id is not None:
            # 到RootNode的ETX: 递归最小{邻居ETX+单跳ETX}
            if uav1.id == root_id:
                return 0.0
            min_etx = float('inf')
            for neighbor in self._get_neighbors(uav1):
                etx_link = self.get_link_base_delay(uav1, neighbor)
                etx_neighbor = self.get_link_base_delay(neighbor, None, root_id)
                total_etx = etx_link + etx_neighbor
                if total_etx < min_etx:
                    min_etx = total_etx
            return min_etx
        else:
            raise ValueError("get_link_base_delay: uav2和root_id不能同时为None")

    def _get_prr(self, uav1, uav2):
        """获取uav1到uav2的PRR，基于距离分段随机。"""
        import random
        dist = math.sqrt((uav1.x - uav2.x) ** 2 + (uav1.y - uav2.y) ** 2 + (uav1.z - uav2.z) ** 2)
        if dist <= 10:
            return random.uniform(0.85, 0.9)
        elif dist <= 30:
            return random.uniform(0.75, 0.85)
        elif dist <= 60:
            return random.uniform(0.65, 0.75)
        elif dist <= 100:
            return random.uniform(0.5, 0.65)
        else:
            return 0

    def _get_neighbors(self, uav):
        """获取uav的邻居节点（通信范围内）。"""
        neighbors = []
        for other in self.uav_map.values():
            if other.id == uav.id:
                continue
            dist = math.sqrt((uav.x - other.x) ** 2 + (uav.y - other.y) ** 2 + (uav.z - other.z) ** 2)
            if dist <= UAV_COMMUNICATION_RANGE:
                neighbors.append(other)
        return neighbors

    def are_vectors_concurrent(self, p1, q1, p2, q2):
        """CMTP下判断向量并发（可根据需要实现）"""
        return False  # 默认无并发

    def calculate_concurrent_region_delay(self, vec1_p1, vec1_q1, vec2_p2, vec2_q2):
        """
        计算并发区域的延迟。这里简化为：若两条链路在congestion_links中重叠，则返回常数拥塞延迟。
        """
        # 将向量端点映射为链路
        link1 = tuple(sorted([vec1_p1, vec1_q1]))
        link2 = tuple(sorted([vec2_p2, vec2_q2]))
        # 检查是否为同一链路或有重叠
        if hasattr(self, 'congestion_links'):
            if link1 in self.congestion_links and link2 in self.congestion_links:
                # 若两链路都在拥塞集合，且有共同root，视为并发
                roots1 = set(self.congestion_links[link1])
                roots2 = set(self.congestion_links[link2])
                if roots1 & roots2:
                    return 0.2  # 拥塞延迟常数，可根据实际情况调整
        return 0.0

    def get_grid_cell(self, x, y):
        """根据坐标获取所在网格的索引。"""
        if not (0 <= x < MAX_X and 0 <= y < MAX_Y):
            return None, None
        cell_width = MAX_X / GRID_COLS
        cell_height = MAX_Y / GRID_ROWS
        col = min(int(x / cell_width), GRID_COLS - 1)
        row = min(int(y / cell_height), GRID_ROWS - 1)
        return row, col

    def self_heal_virtual_trees(self):
        """
        论文MTP增强：能效树更新机制，只有ETX显著变化才更新树
        """
        if not self.virtual_trees or not self.root_nodes:
            return
        for root_id in self.root_nodes:
            tree = self.virtual_trees[root_id]
            for node_id in list(tree.keys()):
                parent_id = tree[node_id]
                if parent_id is None:
                    continue
                node = self.uav_map.get(node_id)
                parent = self.uav_map.get(parent_id)
                if node is None or parent is None:
                    tree[node_id] = None
                    continue
                dist = math.sqrt((node.x - parent.x) ** 2 + (node.y - parent.y) ** 2 + (node.z - parent.z) ** 2)
                if dist > UAV_COMMUNICATION_RANGE:
                    new_parent, min_etx = self._find_new_parent(node, root_id)
                    # 论文MTP增强：只有ETX变化大于阈值才更新
                    last_etx = self.last_etx_to_root.get((node_id, root_id), float('inf'))
                    if abs(min_etx - last_etx) > self.ETX_UPDATE_THRESHOLD:
                        tree[node_id] = new_parent.id if new_parent else None
                        self.last_etx_to_root[(node_id, root_id)] = min_etx
            self._update_etx_recursive(tree, root_id, 0.0)

    def _find_new_parent(self, node, root_id):
        """在邻居中重选一个到root_id ETX最小且可达的父节点。"""
        min_etx = float('inf')
        best_parent = None
        for neighbor in self._get_neighbors(node):
            etx = self.get_link_base_delay(node, neighbor) + self.get_link_base_delay(neighbor, None, root_id)
            if etx < min_etx:
                min_etx = etx
                best_parent = neighbor
        return best_parent, min_etx

    def _update_etx_recursive(self, tree, node_id, etx_to_root):
        """递归更新所有子节点的ETX到根节点的值。"""
        node = self.uav_map.get(node_id)
        if node is not None:
            node.etx_to_root = etx_to_root
        # 遍历所有以node_id为父节点的子节点
        for child_id, parent_id in tree.items():
            if parent_id == node_id:
                child = self.uav_map.get(child_id)
                if child is not None:
                    etx = self.get_link_base_delay(child, node)
                    self._update_etx_recursive(tree, child_id, etx_to_root + etx)

    # 可根据论文公式和仿真需求继续扩展更多方法 