# 文件: backend/protocols/cmtp_protocol.py
# 描述: CMTP 路由协议实现（拥塞感知多层树协议）

import math
import numpy as np
import time  # 添加time模块导入，用于记录时间
import random  # 添加random模块导入，用于PRR计算
from functools import lru_cache  # 添加lru_cache用于缓存计算结果
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
        
        # 添加协议状态控制变量，类似DHyTP
        self.tree_construction_started = False  # 是否已开始构建树
        self.destination_list = []  # 目标节点列表
        self.tree_build_progress = 0.0  # 树构建进度(0-1)
        self.tree_build_start_time = None  # 记录开始构建树的时间
        
        # 随机时间区间配置
        self.min_tree_build_time_range = (0.3, 0.5)  # 树构建时间范围(最小值, 最大值)
        self.min_tree_build_time = self._generate_random_build_time()  # 当前实验的随机树构建时间
        
        self.virtual_nodes_history = []  # 记录虚拟树节点数量历史
        self.last_update_time = None  # 上次更新时间
        self.tree_ready = False  # 树是否已经构建完成

        # ## **** ENERGY MODIFICATION START: 添加能耗累积计数器 **** ##
        self.accumulated_tree_creation_energy = 0.0  # 累积的树创建能耗
        self.accumulated_tree_maintenance_energy = 0.0  # 累积的树维护能耗
        self.tree_created = False  # 标记树是否已创建，避免重复计算树创建能耗
        # ## **** ENERGY MODIFICATION END **** ##

    def _generate_random_build_time(self):
        """生成随机树构建时间"""
        import random
        min_time, max_time = self.min_tree_build_time_range
        random_time = random.uniform(min_time, max_time)
        # 禁用输出
        # print(f"◆ CMTP本次树构建时间设定为: {random_time:.2f}秒")
        return random_time

    def update_protocol_status(self, destination_ids=None, sim_time=None):
        """
        更新协议状态：
        1. 记录目标节点列表
        2. 动态评估树构建进度
        3. 构建多层虚拟树结构
        4. 更新拥塞感知信息
        5. 判断是否树已构建完成
        """
        # 使用仿真时间而不是实际时间
        current_time = sim_time if sim_time is not None else 0.0

        # 首次指定目标节点，开始构建树
        if destination_ids and not self.tree_construction_started:
            self.destination_list = destination_ids if isinstance(destination_ids, list) else [destination_ids]
            self.tree_construction_started = True
            self.tree_build_progress = 0.0
            self.tree_build_start_time = current_time
            self.last_update_time = current_time
            # 重新生成随机树构建时间
            self.min_tree_build_time = self._generate_random_build_time()
            
            # 开始构建虚拟树结构
            self.build_virtual_tree_structures(self.destination_list)
            # 禁用输出
            # print(f"◆ CMTP开始构建树：目标节点 {self.destination_list}, 预计时间 {self.min_tree_build_time:.2f}秒")
            return

        # 如果树已经构建完成，继续维护树结构和拥塞信息
        if self.tree_ready:
            # 定期更新拥塞信息和树自愈
            self.update_congestion_info()
            try:
                # 包装在try-except中防止递归错误影响系统稳定性
                self.self_heal_virtual_trees()
            except RecursionError as e:
                print(f"◆ 警告：树自愈过程中遇到递归错误：{str(e)}. 跳过本次自愈操作.")
            except Exception as e:
                print(f"◆ 警告：树自愈过程中遇到错误：{str(e)}. 跳过本次自愈操作.")
            return

        # 尝试构建CMTP树
        if self.tree_construction_started and self.destination_list:
            # 计算时间经过（使用仿真时间）
            if self.tree_build_start_time is None:
                self.tree_build_start_time = current_time
                
            elapsed_time = current_time - self.tree_build_start_time

            # 限制更新频率，每0.1秒最多更新一次（使用仿真时间）
            if self.last_update_time and current_time - self.last_update_time < 0.1:
                return

            self.last_update_time = current_time

            # 构建虚拟树结构
            self.build_virtual_tree_structures(self.destination_list)

            # 更新拥塞信息
            self.update_congestion_info()

            # 评估树构建进度
            if self.virtual_trees:
                # 记录虚拟树节点数量历史
                covered_nodes = set()
                for _, tree in self.virtual_trees.items():
                    covered_nodes.update(tree.keys())

                self.virtual_nodes_history.append(len(covered_nodes))
                if len(self.virtual_nodes_history) > 10:
                    self.virtual_nodes_history = self.virtual_nodes_history[-10:]

                # 记录上一次进度
                old_progress = self.tree_build_progress

                # 与DHyTP一致：完全基于时间因子计算进度
                time_factor = min(1.0, elapsed_time / self.min_tree_build_time)
                
                # 进度直接等于时间因子
                self.tree_build_progress = time_factor
                
                # 切换条件：只基于时间因子，与DHyTP保持一致
                can_switch = elapsed_time >= self.min_tree_build_time

                # 树已构建完成但尚未标记为ready时，立即标记为ready
                if can_switch and not self.tree_ready:
                    print(f"\n◆◆◆ CMTP树构建完成：时间={elapsed_time:.1f}s/{self.min_tree_build_time:.2f}s, 进度={self.tree_build_progress:.2f} ◆◆◆\n")
                    self.tree_ready = True
                    # 最终更新拥塞信息
                    self.update_congestion_info()
                elif not can_switch:
                    # 进度有显著变化时才输出日志
                    progress_change = self.tree_build_progress - old_progress
                    if progress_change >= 0.1:
                        # 禁用输出
                        # print(f"◆ CMTP构建进度: {self.tree_build_progress:.2f} (时间:{elapsed_time:.2f}/{self.min_tree_build_time:.2f}秒)")
                        pass
                        
                # 禁用调试输出
                # print(f"◆ CMTP构建状态: 进度={self.tree_build_progress:.2f}, 时间={elapsed_time:.2f}/{self.min_tree_build_time:.2f}秒, 可切换={can_switch}, 树已就绪={self.tree_ready}")
                
    def reset_protocol_state(self):
        """重置CMTP协议状态，用于新的实验轮次"""
        self.tree_construction_started = False
        self.destination_list = []
        self.tree_build_progress = 0.0
        self.tree_build_start_time = None
        self.virtual_nodes_history = []
        self.last_update_time = None
        self.virtual_trees = None
        self.root_nodes = None
        self.root_groups = None
        self.last_etx_to_root = {}
        self.tree_ready = False
        
        # 清除所有计算缓存
        if hasattr(self, '_neighbors_cache'):
            self._neighbors_cache.clear()
        if hasattr(self, '_prr_cache'):
            self._prr_cache.clear()
        if hasattr(self, '_etx_to_root_cache'):
            self._etx_to_root_cache.clear()
            
        # 每次重置时不重新生成随机树构建时间，等到开始构建树时再生成
        print("◆ CMTP协议状态已重置，准备新的实验轮次")
        
        # ## **** ENERGY MODIFICATION START: 重置能耗累积计数器 **** ##
        self.accumulated_tree_creation_energy = 0.0
        self.accumulated_tree_maintenance_energy = 0.0
        self.tree_created = False
        # ## **** ENERGY MODIFICATION END **** ##

    def build_virtual_tree_structures(self, destination_ids=None):
        """
        构建多层虚拟树结构，选择RootNodes。
        论文MTP增强：支持目标节点合并树
        """
        if destination_ids is None:
            destination_ids = list(self.uav_map.keys())
        self.root_nodes = []
        self.virtual_trees = {}
        
        # ## **** ENERGY MODIFICATION START: 记录树创建能耗 **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS and not self.tree_created:
            tree_creation_energy = PROTOCOL_ENERGY_CONFIG["CMTP"]["TREE_CREATION"]
            self.accumulated_tree_creation_energy += tree_creation_energy
            self.tree_created = True
            print(f"⚡ CMTP: 累计树创建能耗 +{tree_creation_energy:.2f}J")
        # ## **** ENERGY MODIFICATION END **** ##
        
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
                
        # 不再需要记录拥塞更新能耗，因为我们使用累积计数器并在最终分摊

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
        修改：在树构建完成前不选择下一跳。
        返回: (best_neighbor, min_ett)
        """
        # 更新协议状态，确保传递正确的仿真时间
        destination_id = packet.destination_id if packet and hasattr(packet, 'destination_id') else None
        self.update_protocol_status([destination_id] if destination_id else None, sim_time)
        
        # 如果树正在构建中且未完成，不选择下一跳
        if self.tree_construction_started and not self.tree_ready:
            # 记录事件
            if packet and hasattr(packet, 'add_event'):
                elapsed = sim_time - (self.tree_build_start_time or sim_time)
                remaining = max(0, self.min_tree_build_time - elapsed)
                info = f"树构建中，进度={self.tree_build_progress:.2f}，已等待={elapsed:.1f}秒, 剩余≈{remaining:.1f}秒"
                packet.add_event("cmtp_waiting_tree", getattr(current_uav, 'id', None), 
                               getattr(packet, 'current_hop_index', None), 
                               sim_time if sim_time is not None else 0, info)
            # 返回None表示暂不转发
            return None, float('inf')

        # 树已构建完成，正常选择下一跳
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

    def get_link_base_delay(self, uav1, uav2=None, root_id=None, visited_nodes=None):
        """
        计算单跳ETX或到RootNode的ETX。
        - 若uav2不为None，则返回uav1到uav2的单跳ETX。
        - 若uav2为None且root_id不为None，则递归计算uav1到root的最小ETX。
        添加缓存提高性能
        """
        if uav2 is not None:
            # 单跳ETX: 1 / PRR(x, y)
            prr = self._get_prr(uav1, uav2)
            if prr == 0:
                return float('inf')
            return 1.0 / prr
        elif root_id is not None:
            # 使用缓存优化递归计算
            # 检查是否有单跳缓存
            if not hasattr(self, '_etx_to_root_cache'):
                self._etx_to_root_cache = {}
                
            cache_key = (uav1.id, root_id)
            # 如果已经有缓存结果，直接返回
            if cache_key in self._etx_to_root_cache:
                return self._etx_to_root_cache[cache_key]
                
            # 到RootNode的ETX: 递归最小{邻居ETX+单跳ETX}
            if uav1.id == root_id:
                # 缓存结果并返回
                self._etx_to_root_cache[cache_key] = 0.0
                return 0.0
                
            # 初始化已访问节点集合，防止循环
            if visited_nodes is None:
                visited_nodes = set()
                
            # 如果当前节点已访问，返回无穷大以避免环路
            if uav1.id in visited_nodes:
                return float('inf')
                
            # 标记当前节点为已访问
            visited_nodes.add(uav1.id)
            
            min_etx = float('inf')
            for neighbor in self._get_neighbors(uav1):
                # 只考虑未访问过的邻居
                if neighbor.id not in visited_nodes:
                    etx_link = self.get_link_base_delay(uav1, neighbor)
                    # 递归计算邻居到根的ETX，传递已访问节点集合
                    etx_neighbor = self.get_link_base_delay(neighbor, None, root_id, visited_nodes.copy())
                    total_etx = etx_link + etx_neighbor
                    if total_etx < min_etx:
                        min_etx = total_etx
                        
            # 计算完成后移除当前节点标记，允许其他路径重用此节点
            visited_nodes.remove(uav1.id)
            
            # 限制缓存大小
            if len(self._etx_to_root_cache) > 2000:  # 允许更大的缓存，因为这个函数递归调用多
                self._etx_to_root_cache.clear()
                
            # 缓存结果
            self._etx_to_root_cache[cache_key] = min_etx
            
            return min_etx
        else:
            raise ValueError("get_link_base_delay: uav2和root_id不能同时为None")

    def _get_prr(self, uav1, uav2):
        """获取uav1到uav2的PRR，基于距离分段随机，使用缓存提高性能"""
        # 计算距离（使用缓存版本）
        dist = self._calculate_distance(uav1, uav2)
        
        # 使用距离区间作为键
        if not hasattr(self, '_prr_cache'):
            self._prr_cache = {}
            
        # 为了避免随机值在每次调用时都不同，我们对距离进行离散化处理
        dist_key = int(dist * 10)  # 0.1的精度
        
        if dist_key in self._prr_cache:
            return self._prr_cache[dist_key]
        
        # 从配置文件获取PRR上下限
        from simulation_config import PRR_MIN, PRR_MAX
        
        # 计算PRR，区间平均分布
        prr = 0
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

    def _get_neighbors(self, uav):
        """获取uav的邻居节点（通信范围内），使用缓存提高性能"""
        # 创建临时缓存键
        cache_key = (id(uav), uav.x, uav.y, uav.z)
        
        # 检查是否有缓存
        if hasattr(self, '_neighbors_cache') and cache_key in self._neighbors_cache:
            return self._neighbors_cache[cache_key]
        
        # 如果没有缓存，则计算邻居
        neighbors = []
        for other in self.uav_map.values():
            if other.id == uav.id:
                continue
            dist = self._calculate_distance(uav, other)
            if dist <= UAV_COMMUNICATION_RANGE:
                neighbors.append(other)
        
        # 初始化缓存（如果需要）并存储结果
        if not hasattr(self, '_neighbors_cache'):
            self._neighbors_cache = {}
        # 限制缓存大小
        if len(self._neighbors_cache) > 1000:
            self._neighbors_cache.clear()  # 防止内存泄漏，定期清空
        self._neighbors_cache[cache_key] = neighbors
        
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
        
        # ## **** ENERGY MODIFICATION START: 记录树维护能耗 **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS:
            tree_maintenance_energy = PROTOCOL_ENERGY_CONFIG["CMTP"]["TREE_MAINTENANCE"]
            self.accumulated_tree_maintenance_energy += tree_maintenance_energy
            print(f"⚡ CMTP: 累计树维护能耗 +{tree_maintenance_energy:.2f}J")
        # ## **** ENERGY MODIFICATION END **** ##
        
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
            # 使用空的visited_nodes集合初始化搜索
            etx_link = self.get_link_base_delay(node, neighbor)
            etx_to_root = self.get_link_base_delay(neighbor, None, root_id, set())
            etx = etx_link + etx_to_root
            
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

    def _calculate_distance(self, uav1, uav2):
        """计算两个UAV之间的欧几里得距离"""
        return self._calculate_distance_cached(
            (uav1.x, uav1.y, uav1.z),
            (uav2.x, uav2.y, uav2.z)
        )
        
    @lru_cache(maxsize=1024)
    def _calculate_distance_cached(self, pos1, pos2):
        """缓存版本的距离计算，使用坐标元组作为参数"""
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2) 