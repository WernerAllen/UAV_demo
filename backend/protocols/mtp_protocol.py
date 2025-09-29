# 文件: backend/protocols/mtp_protocol.py
# 描述: MTP 路由协议实现（拥塞感知多层树协议）

import math
import time  # 添加time模块导入，用于记录时间
import random  # 添加random模块导入，用于PRR计算
try:
    import numpy as np
except ImportError:
    # 如果没有numpy，使用math模块替代
    np = None
from functools import lru_cache  # 添加lru_cache用于缓存计算结果
from simulation_config import *
from core.uav import UAV

class MTPRoutingModel:
    """
    拥塞感知多层树协议（Multi-Layer Tree Protocol, MTP）骨架
    支持能效树更新、路径合并、动态拥塞延迟
    """
    # 论文MTP增强参数
    ETX_UPDATE_THRESHOLD = 0.3  # ETX变化阈值，超过才更新树
    MERGE_DISTANCE_THRESHOLD = 30  # 目标节点合并树的距离阈值

    def __init__(self, uav_map):
        self.uav_map = uav_map
        self.virtual_trees = None
        self.root_nodes = None
        self.root_groups = None  # 合并树的分组
        self.last_etx_to_root = {}  # 记录上次ETX
        
        # 添加协议状态控制变量，类似DHyTP
        self.tree_construction_started = False  # 是否已开始构建树
        self.destination_list = []  # 目标节点列表
        self.tree_build_progress = 0.0  # 树构建进度(0-1)
        self.tree_build_start_time = None  # 记录开始构建树的时间
        
        # 构建时间配置
        self.min_tree_build_time_range = (0.3, 0.5)  # 树构建时间范围(最小值, 最大值) - 保留向后兼容
        self.min_tree_build_time = None  # 构建时间将在首次运行时动态计算
        self._build_time_calculated = False  # 标记构建时间是否已计算
        
        self.virtual_nodes_history = []  # 记录虚拟树节点数量历史
        self.last_update_time = None  # 上次更新时间
        self.tree_ready = False  # 树是否已经构建完成

        # ## **** ENERGY MODIFICATION START: 添加能耗累积计数器 **** ##
        self.accumulated_tree_creation_energy = 0.0  # 累积的树创建能耗
        self.accumulated_tree_maintenance_energy = 0.0  # 累积的树维护能耗
        self.tree_created = False  # 标记树是否已创建，避免重复计算树创建能耗
        # ## **** ENERGY MODIFICATION END **** ##
        
        # ## **** TREE PRUNING MODIFICATION START: 添加树剪枝相关变量 **** ##
        self.last_etx_update_time = {}  # 记录每个节点上次ETX更新时间
        self.ellipse_regions = {}  # 记录每个源-目标对的椭圆区域信息
        self.pruned_nodes = set()  # 记录被剪枝的节点
        self.pruning_statistics = {}  # 记录剪枝统计信息
        self.pruning_start_time = None  # 剪枝开始时间
        self.total_pruning_operations = 0  # 总剪枝操作数
        # ## **** TREE PRUNING MODIFICATION END **** ##

    def _calculate_realistic_build_time(self):
        """
        基于网络规模和剪枝效果计算真实的树构建时间
        这样可以展示树剪枝对构建时间的实际优化效果
        """
        # 检查当前的剪枝设置
        from simulation_config import TREE_PRUNING_ENABLED as current_pruning_enabled
        
        if not current_pruning_enabled:
            # 未启用剪枝：基于网络规模的基础构建时间，但考虑网络拓扑复杂度
            base_time = len(self.uav_map) * 0.001  # 每个节点需要0.001秒
            complexity_factor = len(self.destination_list) * 0.05  # 目标节点复杂度
            
            # 新增：考虑网络拓扑复杂度（即使无剪枝也应该考虑UAV分布）
            topology_factor = self._calculate_topology_complexity()
            
            total_time = base_time + complexity_factor + topology_factor
            print(f"🔧 MTP无剪枝构建时间: {total_time:.3f}s (节点={len(self.uav_map)}, 目标={len(self.destination_list)}, 拓扑复杂度={topology_factor:.3f}s)")
            return total_time
        else:
            # 启用剪枝：计算剪枝后的实际构建时间
            return self._calculate_pruned_build_time()
    
    def _calculate_pruned_build_time(self):
        """计算启用剪枝后的实际构建时间"""
        if not self.destination_list:
            return 0.1
            
        # 计算所有椭圆区域的剪枝效果
        total_nodes = len(self.uav_map)
        total_active_nodes = 0
        total_pruned_nodes = 0
        
        # 假设第一个目标节点对应的源节点是网络中的第一个节点
        source_nodes = list(self.uav_map.keys())[:len(self.destination_list)]
        
        for i, dest_id in enumerate(self.destination_list):
            if i < len(source_nodes):
                source_id = source_nodes[i]
            else:
                source_id = source_nodes[0]
                
            source_uav = self.uav_map.get(source_id)
            dest_uav = self.uav_map.get(dest_id)
            
            if source_uav and dest_uav:
                # 统计椭圆区域内外的节点
                inside_count = 0
                for node in self.uav_map.values():
                    if node.is_within_ellipse_region(source_uav, dest_uav):
                        inside_count += 1
                        
                total_active_nodes += inside_count
                total_pruned_nodes += (total_nodes - inside_count)
        
        # 计算平均剪枝率
        if total_active_nodes + total_pruned_nodes > 0:
            pruning_rate = total_pruned_nodes / (total_active_nodes + total_pruned_nodes)
        else:
            pruning_rate = 0.0
            
        # 基础构建时间（大幅缩短）
        base_time = total_nodes * 0.001  # 每个节点0.001秒
        complexity_factor = len(self.destination_list) * 0.05  # 目标节点复杂度
        
        # 应用剪枝优化：剪枝率越高，时间减少越多
        pruned_time = (base_time + complexity_factor) * (1 - pruning_rate * 0.7)  # 最多减少70%
        
        print(f"🌳 MTP剪枝构建时间: {pruned_time:.3f}s (剪枝率={pruning_rate*100:.1f}%, 节约={((base_time + complexity_factor - pruned_time)/(base_time + complexity_factor)*100):.1f}%)")
        
        return max(pruned_time, 0.05)  # 最小0.05秒

    def _calculate_topology_complexity(self):
        """
        最简化的拓扑复杂度：基于UAV位置生成确定性随机数
        """
        # 基于UAV位置生成确定性的种子，确保相同分布产生相同结果
        position_seed = sum(int(uav.x) + int(uav.y) for uav in self.uav_map.values()) % 10000
        
        import random
        random.seed(position_seed)
        
        # 生成0.02-0.12秒的随机复杂度
        return random.uniform(0.02, 0.12)

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
            # 计算基于网络规模和剪枝效果的真实构建时间（只在初始化时计算一次）
            if not hasattr(self, '_build_time_calculated') or not self._build_time_calculated:
                self.min_tree_build_time = self._calculate_realistic_build_time()
                self._build_time_calculated = True
                print(f"🕐 MTP构建时间设定: {self.min_tree_build_time:.3f}s")
            
            # ## **** TREE PRUNING MODIFICATION START: 在树构建阶段应用剪枝 **** ##
            if TREE_PRUNING_ENABLED and len(self.destination_list) > 0:
                # 为每个目标节点构建剪枝树
                print(f"🌳 MTP开始剪枝树构建：目标节点 {self.destination_list}")
                self.build_pruned_trees_for_destinations(self.destination_list, current_time)
            else:
                # 开始构建虚拟树结构（原有方法）
                self.build_virtual_tree_structures(self.destination_list)
            # ## **** TREE PRUNING MODIFICATION END **** ##
            
            # 禁用输出
            # print(f"◆ MTP开始构建树：目标节点 {self.destination_list}, 预计时间 {self.min_tree_build_time:.2f}秒")
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

        # 尝试构建MTP树
        if self.tree_construction_started and self.destination_list:
            # 计算时间经过（使用仿真时间）
            # tree_build_start_time 已经在首次调用时设置，这里直接计算elapsed_time
            elapsed_time = current_time - self.tree_build_start_time

            # 限制更新频率，每0.1秒最多更新一次（使用仿真时间）
            # 但如果已经到达切换时间，则不限制更新频率
            # 确保min_tree_build_time已设置，如果没有则不进行切换判断
            if self.min_tree_build_time is None:
                # 如果构建时间还没有计算，跳过本次更新
                if self.last_update_time and current_time - self.last_update_time < 0.1:
                    return
            else:
                # 使用浮点数容差来避免精度问题
                should_switch_by_time = (elapsed_time + 1e-6) >= self.min_tree_build_time
                if (self.last_update_time and current_time - self.last_update_time < 0.1 
                    and not should_switch_by_time and not self.tree_ready):
                    return  # 距离上次更新时间太短，且未达到切换条件，跳过本次更新
            
            build_time_threshold = self.min_tree_build_time
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
                # 使用之前已经计算的build_time_threshold，避免重复定义
                time_factor = min(1.0, elapsed_time / build_time_threshold)
                
                # 进度直接等于时间因子
                self.tree_build_progress = time_factor
                
                # 切换条件：只基于时间因子，与DHyTP保持一致
                # 使用浮点数容差来避免精度问题
                can_switch = (elapsed_time + 1e-6) >= build_time_threshold

                # 树已构建完成但尚未标记为ready时，立即标记为ready
                if can_switch and not self.tree_ready:
                    # 显示树剪枝统计信息
                    pruning_info = self._get_pruning_summary()
                    print(f"\n◆◆◆ MTP树构建完成：时间={elapsed_time:.1f}s/{self.min_tree_build_time:.2f}s, 进度={self.tree_build_progress:.2f} ◆◆◆")
                    if TREE_PRUNING_ENABLED and pruning_info:
                        print(f"🌳 树剪枝统计: {pruning_info}")
                    print()
                    self.tree_ready = True
                    # 最终更新拥塞信息
                    self.update_congestion_info()
                elif not can_switch:
                    # 进度有显著变化时才输出日志
                    progress_change = self.tree_build_progress - old_progress
                    if progress_change >= 0.1:
                        # 禁用输出
                        # print(f"◆ MTP构建进度: {self.tree_build_progress:.2f} (时间:{elapsed_time:.2f}/{self.min_tree_build_time:.2f}秒)")
                        pass
                        
                # 禁用调试输出
                # print(f"◆ MTP构建状态: 进度={self.tree_build_progress:.2f}, 时间={elapsed_time:.2f}/{self.min_tree_build_time:.2f}秒, 可切换={can_switch}, 树已就绪={self.tree_ready}")
                
    def reset_protocol_state(self):
        """重置MTP协议状态，用于新的实验轮次"""
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
        
        # ## **** 重要：清除椭圆区域数据 **** ##
        self.ellipse_regions.clear()
        self.pruned_nodes.clear()
        self.pruning_statistics.clear()
        self.total_pruning_operations = 0
        print("🧹 MTP: 清除椭圆区域和剪枝数据")
        
        # 清除所有计算缓存
        if hasattr(self, '_neighbors_cache'):
            self._neighbors_cache.clear()
        if hasattr(self, '_prr_cache'):
            self._prr_cache.clear()
        if hasattr(self, '_etx_to_root_cache'):
            self._etx_to_root_cache.clear()
            
        # 每次重置时不重新生成随机树构建时间，等到开始构建树时再生成
        print("◆ MTP协议状态已重置，准备新的实验轮次")
        
        # ## **** ENERGY MODIFICATION START: 重置能耗累积计数器 **** ##
        self.accumulated_tree_creation_energy = 0.0
        self.accumulated_tree_maintenance_energy = 0.0
        self.tree_created = False
        # ## **** ENERGY MODIFICATION END **** ##
        
        # 重置构建时间计算标志
        self._build_time_calculated = False

    def build_virtual_tree_structures(self, destination_ids=None, source_id=None):
        """
        构建多层虚拟树结构，选择RootNodes。
        论文MTP增强：支持目标节点合并树
        树剪枝增强：支持基于椭圆区域的树构建优化
        
        Args:
            destination_ids: 目标节点ID列表
            source_id: 源节点ID（用于树剪枝）
        """
        if destination_ids is None:
            destination_ids = list(self.uav_map.keys())
        self.root_nodes = []
        self.virtual_trees = {}
        
        # ## **** ENERGY MODIFICATION START: 记录树创建能耗 **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS and not self.tree_created:
            tree_creation_energy = PROTOCOL_ENERGY_CONFIG["MTP"]["TREE_CREATION"]
            self.accumulated_tree_creation_energy += tree_creation_energy
            self.tree_created = True
            print(f"⚡ MTP: 累计树创建能耗 +{tree_creation_energy:.2f}J")
        # ## **** ENERGY MODIFICATION END **** ##
        
        # 论文MTP增强：路径合并机制
        self.root_groups = self._group_roots_by_distance(destination_ids)
        
        # 注意：这里不记录椭圆区域，等待后续调用record_actual_source_dest_pairs来记录
        # 避免重复记录导致椭圆数量与路径不匹配
        
        for group in self.root_groups:
            # 以第一个目标为主树根
            root_id = group[0]
            self.root_nodes.append(root_id)
            
            # ## **** TREE PRUNING MODIFICATION START: 使用剪枝树构建 **** ##
            if TREE_PRUNING_ENABLED and source_id:
                # 使用基于椭圆区域的剪枝树构建
                tree = self.build_pruned_tree_for_pair(source_id, root_id)
            else:
                # 使用原有的树构建方法
                tree = self._build_tree_for_root(root_id)
            # ## **** TREE PRUNING MODIFICATION END **** ##
            
            # 合并组内其他目标节点
            for other_id in group[1:]:
                if TREE_PRUNING_ENABLED and source_id:
                    other_tree = self.build_pruned_tree_for_pair(source_id, other_id)
                else:
                    other_tree = self._build_tree_for_root(other_id)
                tree = self._merge_tree(tree, other_tree)
            self.virtual_trees[root_id] = tree
        
        print(f"🌳 MTP: 构建完成，共{len(self.root_groups)}个根组，椭圆区域数量: {len(self.ellipse_regions)}")

    def record_actual_source_dest_pairs(self, source_dest_pairs):
        """记录实际的源-目标对的椭圆区域信息"""
        print(f"🎯 MTP: 记录实际源-目标对的椭圆区域，共{len(source_dest_pairs)}对")
        
        # 清除之前的椭圆区域记录
        self.ellipse_regions.clear()
        
        for pair in source_dest_pairs:
            source_id = pair.get('source')
            dest_id = pair.get('destination')
            if source_id is not None and dest_id is not None:
                self._record_ellipse_region(source_id, dest_id)

    def _record_ellipse_region(self, source_id, dest_id):
        """记录源-目标对的椭圆区域信息"""
        source_uav = self.uav_map.get(source_id)
        dest_uav = self.uav_map.get(dest_id)
        
        if source_uav and dest_uav:
            ellipse_key = (source_id, dest_id)
            self.ellipse_regions[ellipse_key] = {
                'source': source_uav,
                'destination': dest_uav,
                'last_update': time.time()
            }
            print(f"🔍 MTP: 记录椭圆区域 {source_id}→{dest_id}")

    def _group_roots_by_distance(self, destination_ids):
        """将距离较近的目标节点分为一组，返回分组列表。"""
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
        """合并两棵树，优先保留ETX更小的父节点。"""
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
                link = tuple(sorted([node_id, parent_id]))  # 无向链路（使用ID进行排序）
                if link not in self.congestion_links:
                    self.congestion_links[link] = []
                self.congestion_links[link].append(root_id)
                
        # 不再需要记录拥塞更新能耗，因为我们使用累积计数器并在最终分摊

    def calculate_expected_transmission_time(self, from_uav, to_uav, layer=0, packet=None, sim_time=None):
        """
       Δ_pred动态计算（基于PRR和链路利用率）
        """
        etx = self.get_link_base_delay(from_uav, to_uav)
        congestion_delay = 0.0
        if hasattr(self, 'congestion_links'):
            from_id, to_id = from_uav.id, to_uav.id
            link = tuple(sorted([from_id, to_id]))  # 使用ID进行排序
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
        树剪枝增强：使用椭圆区域过滤候选邻居
        返回: (best_neighbor, min_ett)
        """
        # 更新协议状态，确保传递正确的仿真时间
        destination_id = packet.destination_id if packet and hasattr(packet, 'destination_id') else None
        self.update_protocol_status([destination_id] if destination_id else None, sim_time)
        
        # ## **** TREE PRUNING MODIFICATION START: 应用树剪枝到ETX更新 **** ##
        if TREE_PRUNING_ENABLED and destination_id and sim_time:
            source_id = getattr(current_uav, 'id', None)
            if source_id:
                self.update_etx_with_pruning(source_id, destination_id, sim_time)
        # ## **** TREE PRUNING MODIFICATION END **** ##
        
        # 如果树正在构建中且未完成，不选择下一跳
        if self.tree_construction_started and not self.tree_ready:
            # 记录事件
            if packet and hasattr(packet, 'add_event'):
                elapsed = sim_time - (self.tree_build_start_time or sim_time)
                remaining = max(0, self.min_tree_build_time - elapsed)
                info = f"树构建中，进度={self.tree_build_progress:.2f}，已等待={elapsed:.1f}秒, 剩余≈{remaining:.1f}秒"
                packet.add_event("mtp_waiting_tree", getattr(current_uav, 'id', None), 
                               getattr(packet, 'current_hop_index', None), 
                               sim_time if sim_time is not None else 0, info)
            # 返回None表示暂不转发
            return None, float('inf')

        # 树已构建完成，正常选择下一跳
        # ## **** TREE PRUNING MODIFICATION START: 过滤候选邻居 **** ##
        if TREE_PRUNING_ENABLED and destination_id:
            source_id = getattr(current_uav, 'id', None)
            if source_id:
                # 使用椭圆区域过滤候选邻居
                pruned_neighbors = self.get_pruned_neighbors(current_uav, source_id, destination_id)
                original_count = len(candidate_neighbors)
                pruned_count = len(pruned_neighbors)
                if pruned_count < original_count:
                    print(f"🌳 MTP邻居剪枝: {current_uav.id}→{destination_id} | 原始邻居={original_count} | 剪枝后邻居={pruned_count} | 剪枝率={(original_count-pruned_count)/original_count*100:.1f}%")
                candidate_neighbors = pruned_neighbors
        # ## **** TREE PRUNING MODIFICATION END **** ##
        
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
            packet.add_event("mtp_path_select", getattr(current_uav, 'id', None), getattr(packet, 'current_hop_index', None), sim_time if sim_time is not None else 0, info)
        
        # 注释掉这里的输出，将在数据包传输成功后显示
        # if best_neighbor:
        #     # 计算进度值：ETT越小，进度越高，最大为1.0
        #     if min_ett > 0:
        #         progress = min(1.0, 1.0 / min_ett)  # ETT的倒数，限制最大值为1.0
        #     else:
        #         progress = 1.0
        #     print(f"【MTP】 UAV-{current_uav.id}→{best_neighbor.id} 选择→UAV-{best_neighbor.id} 进度:{progress:.2f}")
            
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
        """
        MTP下判断向量并发：基于拥塞链路信息检查两条传输向量是否并发
        采用DHyTP协议的实用方法，基于实际的拥塞链路信息而不是复杂的几何计算
        
        Args:
            p1, q1: 第一条向量的起点和终点坐标/ID
            p2, q2: 第二条向量的起点和终点坐标/ID
            
        Returns:
            bool: 如果两条向量并发则返回True
        """
        # 将向量端点转换为链路表示
        link1 = tuple(sorted([p1, q1]))
        link2 = tuple(sorted([p2, q2]))

        # 检查是否有拥塞链路信息
        if not hasattr(self, 'congestion_links') or not self.congestion_links:
            return False

        # 检查两个链路是否有共同的根节点（表示并发）
        roots1 = set(self.congestion_links.get(link1, []))
        roots2 = set(self.congestion_links.get(link2, []))

        return bool(roots1 & roots2)

    def calculate_concurrent_region_delay(self, vec1_p1, vec1_q1, vec2_p2, vec2_q2):
        """
        计算并发区域的延迟：采用DHyTP协议的实用方法
        基于实际的拥塞信息和PRR计算，而不是复杂的空间计算
        
        Args:
            vec1_p1, vec1_q1: 第一条向量的起点和终点
            vec2_p2, vec2_q2: 第二条向量的起点和终点
            
        Returns:
            float: 并发延迟惩罚值
        """
        # 检查两条向量是否并发
        if self.are_vectors_concurrent(vec1_p1, vec1_q1, vec2_p2, vec2_q2):
            # 基于链路的PRR计算拥塞延迟（采用DHyTP的方法）
            uav1 = self.uav_map.get(vec1_p1) if isinstance(vec1_p1, (int, str)) else None
            uav2 = self.uav_map.get(vec1_q1) if isinstance(vec1_q1, (int, str)) else None
            
            # 如果输入是坐标而不是UAV ID，则寻找最近的UAV
            if uav1 is None and isinstance(vec1_p1, (tuple, list)) and len(vec1_p1) >= 2:
                uav1 = self._find_closest_uav(vec1_p1[0], vec1_p1[1])
            if uav2 is None and isinstance(vec1_q1, (tuple, list)) and len(vec1_q1) >= 2:
                uav2 = self._find_closest_uav(vec1_q1[0], vec1_q1[1])
                
            if uav1 and uav2:
                prr = self._get_prr(uav1, uav2)
                if prr > 0:
                    # 使用DHyTP的计算方法：基于PRR和假设的50%利用率
                    return (1.0 / prr) * 0.5
                    
        return 0.0

    def _find_closest_uav(self, x, y):
        """根据坐标找到最近的UAV"""
        min_distance = float('inf')
        closest_uav = None
        
        for uav in self.uav_map.values():
            dist = math.sqrt((uav.x - x) ** 2 + (uav.y - y) ** 2)
            if dist < min_distance:
                min_distance = dist
                closest_uav = uav
                
        return closest_uav
    
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
            tree_maintenance_energy = PROTOCOL_ENERGY_CONFIG["MTP"]["TREE_MAINTENANCE"]
            self.accumulated_tree_maintenance_energy += tree_maintenance_energy
            print(f"⚡ MTP: 累计树维护能耗 +{tree_maintenance_energy:.2f}J")
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
    
    # ## **** TREE PRUNING MODIFICATION START: 树剪枝机制实现 **** ##
    
    def update_etx_with_pruning(self, source_id, destination_id, sim_time):
        """
        基于树剪枝机制的ETX更新方法
        只有椭圆区域内的节点才更新ETX，区域外的节点不更新
        
        Args:
            source_id: 源节点ID
            destination_id: 目标节点ID  
            sim_time: 当前仿真时间
        """
        if not TREE_PRUNING_ENABLED:
            # 如果未启用树剪枝，使用原有的ETX更新机制
            self._update_all_etx(source_id, destination_id, sim_time)
            return
            
        # 获取源节点和目标节点
        source_uav = self.uav_map.get(source_id)
        destination_uav = self.uav_map.get(destination_id)
        
        if not source_uav or not destination_uav:
            return
            
        # 创建椭圆区域键
        ellipse_key = (source_id, destination_id)
        
        # 检查是否需要更新ETX（基于时间间隔）
        if ellipse_key in self.last_etx_update_time:
            time_since_last_update = sim_time - self.last_etx_update_time[ellipse_key]
            if time_since_last_update < PRUNING_UPDATE_INTERVAL:
                return  # 还未到更新时间
                
        # 更新椭圆区域信息
        self.ellipse_regions[ellipse_key] = {
            'source': source_uav,
            'destination': destination_uav,
            'last_update': sim_time
        }
        
        # 遍历所有节点，只更新椭圆区域内节点的ETX
        updated_count = 0
        pruned_count = 0
        
        for node_id, node in self.uav_map.items():
            if node.is_within_ellipse_region(source_uav, destination_uav):
                # 节点在椭圆区域内，更新ETX
                self._update_node_etx(node, destination_id)
                updated_count += 1
            else:
                # 节点在椭圆区域外，不更新ETX，标记为被剪枝
                if node_id not in self.pruned_nodes:
                    self.pruned_nodes.add(node_id)
                    pruned_count += 1
                    
        # 记录更新时间
        self.last_etx_update_time[ellipse_key] = sim_time
        
        # 记录剪枝统计信息
        self._update_pruning_statistics(source_id, destination_id, updated_count, pruned_count, sim_time)
        
        # 显示剪枝执行信息
        efficiency = (pruned_count / (updated_count + pruned_count)) * 100 if (updated_count + pruned_count) > 0 else 0
        print(f"🌳 MTP树剪枝执行: 源={source_id}→目标={destination_id} | 活跃节点={updated_count} | 剪枝节点={pruned_count} | 剪枝率={efficiency:.1f}%")
    
    def _update_all_etx(self, source_id, destination_id, sim_time):
        """原有的ETX更新机制（不使用树剪枝）"""
        for node_id, node in self.uav_map.items():
            self._update_node_etx(node, destination_id)
    
    def _update_node_etx(self, node, destination_id):
        """更新单个节点的ETX值"""
        if self.virtual_trees and destination_id in self.virtual_trees:
            tree = self.virtual_trees[destination_id]
            if node.id in tree:
                # 计算到目标节点的ETX
                etx = self.get_link_base_delay(node, None, destination_id, set())
                node.etx_to_root = etx
        else:
            # 如果没有虚拟树，直接计算ETX
            dest_uav = self.uav_map.get(destination_id)
            if dest_uav:
                etx = self.get_link_base_delay(node, dest_uav)
                node.etx_to_root = etx
    
    def build_pruned_tree_for_pair(self, source_id, destination_id):
        """
        为特定的源-目标对构建剪枝后的树结构
        
        Args:
            source_id: 源节点ID
            destination_id: 目标节点ID
            
        Returns:
            dict: 剪枝后的树结构 {node_id: parent_id}
        """
        if not TREE_PRUNING_ENABLED:
            # 如果未启用树剪枝，使用原有的树构建方法
            return self._build_tree_for_root(destination_id)
            
        source_uav = self.uav_map.get(source_id)
        destination_uav = self.uav_map.get(destination_id)
        
        if not source_uav or not destination_uav:
            return {}
            
        # 构建剪枝后的树
        pruned_tree = {destination_id: None}  # 目标节点作为根节点
        visited = set([destination_id])
        queue = [destination_id]
        
        while queue:
            current_id = queue.pop(0)
            current_uav = self.uav_map[current_id]
            
            # 获取邻居节点，但只考虑椭圆区域内的节点
            for neighbor in self._get_neighbors(current_uav):
                if (neighbor.id not in visited and 
                    neighbor.is_within_ellipse_region(source_uav, destination_uav)):
                    
                    # 选择ETX最小的父节点
                    min_etx = float('inf')
                    best_parent = None
                    
                    for parent in self._get_neighbors(neighbor):
                        if (parent.is_within_ellipse_region(source_uav, destination_uav) and
                            parent.id in visited):
                            etx = self.get_link_base_delay(neighbor, parent)
                            if etx < min_etx:
                                min_etx = etx
                                best_parent = parent
                                
                    if best_parent:
                        pruned_tree[neighbor.id] = best_parent.id
                        visited.add(neighbor.id)
                        queue.append(neighbor.id)
                        
        # 计算剪枝效果
        total_nodes = len(self.uav_map)
        pruning_efficiency = ((total_nodes - len(pruned_tree)) / total_nodes) * 100 if total_nodes > 0 else 0
        
        print(f"🌳 MTP剪枝树构建: 源={source_id}→目标={destination_id} | 树节点={len(pruned_tree)}/{total_nodes} | 剪枝效率={pruning_efficiency:.1f}%")
        return pruned_tree
    
    def get_pruned_neighbors(self, node, source_id, destination_id):
        """
        获取节点在椭圆区域内的邻居节点
        
        Args:
            node: 当前节点
            source_id: 源节点ID
            destination_id: 目标节点ID
            
        Returns:
            list: 椭圆区域内的邻居节点列表
        """
        if not TREE_PRUNING_ENABLED:
            return self._get_neighbors(node)
            
        source_uav = self.uav_map.get(source_id)
        destination_uav = self.uav_map.get(destination_id)
        
        if not source_uav or not destination_uav:
            return self._get_neighbors(node)
            
        # 过滤出椭圆区域内的邻居
        pruned_neighbors = []
        for neighbor in self._get_neighbors(node):
            if neighbor.is_within_ellipse_region(source_uav, destination_uav):
                pruned_neighbors.append(neighbor)
                
        return pruned_neighbors
    
    def is_node_pruned(self, node_id):
        """检查节点是否被剪枝"""
        return node_id in self.pruned_nodes
    
    def get_ellipse_region_info(self, source_id, destination_id):
        """获取椭圆区域信息"""
        ellipse_key = (source_id, destination_id)
        return self.ellipse_regions.get(ellipse_key, None)
    
    def _update_pruning_statistics(self, source_id, destination_id, updated_count, pruned_count, sim_time):
        """更新剪枝统计信息"""
        if not TREE_PRUNING_ENABLED:
            return
            
        if self.pruning_start_time is None:
            self.pruning_start_time = sim_time
            
        ellipse_key = (source_id, destination_id)
        if ellipse_key not in self.pruning_statistics:
            self.pruning_statistics[ellipse_key] = {
                'total_updates': 0,
                'total_active_nodes': 0,
                'total_pruned_nodes': 0,
                'first_update_time': sim_time,
                'last_update_time': sim_time
            }
        
        stats = self.pruning_statistics[ellipse_key]
        stats['total_updates'] += 1
        stats['total_active_nodes'] += updated_count
        stats['total_pruned_nodes'] += pruned_count
        stats['last_update_time'] = sim_time
        
        self.total_pruning_operations += 1
    
    def _get_pruning_summary(self):
        """获取剪枝统计总结"""
        if not TREE_PRUNING_ENABLED or not self.pruning_statistics:
            return ""
            
        total_active = sum(stats['total_active_nodes'] for stats in self.pruning_statistics.values())
        total_pruned = sum(stats['total_pruned_nodes'] for stats in self.pruning_statistics.values())
        total_nodes = total_active + total_pruned
        
        if total_nodes == 0:
            return ""
            
        pruning_rate = (total_pruned / total_nodes) * 100
        ellipse_pairs = len(self.pruning_statistics)
        
        return f"椭圆对={ellipse_pairs}, 剪枝操作={self.total_pruning_operations}, 活跃节点={total_active}, 剪枝节点={total_pruned}, 总剪枝率={pruning_rate:.1f}%"
    
    def display_pruning_progress(self, sim_time):
        """显示剪枝进度信息"""
        if not TREE_PRUNING_ENABLED or not self.pruning_statistics:
            return
            
        if self.pruning_start_time is None:
            return
            
        elapsed_time = sim_time - self.pruning_start_time
        active_ellipses = len([k for k, v in self.pruning_statistics.items() 
                             if sim_time - v['last_update_time'] < PRUNING_UPDATE_INTERVAL * 2])
        
        print(f"🌳 MTP剪枝状态: 运行时间={elapsed_time:.1f}s | 活跃椭圆区域={active_ellipses} | 总剪枝操作={self.total_pruning_operations}")
    
    def build_pruned_trees_for_destinations(self, destination_list, sim_time):
        """
        为目标节点列表构建剪枝树（在树构建阶段执行）
        这是树剪枝机制的核心：在构建阶段就减少不必要的计算
        
        注意：这个方法不记录椭圆区域，椭圆区域由record_actual_source_dest_pairs统一管理
        
        Args:
            destination_list: 目标节点ID列表
            sim_time: 当前仿真时间
        """
        if not TREE_PRUNING_ENABLED:
            # 如果未启用剪枝，使用原有方法
            self.build_virtual_tree_structures(destination_list)
            return
            
        print(f"🌳 MTP树剪枝构建开始: {len(destination_list)} 个目标节点")
        
        # 初始化剪枝统计
        if self.pruning_start_time is None:
            self.pruning_start_time = sim_time
            
        self.root_nodes = []
        self.virtual_trees = {}
        
        # ## **** ENERGY MODIFICATION START: 记录树创建能耗 **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS and not self.tree_created:
            tree_creation_energy = PROTOCOL_ENERGY_CONFIG["MTP"]["TREE_CREATION"]
            self.accumulated_tree_creation_energy += tree_creation_energy
            self.tree_created = True
            print(f"⚡ MTP: 累计树创建能耗 +{tree_creation_energy:.2f}J")
        # ## **** ENERGY MODIFICATION END **** ##
        
        # 假设第一个目标节点对应的源节点是网络中的第一个节点
        # 在实际应用中，源节点应该从数据包或其他上下文中获取
        source_nodes = list(self.uav_map.keys())[:len(destination_list)]
        
        total_original_nodes = 0
        total_pruned_nodes = 0
        
        # 为每个源-目标对构建剪枝树
        for i, dest_id in enumerate(destination_list):
            if i < len(source_nodes):
                source_id = source_nodes[i]
            else:
                source_id = source_nodes[0]  # 默认使用第一个源节点
                
            source_uav = self.uav_map.get(source_id)
            dest_uav = self.uav_map.get(dest_id)
            
            if source_uav and dest_uav:
                # 计算椭圆区域
                focal_distance = math.sqrt(
                    (source_uav.x - dest_uav.x) ** 2 + 
                    (source_uav.y - dest_uav.y) ** 2 + 
                    (source_uav.z - dest_uav.z) ** 2
                )
                
                # 统计椭圆区域内外的节点
                inside_count = 0
                outside_count = 0
                for node in self.uav_map.values():
                    if node.is_within_ellipse_region(source_uav, dest_uav):
                        inside_count += 1
                    else:
                        outside_count += 1
                        
                total_original_nodes += len(self.uav_map)
                total_pruned_nodes += outside_count
                
                # 构建剪枝树
                pruned_tree = self.build_pruned_tree_for_pair(source_id, dest_id)
                self.virtual_trees[dest_id] = pruned_tree
                self.root_nodes.append(dest_id)
                
                print(f"🌳 椭圆区域 {source_id}→{dest_id}: 焦点距离={focal_distance:.1f}m, 椭圆内={inside_count}, 椭圆外={outside_count}")
        
        # 显示总体剪枝效果
        if total_original_nodes > 0:
            overall_pruning_rate = (total_pruned_nodes / total_original_nodes) * 100
            print(f"🌳 MTP树构建剪枝完成: 总节点={total_original_nodes}, 剪枝节点={total_pruned_nodes}, 总剪枝率={overall_pruning_rate:.1f}%")
        
        # 注意：椭圆区域信息由record_actual_source_dest_pairs统一管理，这里不再重复记录
    
    # ## **** TREE PRUNING MODIFICATION END **** ## 