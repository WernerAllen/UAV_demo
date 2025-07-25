# 文件: backend/protocols/dhytp_protocol.py
# 描述: DHyTP 路由协议实现（融合PTP和CMTP，具备完整的拥塞感知和多层树优势）

from .cmtp_protocol import CMTPRoutingModel
from .ptp_protocol import PTPRoutingModel
import time
import math
import random  # 添加random模块导入，用于PRR计算
import numpy as np  # 添加numpy模块导入，用于向量计算
from simulation_config import UAV_COMMUNICATION_RANGE
from functools import lru_cache  # 添加lru_cache用于缓存计算结果

class DHyTPRoutingModel:
    """
    DHyTP协议：融合PTP和CMTP，树构建过程中同时传输数据，树构建好后切换到CMTP。
    特点：
    1. 在构建树的同时也进行数据传输，无需等待树构建完成
    2. 具备完整的CMTP拥塞感知机制
    3. 支持多层虚拟树和动态树自愈
    4. 基于ETT的智能路径选择
    5. 完整实现纯PTP协议功能，包括:
       - 并发传输干扰感知
       - 移动性约束考虑
       - 并发区域延迟计算
    """
    def __init__(self, uav_map):
        self.uav_map = uav_map
        # 保留CMTP和PTP实例用于兼容性，但主要使用自己的增强实现
        self.cmtp = CMTPRoutingModel(uav_map)
        self.ptp = PTPRoutingModel(uav_map)

        # 协议状态控制
        self.use_cmtp = False  # 是否切换到CMTP
        self.tree_construction_started = False  # 是否已开始构建树
        self.destination_list = []  # 目标节点列表
        self.tree_build_progress = 0.0  # 树构建进度(0-1)
        self.tree_build_threshold = 0.6  # 树构建完成阈值，适中的阈值确保合理切换
        self.tree_build_start_time = None  # 记录开始构建树的时间，确保初始化为None
        self.tree_ready = False  # 树是否已经构建完成
        
        # 随机时间区间配置
        self.min_tree_build_time_range = (0.2, 0.5)  # 树构建时间范围(最小值, 最大值)
        self.min_tree_build_time = self._generate_random_build_time()  # 当前实验的随机树构建时间
        
        self.virtual_nodes_history = []  # 记录虚拟树节点数量历史，用于计算增长率
        self.last_update_time = None  # 上次更新时间

        # 增强的CMTP功能
        self.virtual_trees = {}  # 虚拟树结构 {root_id: {node_id: parent_id}}
        self.root_nodes = []  # 根节点列表
        self.root_groups = []  # 合并树的分组
        self.congestion_links = {}  # 拥塞链路映射 {link_tuple: [root_id, ...]}
        self.last_etx_to_root = {}  # 记录上次ETX {(node_id, root_id): etx_value}
        self.last_congestion_update = None  # 上次拥塞信息更新时间

        # CMTP增强参数
        self.ETX_UPDATE_THRESHOLD = 0.3  # ETX变化阈值，超过才更新树
        self.MERGE_DISTANCE_THRESHOLD = 30  # 目标节点合并树的距离阈值
        self.CONGESTION_UPDATE_INTERVAL = 0.5  # 拥塞信息更新间隔(秒)
        
        # ## **** ENERGY MODIFICATION START: 添加能耗累积计数器 **** ##
        self.accumulated_tree_creation_energy = 0.0  # 累积的树创建能耗
        self.accumulated_tree_maintenance_energy = 0.0  # 累积的树维护能耗
        self.accumulated_phase_transition_energy = 0.0  # 累积的阶段转换能耗
        self.tree_created = False  # 标记树是否已创建，避免重复计算树创建能耗
        self.phase_transitioned = False  # 标记是否已进行阶段转换，避免重复计算
        # ## **** ENERGY MODIFICATION END **** ##

    def _generate_random_build_time(self):
        """生成随机树构建时间"""
        import random
        min_time, max_time = self.min_tree_build_time_range
        random_time = random.uniform(min_time, max_time)
        
        # 完全禁用输出
        # 只在首次生成时输出，避免重复输出
        # 添加时间戳检查，确保短时间内不重复输出
        # current_time = time.time()
        # if (not hasattr(self, '_last_build_time_print') or 
        #     current_time - self._last_build_time_print > 2.0):  # 2秒内不重复输出
        #     print(f"◆ DHyTP本次树构建时间设定为: {random_time:.2f}秒")
        #     self._last_build_time_print = current_time
        #     self._has_printed_build_time = True
            
        return random_time

    def reset_protocol_state(self):
        """重置DHyTP协议状态，用于新的实验轮次"""
        self.use_cmtp = False
        self.tree_construction_started = False
        self.destination_list = []
        self.tree_build_progress = 0.0
        self.tree_build_start_time = None
        self.tree_ready = False
        self.virtual_nodes_history = []
        self.last_update_time = None
        self.virtual_trees = {}
        self.root_nodes = []
        self.root_groups = []
        self.congestion_links = {}
        self.last_etx_to_root = {}
        self.last_congestion_update = None
        # 重置输出控制标志
        self._has_printed_build_time = False
        self._last_build_time_print = 0  # 重置时间戳
        
        # 清除所有计算缓存
        if hasattr(self, '_neighbors_cache'):
            self._neighbors_cache.clear()
        if hasattr(self, '_prr_cache'):
            self._prr_cache.clear()
        if hasattr(self, '_link_delay_cache'):
            self._link_delay_cache.clear()
        if hasattr(self, '_etx_to_root_cache'):
            self._etx_to_root_cache.clear()
            
        # ## **** ENERGY MODIFICATION START: 重置能耗累积计数器 **** ##
        self.accumulated_tree_creation_energy = 0.0
        self.accumulated_tree_maintenance_energy = 0.0
        self.accumulated_phase_transition_energy = 0.0
        self.tree_created = False
        self.phase_transitioned = False
        # ## **** ENERGY MODIFICATION END **** ##
            
        # 每次重置时不重新生成随机树构建时间，等到开始构建树时再生成
        print("◆ DHyTP协议状态已重置，准备新的实验轮次")

    def update_protocol_status(self, destination_ids=None, sim_time=None):
        """
        更新协议状态：
        1. 记录目标节点列表
        2. 动态评估树构建进度
        3. 构建多层虚拟树结构
        4. 更新拥塞感知信息
        5. 判断是否可以切换到CMTP（树构建达到阈值时）
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
            self._build_enhanced_virtual_trees()
            
            # 限制输出
            # print(f"◆◆◆ DHyTP开始构建树：目标节点 {self.destination_list}, 预计时间 {self.min_tree_build_time:.2f}秒 ◆◆◆")
            return

        # 如果树已经构建完成并且已经切换到CMTP，继续维护树结构和拥塞信息
        if self.tree_ready and self.use_cmtp:
            # 定期更新拥塞信息和树自愈
            self._update_congestion_info()
            try:
                # 包装在try-except中防止递归错误影响系统稳定性
                self._self_heal_virtual_trees()
            except RecursionError as e:
                print(f"◆ 警告：树自愈过程中遇到递归错误：{str(e)}. 跳过本次自愈操作.")
            except Exception as e:
                print(f"◆ 警告：树自愈过程中遇到错误：{str(e)}. 跳过本次自愈操作.")
            return  # 只有在树已经构建完成并切换到CMTP时才返回

        # 尝试构建DHyTP树并进行PTP->CMTP的转换判断
        if self.tree_construction_started and self.destination_list:
            # 计算时间经过（使用仿真时间）
            if self.tree_build_start_time is None:
                self.tree_build_start_time = current_time
                
            elapsed_time = current_time - self.tree_build_start_time

            # 限制更新频率，每0.1秒最多更新一次（使用仿真时间）
            if self.last_update_time and current_time - self.last_update_time < 0.1:
                return  # 距离上次更新时间太短，跳过本次更新

            self.last_update_time = current_time

            # 计算树构建进度（0-1之间）
            # 使用时间比例，但限制在0-1之间
            time_ratio = min(1.0, elapsed_time / self.min_tree_build_time)
            
            # 使用更平滑的进度函数，初期稍微快一些，后期减慢
            progress = time_ratio ** 0.8  # 指数小于1，使得初期进度快一些
            self.tree_build_progress = progress
            
            # 缓存当前树节点数量
            self.virtual_nodes_history.append(self._count_virtual_tree_nodes())
            
            # 每经过0.5秒输出一次进度
            if int(elapsed_time * 2) > int((elapsed_time - 0.1) * 2):
                # print(f"◆ DHyTP树构建进度: {progress:.2f}, 已用时间: {elapsed_time:.1f}秒")
                pass
                
            # 判断是否可以切换到CMTP（根据进度阈值）
            can_switch = progress >= 1.0 or elapsed_time >= self.min_tree_build_time
            
            # 树已构建完成但尚未标记为tree_ready时，立即标记
            if can_switch and not self.tree_ready:
                # 标记树已构建完成
                self.tree_ready = True

                # 树已构建完成但尚未标记为use_cmtp时，立即标记
                if can_switch and not self.use_cmtp:
                    print(f"\n◆◆◆ DHyTP树构建完成：时间={elapsed_time:.1f}s/{self.min_tree_build_time:.2f}s, 进度={self.tree_build_progress:.2f} ◆◆◆")
                    print(f"◆◆◆ 切换到CMTP模式 ◆◆◆\n")
                
                # ## **** ENERGY MODIFICATION START: 记录阶段转换能耗 **** ##
                from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
                if COLLECT_ENERGY_STATS and not self.phase_transitioned:
                    phase_transition_energy = PROTOCOL_ENERGY_CONFIG["DHYTP"]["PHASE_TRANSITION"]
                    self.accumulated_phase_transition_energy += phase_transition_energy
                    self.phase_transitioned = True
                    print(f"⚡ DHYTP: 累计阶段转换能耗 +{phase_transition_energy:.2f}J")
                # ## **** ENERGY MODIFICATION END **** ##
                
                    self.use_cmtp = True
                    # 最终更新拥塞信息
                    self._update_congestion_info()

    def _build_enhanced_virtual_trees(self):
        """
        构建增强的多层虚拟树结构，支持目标节点合并和拥塞感知
        """
        if not self.destination_list:
            return

        # ## **** ENERGY MODIFICATION START: 记录树创建能耗 **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS and not self.tree_created:
            tree_creation_energy = PROTOCOL_ENERGY_CONFIG["DHYTP"]["TREE_CREATION"]
            self.accumulated_tree_creation_energy += tree_creation_energy
            self.tree_created = True
            print(f"⚡ DHYTP: 累计树创建能耗 +{tree_creation_energy:.2f}J")
        # ## **** ENERGY MODIFICATION END **** ##

        self.root_nodes = []
        self.virtual_trees = {}

        # 将距离较近的目标节点分组
        self.root_groups = self._group_roots_by_distance(self.destination_list)

        for group in self.root_groups:
            # 以第一个目标为主树根
            root_id = group[0]
            self.root_nodes.append(root_id)
            tree = self._build_tree_for_root(root_id)

            # 合并组内其他目标节点的树
            for other_id in group[1:]:
                other_tree = self._build_tree_for_root(other_id)
                tree = self._merge_tree(tree, other_tree)

            self.virtual_trees[root_id] = tree

    def _group_roots_by_distance(self, destination_ids):
        """将距离较近的目标节点分为一组，返回分组列表"""
        groups = []
        used = set()

        for i, id1 in enumerate(destination_ids):
            if id1 in used or id1 not in self.uav_map:
                continue

            group = [id1]
            uav1 = self.uav_map[id1]

            for j, id2 in enumerate(destination_ids):
                if i == j or id2 in used or id2 not in self.uav_map:
                    continue

                uav2 = self.uav_map[id2]
                dist = self._calculate_distance(uav1, uav2)

                if dist < self.MERGE_DISTANCE_THRESHOLD:
                    group.append(id2)
                    used.add(id2)

            used.add(id1)
            groups.append(group)

        return groups

    def _calculate_distance(self, uav1, uav2):
        """统一的距离计算方法，避免代码重复"""
        return self._calculate_distance_cached(
            (uav1.x, uav1.y, uav1.z),
            (uav2.x, uav2.y, uav2.z)
        )
        
    @lru_cache(maxsize=1024)
    def _calculate_distance_cached(self, pos1, pos2):
        """缓存版本的距离计算，使用坐标元组作为参数"""
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2)

    def _add_packet_event(self, packet, event_type, uav_id, info, sim_time=None):
        """统一的事件记录方法，避免代码重复"""
        if packet and hasattr(packet, 'add_event'):
            packet.add_event(event_type, uav_id, getattr(packet, 'current_hop_index', None),
                           sim_time if sim_time is not None else 0, info)

    def _build_tree_for_root(self, root_id):
        """以root_id为根，递归建立虚拟树，返回{node_id: parent_id}映射"""
        if root_id not in self.uav_map:
            return {}

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
                        if parent.id in visited:  # 只考虑已在树中的节点作为父节点
                            etx = self._get_link_base_delay(neighbor, parent)
                            if etx < min_etx:
                                min_etx = etx
                                best_parent = parent

                    if best_parent:
                        tree[neighbor.id] = best_parent.id
                        visited.add(neighbor.id)
                        queue.append(neighbor.id)

        return tree

    def _merge_tree(self, tree1, tree2):
        """合并两棵树，优先保留ETX更小的父节点"""
        merged = dict(tree1)

        for node_id, parent_id in tree2.items():
            if node_id not in merged:
                merged[node_id] = parent_id
            else:
                # 选择ETX更小的父节点
                uav = self.uav_map.get(node_id)
                p1 = self.uav_map.get(merged[node_id]) if merged[node_id] else None
                p2 = self.uav_map.get(parent_id) if parent_id else None

                if uav and p1 and p2:
                    etx1 = self._get_link_base_delay(uav, p1)
                    etx2 = self._get_link_base_delay(uav, p2)
                    if etx2 < etx1:
                        merged[node_id] = parent_id

        return merged

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

    def _get_link_base_delay(self, uav1, uav2):
        """计算单跳ETX: 1 / PRR(x, y)"""
        prr = self._get_prr(uav1, uav2)
        if prr == 0:
            return float('inf')
        return 1.0 / prr

    def _get_prr(self, uav1, uav2):
        """获取uav1到uav2的PRR，基于距离分段随机，使用缓存提高性能"""
        # 创建缓存键：只考虑距离，因为PRR只与距离相关，而不是具体的坐标
        dist = self._calculate_distance(uav1, uav2)
        
        # 使用距离区间作为键
        if not hasattr(self, '_prr_cache'):
            self._prr_cache = {}
            
        # 为了避免随机值在每次调用时都不同，我们对距离进行离散化处理
        dist_key = int(dist * 10)  # 0.1的精度
        
        if dist_key in self._prr_cache:
            return self._prr_cache[dist_key]
        
        # 计算PRR
        prr = 0
        if dist <= 10:
            prr = random.uniform(0.85, 0.9)
        elif dist <= 30:
            prr = random.uniform(0.75, 0.85)
        elif dist <= 60:
            prr = random.uniform(0.65, 0.75)
        elif dist <= 100:
            prr = random.uniform(0.5, 0.65)
        else:
            prr = 0  # 超出范围返回0
        
        # 限制缓存大小
        if len(self._prr_cache) > 1000:
            self._prr_cache.clear()
        
        # 存储结果
        self._prr_cache[dist_key] = prr
        
        return prr

    def _filter_candidates_by_mobility(self, current_uav, candidates, prediction_time=0.4):
        """
        根据移动性约束筛选候选邻居
        prediction_time: 预测时长（秒）
        
        返回满足移动性约束的候选邻居列表
        """
        from simulation_config import UAV_COMMUNICATION_RANGE
        
        valid_candidates = []
        for neighbor in candidates:
            if neighbor.id == current_uav.id:
                continue
                
            # 通信范围约束（当前）
            dist = math.sqrt((current_uav.x - neighbor.x) ** 2 + (current_uav.y - neighbor.y) ** 2)
            if dist > UAV_COMMUNICATION_RANGE:
                continue
                
            # mobility约束：预测T秒后距离
            future_x1 = getattr(current_uav, 'x', 0) + getattr(current_uav, 'vx', 0) * prediction_time
            future_y1 = getattr(current_uav, 'y', 0) + getattr(current_uav, 'vy', 0) * prediction_time
            future_x2 = getattr(neighbor, 'x', 0) + getattr(neighbor, 'vx', 0) * prediction_time
            future_y2 = getattr(neighbor, 'y', 0) + getattr(neighbor, 'vy', 0) * prediction_time
            future_dist = math.sqrt((future_x1 - future_x2) ** 2 + (future_y1 - future_y2) ** 2)
            
            if future_dist <= UAV_COMMUNICATION_RANGE:
                valid_candidates.append(neighbor)
                
        return valid_candidates
        
    def _enhanced_ptp_select_next_hop(self, current_uav, candidate_neighbors, destination_id, packet=None, sim_time=None):
        """
        增强的PTP下一跳选择，完整实现PTP协议的功能
        考虑并发传输干扰和移动性约束
        """
        dest_uav = self.uav_map[destination_id] if destination_id else None
        if not dest_uav:
            return None, float('inf')
            
        # 先根据移动性约束筛选候选邻居
        mobility_filtered_candidates = self._filter_candidates_by_mobility(current_uav, candidate_neighbors)
        
        # 如果筛选后没有候选邻居，则返回None
        if not mobility_filtered_candidates:
            self._add_packet_event(packet, "mobility_constraint", current_uav.id, 
                                 f"all {len(candidate_neighbors)} candidates filtered out by mobility constraint", sim_time)
            return None, float('inf')
            
        # 获取当前网络中的所有发送向量
        all_sending_vectors = self._get_current_sending_vectors()
        
        # 使用PTP的select_next_hop_with_utility方法
        next_hop, utility = self.ptp.select_next_hop_with_utility(
            current_uav, 
            dest_uav, 
            mobility_filtered_candidates,
            all_sending_vectors
        )
        
        # 记录详细的选择过程
        if next_hop:
            # 计算并记录并发区域延迟
            concurrent_delays = {}
            for neighbor in mobility_filtered_candidates:
                if neighbor.id == current_uav.id:
                    continue
                    
                # 计算到每个邻居的并发区域延迟
                concurrent_delay = 0.0
                my_vec = ((current_uav.x, current_uav.y), (neighbor.x, neighbor.y))
                
                for other_vec in all_sending_vectors:
                    if other_vec[0] == my_vec[0] and other_vec[1] == my_vec[1]:
                        continue
                        
                    if self.ptp.are_vectors_concurrent(my_vec[0], my_vec[1], other_vec[0], other_vec[1]):
                        delay = self.ptp.calculate_concurrent_region_delay(
                            my_vec[0], my_vec[1], other_vec[0], other_vec[1]
                        )
                        concurrent_delay += delay
                        
                concurrent_delays[neighbor.id] = concurrent_delay
                
            # 记录事件
            filtered_count = len(candidate_neighbors) - len(mobility_filtered_candidates)
            mobility_info = f"mobility_filtered={filtered_count}"
            candidates_str = ', '.join([f"{nid}:{concurrent_delays.get(nid, 0):.3f}" for nid in concurrent_delays])
            info = f"{mobility_info}, candidates=[{candidates_str}], selected={next_hop.id}, utility={utility:.3f}"
            self._add_packet_event(packet, "enhanced_ptp_select", current_uav.id, info, sim_time)
            
        return next_hop, utility

    def select_next_hop(self, current_uav, candidate_neighbors, destination_id=None, packet=None, sim_time=None):
        """
        根据当前协议状态选择下一跳。
        特点：
        1. 在树构建阶段和树构建完成后都能传输数据
        2. 使用增强的CMTP拥塞感知机制
        3. 基于ETT的智能路径选择

        Args:
            current_uav: 当前UAV节点
            candidate_neighbors: 候选邻居节点列表
            destination_id: 目标节点ID
            packet: 数据包对象（用于记录事件）
            sim_time: 当前仿真时间

        Returns:
            下一跳节点和相关度量值的元组
        """
        # 确保目标节点被添加到destination_list中
        if destination_id and not self.destination_list:
            self.destination_list = [destination_id]
            print(f"◆ DHyTP添加目标节点: {destination_id}")
            
        # 如果树构建尚未开始但有目标节点，则强制开始树构建
        if destination_id and not self.tree_construction_started:
            print(f"◆ DHyTP强制开始树构建: 目标节点={destination_id}")
            self.tree_construction_started = True
            self.tree_build_progress = 0.0
            self.tree_build_start_time = sim_time
            self.last_update_time = sim_time
            self.last_congestion_update = sim_time
            # 生成随机树构建时间
            self.min_tree_build_time = self._generate_random_build_time()
            # 开始构建树
            self._build_enhanced_virtual_trees()
        
        # 更新协议状态
        self.update_protocol_status([destination_id] if destination_id else None, sim_time)
        
        # 检查是否应该切换到CMTP模式（确保树构建完成及时切换）
        if self.tree_construction_started and not self.use_cmtp and sim_time and self.tree_build_start_time:
            elapsed_time = sim_time - self.tree_build_start_time
            # 减少重复输出
            # print(f"◆ DHyTP检查切换条件: 经过时间={elapsed_time:.2f}秒, 阈值={self.min_tree_build_time:.2f}秒")
            if elapsed_time >= self.min_tree_build_time:
                print(f"\n◆◆◆ DHyTP树构建完成 (select_next_hop 中检测): 时间={elapsed_time:.1f}s/{self.min_tree_build_time:.2f}s ◆◆◆")
                print(f"◆◆◆ 切换到CMTP模式 ◆◆◆\n")
                self.use_cmtp = True
                self._update_congestion_info()

        # 添加当前UAV和目标信息
        current_id = getattr(current_uav, 'id', 'unknown')

        # 输出树构建进度（避免重复输出）
        # 减少重复输出，仅在特定条件下打印
        # if self.tree_construction_started and self.tree_build_progress > 0 and not self.use_cmtp:
        #     print(f"◆ 树构建进度: {self.tree_build_progress:.2f}")

        if self.use_cmtp:
            # 已构建完树，使用增强的CMTP模式
            next_hop, metric = self._enhanced_cmtp_select_next_hop(
                current_uav, candidate_neighbors, destination_id, packet, sim_time)

            # 记录事件
            self._add_packet_event(packet, "dhytp_mode", current_uav.id,
                                 f"mode=ENHANCED_CMTP, tree_progress={self.tree_build_progress:.2f}", sim_time)

            # 输出选择的下一跳，总是显示
            if next_hop:
                next_id = getattr(next_hop, 'id', 'unknown')

            return next_hop, metric
        else:
            # 树构建阶段，使用完整的PTP功能
            next_hop, metric = self._enhanced_ptp_select_next_hop(
                current_uav, candidate_neighbors, destination_id, packet, sim_time)

            # 尝试同时构建树（即使在使用PTP）
            if not self.destination_list and destination_id:
                self.destination_list = [destination_id]

            # 记录事件
            self._add_packet_event(packet, "dhytp_mode", current_uav.id,
                                 f"mode=PTP_BUILDING, tree_progress={self.tree_build_progress:.2f}", sim_time)

            return next_hop, metric

    def _enhanced_cmtp_select_next_hop(self, current_uav, candidate_neighbors, destination_id, packet=None, sim_time=None):
        """
        增强的CMTP下一跳选择，基于ETT（Expected Transmission Time）
        考虑拥塞延迟和多层树结构
        """
        min_ett = float('inf')
        best_neighbor = None
        ett_map = {}

        for neighbor in candidate_neighbors:
            # 计算期望传输时间（ETT）
            ett = self._calculate_expected_transmission_time(
                current_uav, neighbor, destination_id, packet, sim_time)
            ett_map[neighbor.id] = ett

            if ett < min_ett:
                min_ett = ett
                best_neighbor = neighbor

        # 记录详细的选择过程
            candidates_str = ', '.join([f"{nid}:{ett_map[nid]:.3f}" for nid in ett_map])
            info = f"candidates=[{candidates_str}], selected={getattr(best_neighbor, 'id', None)}, ett={min_ett:.3f}"
        self._add_packet_event(packet, "enhanced_cmtp_select", getattr(current_uav, 'id', None), info, sim_time)

        return best_neighbor, min_ett

    def _calculate_expected_transmission_time(self, from_uav, to_uav, destination_id=None, packet=None, sim_time=None):
        """
        计算期望传输时间（ETT）= ETX + 拥塞延迟
        增强版本考虑多层树结构和动态拥塞
        """
        # 基础ETX
        etx = self._get_link_base_delay(from_uav, to_uav)

        # 计算拥塞延迟
        congestion_delay = self._calculate_congestion_delay(from_uav, to_uav, destination_id)

        # 计算总ETT
        ett = etx + congestion_delay

        # 记录计算过程
        info = f"from={from_uav.id}, to={to_uav.id}, etx={etx:.3f}, congestion_delay={congestion_delay:.3f}, ett={ett:.3f}"
        self._add_packet_event(packet, "enhanced_ett_calc", getattr(from_uav, 'id', None), info, sim_time)

        return ett

    def _calculate_congestion_delay(self, from_uav, to_uav, destination_id=None):
        """
        计算拥塞延迟，基于链路重叠和并发传输
        destination_id: 目标节点ID，用于未来扩展特定目标的拥塞计算
        """
        # 注意：destination_id参数保留用于未来扩展，当前版本基于链路重叠计算拥塞
        _ = destination_id  # 明确标记参数暂未使用但保留
        if not hasattr(self, 'congestion_links') or not self.congestion_links:
            return 0.0

        from_id, to_id = from_uav.id, to_uav.id
        current_link = tuple(sorted([from_id, to_id]))
        congestion_delay = 0.0

        # 检查当前链路是否与其他链路有拥塞
        for link, roots in self.congestion_links.items():
            if link == current_link:
                continue

            # 检查是否有共同的根节点（表示可能的拥塞）
            current_roots = self.congestion_links.get(current_link, [])
            if set(current_roots) & set(roots):
                # 计算基于PRR和链路利用率的动态拥塞延迟
                prr = self._get_prr(self.uav_map.get(link[0]), self.uav_map.get(link[1]))
                if prr > 0:
                    # 假设链路利用率为0.5（可根据实际流量统计）
                    utilization = 0.5
                    delta_pred = (1.0 / prr) * utilization
                    congestion_delay += delta_pred

        return congestion_delay

    def _update_congestion_info(self):
        """
        更新拥塞感知信息，收集所有虚拟树的链路，找出重叠（并发）链路集合
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

        # ## **** ENERGY MODIFICATION START: 记录拥塞更新能耗 **** ##
        if self.use_cmtp:  # 只在CMTP阶段记录拥塞更新能耗
            # 拥塞更新能耗现在作为树维护能耗的一部分，不再单独计算
            pass
        # ## **** ENERGY MODIFICATION END **** ##

    def _self_heal_virtual_trees(self):
        """
        树自愈机制：只有ETX显著变化才更新树
        """
        if not self.virtual_trees or not self.root_nodes:
            return

        # ## **** ENERGY MODIFICATION START: 记录树维护能耗 **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS:
            tree_maintenance_energy = PROTOCOL_ENERGY_CONFIG["DHYTP"]["TREE_MAINTENANCE"]
            # 将树维护能耗添加到累积计数中，而不是每个数据包上
            self.accumulated_tree_maintenance_energy = getattr(self, 'accumulated_tree_maintenance_energy', 0.0) + tree_maintenance_energy
            print(f"⚡ DHYTP: 累计树维护能耗 +{tree_maintenance_energy:.2f}J")
        # ## **** ENERGY MODIFICATION END **** ##

        for root_id in self.root_nodes:
            if root_id not in self.virtual_trees:
                continue

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

                # 检查链路是否仍然有效
                dist = self._calculate_distance(node, parent)

                if dist > UAV_COMMUNICATION_RANGE:
                    # 寻找新的父节点
                    new_parent, min_etx = self._find_new_parent(node, root_id)

                    # 只有ETX变化大于阈值才更新
                    last_etx = self.last_etx_to_root.get((node_id, root_id), float('inf'))
                    if abs(min_etx - last_etx) > self.ETX_UPDATE_THRESHOLD:
                        tree[node_id] = new_parent.id if new_parent else None
                        self.last_etx_to_root[(node_id, root_id)] = min_etx

    def _find_new_parent(self, node, root_id):
        """在邻居中重选一个到root_id ETX最小且可达的父节点"""
        min_etx = float('inf')
        best_parent = None

        for neighbor in self._get_neighbors(node):
            if neighbor.id == node.id:
                continue

            # 计算到邻居的ETX + 邻居到根的ETX
            etx_to_neighbor = self._get_link_base_delay(node, neighbor)
            etx_neighbor_to_root = self._get_etx_to_root(neighbor, root_id)
            total_etx = etx_to_neighbor + etx_neighbor_to_root

            if total_etx < min_etx:
                min_etx = total_etx
                best_parent = neighbor

        return best_parent, min_etx

    def _get_etx_to_root(self, node, root_id):
        """递归计算节点到根的ETX，使用缓存避免重复计算"""
        # 使用节点ID和根ID作为缓存键
        cache_key = (node.id, root_id)
        
        # 初始化缓存（如果需要）
        if not hasattr(self, '_etx_to_root_cache'):
            self._etx_to_root_cache = {}
        
        # 检查缓存
        if cache_key in self._etx_to_root_cache:
            return self._etx_to_root_cache[cache_key]
            
        # 直接计算情况
        if node.id == root_id:
            self._etx_to_root_cache[cache_key] = 0.0
            return 0.0

        if root_id not in self.virtual_trees:
            self._etx_to_root_cache[cache_key] = float('inf')
            return float('inf')

        tree = self.virtual_trees[root_id]
        if node.id not in tree:
            self._etx_to_root_cache[cache_key] = float('inf')
            return float('inf')

        parent_id = tree[node.id]
        if parent_id is None:
            result = 0.0 if node.id == root_id else float('inf')
            self._etx_to_root_cache[cache_key] = result
            return result

        parent = self.uav_map.get(parent_id)
        if parent is None:
            self._etx_to_root_cache[cache_key] = float('inf')
            return float('inf')

        # 递归计算
        etx_to_parent = self._get_link_base_delay(node, parent)
        etx_parent_to_root = self._get_etx_to_root(parent, root_id)
        
        # 计算结果并缓存
        result = etx_to_parent + etx_parent_to_root
        
        # 限制缓存大小
        if len(self._etx_to_root_cache) > 2000:  # 允许更大的缓存，因为这个函数递归调用多
            self._etx_to_root_cache.clear()
            
        self._etx_to_root_cache[cache_key] = result
        return result

    def get_protocol_state_info(self):
        """
        获取协议当前状态信息（用于调试和监控）
        """
        mode = "ENHANCED_CMTP" if self.use_cmtp else "PTP_BUILDING"
        tree_status = f"构建进度: {self.tree_build_progress:.2%}"

        if self.tree_build_start_time:
            elapsed = time.time() - self.tree_build_start_time
            tree_status += f", 已用时间: {elapsed:.2f}秒"

        # 统计虚拟树信息
        tree_stats = {}
        if self.virtual_trees:
            for root_id, tree in self.virtual_trees.items():
                tree_stats[root_id] = {
                    "nodes_count": len(tree),
                    "max_depth": self._calculate_tree_depth(tree, root_id)
                }

        # 统计拥塞链路信息
        congestion_stats = {
            "total_links": len(self.congestion_links) if hasattr(self, 'congestion_links') else 0,
            "congested_links": sum(1 for links in (self.congestion_links.values() if hasattr(self, 'congestion_links') else []) if len(links) > 1)
        }

        return {
            "mode": mode,
            "tree_status": tree_status,
            "destinations": self.destination_list,
            "tree_build_progress": self.tree_build_progress,
            "tree_build_threshold": self.tree_build_threshold,
            "nodes_history": self.virtual_nodes_history,
            "root_groups": self.root_groups,
            "tree_stats": tree_stats,
            "congestion_stats": congestion_stats,
            "etx_update_threshold": self.ETX_UPDATE_THRESHOLD,
            "merge_distance_threshold": self.MERGE_DISTANCE_THRESHOLD
        }

    def _calculate_tree_depth(self, tree, root_id):
        """计算树的最大深度"""
        if not tree or root_id not in tree:
            return 0

        max_depth = 0

        def dfs(node_id, depth):
            nonlocal max_depth
            max_depth = max(max_depth, depth)

            # 找到所有以node_id为父节点的子节点
            for child_id, parent_id in tree.items():
                if parent_id == node_id:
                    dfs(child_id, depth + 1)

        dfs(root_id, 0)
        return max_depth

    # 添加一些实用的接口方法
    def are_vectors_concurrent(self, p1, q1, p2, q2):
        """
        判断两个向量是否并发（继承自CMTP的接口）
        增强版本考虑实际的拥塞链路信息
        """
        link1 = tuple(sorted([p1, q1]))
        link2 = tuple(sorted([p2, q2]))

        if not hasattr(self, 'congestion_links') or not self.congestion_links:
            return False

        # 检查两个链路是否有共同的根节点
        roots1 = set(self.congestion_links.get(link1, []))
        roots2 = set(self.congestion_links.get(link2, []))

        return bool(roots1 & roots2)

    def calculate_concurrent_region_delay(self, vec1_p1, vec1_q1, vec2_p2, vec2_q2):
        """
        计算并发区域的延迟（继承自CMTP的接口）
        增强版本基于实际的拥塞信息
        """
        if self.are_vectors_concurrent(vec1_p1, vec1_q1, vec2_p2, vec2_q2):
            # 基于链路的PRR计算拥塞延迟
            uav1 = self.uav_map.get(vec1_p1)
            uav2 = self.uav_map.get(vec1_q1)
            if uav1 and uav2:
                prr = self._get_prr(uav1, uav2)
                if prr > 0:
                    return (1.0 / prr) * 0.5  # 假设50%的利用率
        return 0.0

    def get_link_base_delay(self, uav1, uav2):
        """
        获取链路基础延迟（继承自CMTP的接口）
        """
        return self._get_link_base_delay(uav1, uav2)

    def _get_current_sending_vectors(self):
        """
        获取当前网络中所有正在发送的向量
        返回格式: [((x1, y1), (x2, y2)), ...] 表示从(x1,y1)到(x2,y2)的发送向量
        """
        sending_vectors = []
        
        # 如果能从MAC层获取当前传输信息
        if hasattr(self, 'uav_map'):
            for uav_id, uav in self.uav_map.items():
                # 如果UAV正在发送数据包
                if hasattr(uav, 'tx_queue') and uav.tx_queue:
                    packet = uav.tx_queue[0]
                    # 如果包有下一跳信息
                    if hasattr(packet, 'next_hop_id') and packet.next_hop_id in self.uav_map:
                        next_hop = self.uav_map[packet.next_hop_id]
                        # 添加发送向量
                        sending_vectors.append(
                            ((uav.x, uav.y), (next_hop.x, next_hop.y))
                        )
        
        return sending_vectors

    def _count_virtual_tree_nodes(self):
        """计算所有虚拟树的节点数量"""
        if not self.virtual_trees:
            return 0
            
        # 收集所有节点ID
        all_nodes = set()
        for _, tree in self.virtual_trees.items():
            all_nodes.update(tree.keys())
            
        return len(all_nodes)