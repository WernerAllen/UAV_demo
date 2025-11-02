# 文件: backend/protocols/dhytp_protocol.py
# 描述: DHyTP 路由协议实现（融合PTP和MTP，具备完整的拥塞感知和多层树优势）

from .mtp_protocol import MTPRoutingModel
from .ptp_protocol import PTPRoutingModel
import time
import math
import random
from simulation_config import UAV_COMMUNICATION_RANGE, TREE_PRUNING_ENABLED, PRUNING_UPDATE_INTERVAL
from functools import lru_cache

class DHyTPRoutingModel:
    """
    DHyTP协议：融合PTP和MTP，树构建过程中同时传输数据，树构建好后切换到MTP。
    特点：
    1. 在构建树的同时也进行数据传输，无需等待树构建完成
    2. 具备完整的MTP拥塞感知机制
    3. 支持多层虚拟树和动态树自愈
    4. 基于ETT的智能路径选择
    5. 完整实现纯PTP协议功能，包括:
       - 并发传输干扰感知
       - 移动性约束考虑
       - 并发区域延迟计算
    """
    def __init__(self, uav_map):
        self.uav_map = uav_map
        # 保留MTP和PTP实例用于兼容性，但主要使用自己的增强实现
        self.mtp = MTPRoutingModel(uav_map)
        self.ptp = PTPRoutingModel(uav_map)

        # 协议状态控制
        self.use_mtp = False  # 是否切换到MTP
        self.tree_construction_started = False  # 是否已开始构建树
        self.destination_list = []  # 目标节点列表
        self.tree_build_progress = 0.0  # 树构建进度(0-1)
        self.tree_build_threshold = 0.6  # 树构建完成阈值，适中的阈值确保合理切换
        self.tree_build_start_time = None  # 记录开始构建树的时间，确保初始化为None
        self.tree_ready = False  # 树是否已经构建完成
        
        # 构建时间配置
        self.min_tree_build_time_range = (0.2, 0.5)  # 树构建时间范围(最小值, 最大值) - 保留向后兼容
        self.min_tree_build_time = None  # 构建时间将在首次运行时动态计算
        self._build_time_calculated = False  # 标记构建时间是否已计算
        
        self.virtual_nodes_history = []  # 记录虚拟树节点数量历史，用于计算增长率
        self.last_update_time = None  # 上次更新时间

        # 增强的MTP功能
        self.virtual_trees = {}  # 虚拟树结构 {root_id: {node_id: parent_id}}
        self.root_nodes = []  # 根节点列表
        self.root_groups = []  # 合并树的分组
        self.congestion_links = {}  # 拥塞链路映射 {link_tuple: [root_id, ...]}
        self.last_etx_to_root = {}  # 记录上次ETX {(node_id, root_id): etx_value}
        self.last_congestion_update = None  # 上次拥塞信息更新时间

        # MTP增强参数
        self.ETX_UPDATE_THRESHOLD = 0.3  # ETX变化阈值，超过才更新树
        self.MERGE_DISTANCE_THRESHOLD = 30  # 目标节点合并树的距离阈值
        self.CONGESTION_UPDATE_INTERVAL = 0.5  # 拥塞信息更新间隔(秒)
        
        # ## **** ENERGY MODIFICATION START: 添加能耗累积计数器 **** ##
        self.packet_count = 0  # 数据包计数器（用于计算树创建能耗）
        self.etx_update_count = 0  # ETX更新次数计数器（用于计算树维护能耗）
        self.accumulated_tree_creation_energy = 0.0  # 累积的树创建能耗（已考虑剪枝节省）
        self.accumulated_tree_maintenance_energy = 0.0  # 累积的树维护能耗
        self.accumulated_phase_transition_energy = 0.0  # 累积的阶段转换能耗
        self.base_tree_creation_energy_per_packet = 0.0  # 每个数据包的基础树创建能耗
        self.pruning_save_rate = 0.0  # 剪枝节省率（0-1之间）
        # ## **** ENERGY MODIFICATION END **** ##
        
        # ## **** TREE PRUNING MODIFICATION START: 添加树剪枝相关变量 **** ##
        self.last_etx_update_time = {}  # 记录每个节点上次ETX更新时间
        self.ellipse_regions = {}  # 记录每个源-目标对的椭圆区域信息
        self.pruned_nodes = set()  # 记录被剪枝的节点
        self.pruning_statistics = {}  # 记录剪枝统计信息
        self.pruning_start_time = None  # 剪枝开始时间
        self.total_pruning_operations = 0  # 总剪枝操作数
        self.pruning_energy_saved = 0.0  # 树剪枝节省的能耗
        self.total_pruning_rate = 0.0  # 总剪枝率（用于统计）
        # ## **** TREE PRUNING MODIFICATION END **** ##

        # ## **** PATH MERGE MODIFICATION START: 添加路径合并相关变量 **** ##
        # 与MTP保持一致的统计与状态，以便在DHYTP完成树构建后执行路径段合并
        self.merged_paths = {}  # 记录已合并的路径段 {(path_id1, path_id2): merged_segment}
        self.path_segments = {}  # 记录所有路径段 {root_id: [segments]}
        self.merge_statistics = {}  # 路径合并统计信息
        self.total_merge_operations = 0  # 总合并操作数
        self.merge_energy_saved = 0.0  # 路径合并节省的能耗
        # ## **** PATH MERGE MODIFICATION END **** ##

    def _calculate_realistic_build_time(self):
        """
        基于网络规模和剪枝效果计算真实的树构建时间
        DHyTP版本：考虑PTP到MTP的转换开销和拓扑复杂度
        """
        if not TREE_PRUNING_ENABLED:
            # 未启用剪枝：基于网络规模的基础构建时间
            base_time = len(self.uav_map) * 0.0005  # 每个节点需要0.001秒
            complexity_factor = len(self.destination_list) * 0.05  # 目标节点复杂度
            phase_transition_cost = 0.02  # PTP到MTP转换开销
            
            # 新增：考虑网络拓扑复杂度（基于UAV位置分布）
            topology_factor = self._calculate_topology_complexity()
            
            total_time = base_time + complexity_factor + phase_transition_cost + topology_factor
            print(f"🔧 DHyTP无剪枝构建时间: {total_time:.3f}s (节点={len(self.uav_map)}, 目标={len(self.destination_list)}, 拓扑复杂度={topology_factor:.3f}s)")
            return total_time
        else:
            # 启用剪枝：计算剪枝后的实际构建时间
            return self._calculate_pruned_build_time_dhytp()
    
    def _calculate_pruned_build_time_dhytp(self):
        """计算DHyTP启用剪枝后的实际构建时间"""
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
        phase_transition_cost = 0.02  # PTP到MTP转换开销
        
        # 应用剪枝优化：剪枝率越高，时间减少越多
        pruned_time = (base_time + complexity_factor + phase_transition_cost) * (1 - pruning_rate * 0.3)  # 最多减少30%
        
        print(f"🌳 DHyTP剪枝构建时间: {pruned_time:.3f}s (剪枝率={pruning_rate*100:.1f}%, 节约={((base_time + complexity_factor + phase_transition_cost - pruned_time)/(base_time + complexity_factor + phase_transition_cost)*100):.1f}%)")
        
        return max(pruned_time, 0.05)  # 最小0.05秒

    def _calculate_topology_complexity(self):
        """
        计算拓扑复杂度：基于UAV位置生成确定性随机数
        确保相同的UAV分布产生相同的复杂度值
        """
        # 基于UAV位置生成确定性的种子，确保相同分布产生相同结果
        position_seed = sum(int(uav.x) + int(uav.y) for uav in self.uav_map.values()) % 10000
        
        import random
        random.seed(position_seed)
        
        # 生成0.02-0.12秒的随机复杂度
        return random.uniform(-0.12, 0.12)

    def reset_protocol_state(self):
        """重置DHyTP协议状态，用于新的实验轮次"""
        self.use_mtp = False
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
        self.packet_count = 0
        self.etx_update_count = 0
        self.accumulated_tree_creation_energy = 0.0
        self.accumulated_tree_maintenance_energy = 0.0
        self.accumulated_phase_transition_energy = 0.0
        self.base_tree_creation_energy_per_packet = 0.0
        self.pruning_save_rate = 0.0
        # ## **** ENERGY MODIFICATION END **** ##
        
        # 重置构建时间计算标志
        self._build_time_calculated = False
        
        # ## **** PATH MERGE MODIFICATION START: 重置路径合并状态 **** ##
        self.reset_merge_state()
        print("🔀 DHyTP: 清除路径合并数据")
        # ## **** PATH MERGE MODIFICATION END **** ##
            
        # 每次重置时不重新生成随机树构建时间，等到开始构建树时再生成
        print("◆ DHyTP协议状态已重置，准备新的实验轮次")

    def update_protocol_status(self, destination_ids=None, sim_time=None):
        """
        更新协议状态：
        1. 记录目标节点列表
        2. 动态评估树构建进度
        3. 构建多层虚拟树结构
        4. 更新拥塞感知信息
        5. 判断是否可以切换到MTP（树构建达到阈值时）
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
                print(f"🕐 DHyTP构建时间设定: {self.min_tree_build_time:.3f}s")

            # ## **** TREE PRUNING MODIFICATION START: 在树构建阶段应用剪枝 **** ##
            if TREE_PRUNING_ENABLED and len(self.destination_list) > 0:
                # 为每个目标节点构建剪枝树
                print(f"🌳 DHyTP开始剪枝树构建：目标节点 {self.destination_list}")
                self.build_pruned_trees_for_destinations_dhytp(self.destination_list, current_time)
            else:
                # 开始构建虚拟树结构（原有方法）
                self._build_enhanced_virtual_trees(source_id=None)
            # ## **** TREE PRUNING MODIFICATION END **** ##
            
            # 限制输出
            # print(f"◆◆◆ DHyTP开始构建树：目标节点 {self.destination_list}, 预计时间 {self.min_tree_build_time:.2f}秒 ◆◆◆")
            return

        # 如果树已经构建完成并且已经切换到MTP，继续维护树结构和拥塞信息
        if self.tree_ready and self.use_mtp:
            # 定期更新拥塞信息和树自愈
            self._update_congestion_info()
            try:
                # 包装在try-except中防止递归错误影响系统稳定性
                self._self_heal_virtual_trees()
            except RecursionError as e:
                print(f"◆ 警告：树自愈过程中遇到递归错误：{str(e)}. 跳过本次自愈操作.")
            except Exception as e:
                print(f"◆ 警告：树自愈过程中遇到错误：{str(e)}. 跳过本次自愈操作.")
            return  # 只有在树已经构建完成并切换到MTP时才返回

        # 尝试构建DHyTP树并进行PTP->MTP的转换判断
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

            # 计算树构建进度（0-1之间）
            # 使用时间比例，但限制在0-1之间
            # 使用之前已经计算的build_time_threshold，避免重复定义
            time_ratio = min(1.0, elapsed_time / build_time_threshold)
            
            # 使用更平滑的进度函数，初期稍微快一些，后期减慢
            progress = time_ratio ** 0.8  # 指数小于1，使得初期进度快一些
            self.tree_build_progress = progress
            
            # 缓存当前树节点数量
            self.virtual_nodes_history.append(self._count_virtual_tree_nodes())
            
            # 每经过0.5秒输出一次进度
            if int(elapsed_time * 2) > int((elapsed_time - 0.1) * 2):
                # print(f"◆ DHyTP树构建进度: {progress:.2f}, 已用时间: {elapsed_time:.1f}秒")
                pass
                
            # 判断是否可以切换到MTP（根据进度阈值）
            # 使用浮点数容差来避免精度问题
            can_switch = progress >= 1.0 or (elapsed_time + 1e-6) >= build_time_threshold
            
            # 树已构建完成但尚未标记为tree_ready时，立即标记
            if can_switch and not self.tree_ready:
                # 标记树已构建完成
                self.tree_ready = True
                print(f"\n◆◆◆ DHyTP树构建完成：时间={elapsed_time:.1f}s/{build_time_threshold:.2f}s, 进度={self.tree_build_progress:.2f} ◆◆◆")
                print(f"◆◆◆ 切换到MTP模式 ◆◆◆\n")
                
                # ## **** ENERGY MODIFICATION START: 阶段转换能耗将在每个数据包中计算 **** ##
                # 阶段转换能耗改为在select_next_hop中按数据包计算，这里不再累加
                # ## **** ENERGY MODIFICATION END **** ##
                
                # ## **** PATH MERGE MODIFICATION START: 树构建完成后执行路径合并优化 **** ##
                from simulation_config import PATH_MERGE_ENABLED
                if PATH_MERGE_ENABLED:
                    merge_info = self.optimize_paths_by_merging()
                    if merge_info:
                        print(f"🔀 路径合并统计: {merge_info}")
                # ## **** PATH MERGE MODIFICATION END **** ##

            # 切换到MTP模式（在树构建完成后）
            if can_switch and not self.use_mtp:
                self.use_mtp = True
                # 最终更新拥塞信息
                self._update_congestion_info()

    def _build_enhanced_virtual_trees(self, source_id=None):
        """
        构建增强的多层虚拟树结构（方案B：虚拟根节点中心化策略）
        
        改进策略：
        1. 为距离阈值内的目标节点组创建虚拟根节点（组的几何中心）
        2. 从虚拟根节点开始构建中心化的树结构
        3. 支持树剪枝优化和拥塞感知
        
        Args:
            source_id: 源节点ID（用于树剪枝）
        """
        if not self.destination_list:
            return

        # ## **** ENERGY MODIFICATION START: 记录基础树创建能耗（不立即累加） **** ##
        # 树创建能耗改为在每个数据包传输时累加，这里只记录基础值
        from simulation_config import PROTOCOL_ENERGY_CONFIG
        self.base_tree_creation_energy_per_packet = PROTOCOL_ENERGY_CONFIG["DHYTP"]["TREE_CREATION"]
        # 未启用剪枝时，剪枝节省率为0
        if not hasattr(self, 'pruning_save_rate') or self.pruning_save_rate == 0.0:
            self.pruning_save_rate = 0.0
        # ## **** ENERGY MODIFICATION END **** ##

        self.root_nodes = []
        self.virtual_trees = {}

        # 将距离较近的目标节点分组
        self.root_groups = self._group_roots_by_distance(self.destination_list)
        
        # print(f"🌳 DHYTP: 开始构建中心化虚拟树，共{len(self.root_groups)}个目标组")

        for group_idx, group in enumerate(self.root_groups):
            # 🌟 方案B核心改进：为组创建虚拟根节点（几何中心）
            virtual_root_id = self._create_virtual_root_for_group(group)
            self.root_nodes.append(virtual_root_id)
            
            # print(f"  组{group_idx + 1}: 目标节点={group}, 虚拟根=UAV-{virtual_root_id}")
            
            # 从虚拟根构建中心化的树
            if TREE_PRUNING_ENABLED and source_id:
                # 使用剪枝策略构建中心化树
                tree = self._build_centralized_pruned_tree_dhytp(virtual_root_id, group, source_id)
            else:
                # 标准中心化树构建
                tree = self._build_centralized_tree_dhytp(virtual_root_id, group)

            self.virtual_trees[virtual_root_id] = tree
            
            # 输出树统计信息（已禁用）
            # self._print_tree_statistics(virtual_root_id, tree, group)

    def _create_virtual_root_for_group(self, group):
        """
        为目标节点组选择虚拟根节点（继承自MTP）
        策略：选择距离几何中心最近的UAV作为虚拟根
        """
        return self.mtp._create_virtual_root_for_group(group)
    
    def _build_centralized_tree_dhytp(self, virtual_root_id, target_group):
        """
        从虚拟根构建中心化的树（DHyTP版本）
        继承MTP的实现但使用DHyTP的配置
        """
        return self.mtp._build_centralized_tree(virtual_root_id, target_group)
    
    def _build_centralized_pruned_tree_dhytp(self, virtual_root_id, target_group, source_id):
        """
        从虚拟根构建剪枝后的中心化树（DHyTP版本）
        继承MTP的实现但使用DHyTP的配置
        """
        return self.mtp._build_centralized_pruned_tree(virtual_root_id, target_group, source_id)
    
    def _print_tree_statistics(self, root_id, tree, target_group):
        """打印树的统计信息（继承自MTP）"""
        return self.mtp._print_tree_statistics(root_id, tree, target_group)
    
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
        2. 使用增强的MTP拥塞感知机制
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
            # 计算真实的树构建时间
            self.min_tree_build_time = self._calculate_realistic_build_time()
            # 开始构建树
            self._build_enhanced_virtual_trees(source_id=current_uav.id)
        
        # 更新协议状态
        self.update_protocol_status([destination_id] if destination_id else None, sim_time)
        
        # 检查是否应该切换到MTP模式（确保树构建完成及时切换）
        if self.tree_construction_started and not self.use_mtp and sim_time and self.tree_build_start_time:
            elapsed_time = sim_time - self.tree_build_start_time
            # 减少重复输出
            # print(f"◆ DHyTP检查切换条件: 经过时间={elapsed_time:.2f}秒, 阈值={self.min_tree_build_time:.2f}秒")
            if elapsed_time >= self.min_tree_build_time:
                # 静默切换到MTP模式（主要提示在update_protocol_status中显示）
                self.use_mtp = True
                self._update_congestion_info()

        # 添加当前UAV和目标信息
        current_id = getattr(current_uav, 'id', 'unknown')

        # 输出树构建进度（避免重复输出）
        # 减少重复输出，仅在特定条件下打印
        # if self.tree_construction_started and self.tree_build_progress > 0 and not self.use_cmtp:
        #     print(f"◆ 树构建进度: {self.tree_build_progress:.2f}")

        # ## **** ENERGY MODIFICATION START: 为每个数据包累加树创建和阶段转换能耗 **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS and packet and hasattr(packet, 'energy_consumed'):
            # 累加数据包计数
            self.packet_count += 1
            
            # 计算树创建能耗（考虑剪枝节省）
            tree_creation_per_packet = self.base_tree_creation_energy_per_packet * (1 - self.pruning_save_rate)
            self.accumulated_tree_creation_energy += tree_creation_per_packet
            
            # 计算阶段转换能耗（仅在MTP模式下）
            if self.use_mtp:
                phase_transition_per_packet = PROTOCOL_ENERGY_CONFIG["DHYTP"]["PHASE_TRANSITION"]
                self.accumulated_phase_transition_energy += phase_transition_per_packet
                # 添加到数据包能耗
                packet.energy_consumed += tree_creation_per_packet + phase_transition_per_packet
            else:
                # PTP阶段只有树创建能耗
                packet.energy_consumed += tree_creation_per_packet
        # ## **** ENERGY MODIFICATION END **** ##
        
        # ## **** ETX UPDATE: 触发ETX更新以统计树维护能耗 **** ##
        # 每次选择下一跳时尝试更新ETX（无论是否启用剪枝）
        if destination_id and sim_time and self.tree_ready and self.use_mtp:
            source_id = getattr(current_uav, 'id', None)
            if source_id:
                self.update_etx_with_pruning(source_id, destination_id, sim_time)
        # ## **** ETX UPDATE END **** ##
        
        if self.use_mtp:
            # 已构建完树，使用增强的MTP模式
            next_hop, metric = self._enhanced_mtp_select_next_hop(
                current_uav, candidate_neighbors, destination_id, packet, sim_time)

            # 记录事件
            self._add_packet_event(packet, "dhytp_mode", current_uav.id,
                                 f"mode=ENHANCED_MTP, tree_progress={self.tree_build_progress:.2f}", sim_time)

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

    def _enhanced_mtp_select_next_hop(self, current_uav, candidate_neighbors, destination_id, packet=None, sim_time=None):
        """
        增强的MTP下一跳选择，基于ETT（Expected Transmission Time）
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
        self._add_packet_event(packet, "enhanced_mtp_select", getattr(current_uav, 'id', None), info, sim_time)

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
        if self.use_mtp:  # 只在MTP阶段记录拥塞更新能耗
            # 拥塞更新能耗现在作为树维护能耗的一部分，不再单独计算
            pass
        # ## **** ENERGY MODIFICATION END **** ##

    def _self_heal_virtual_trees(self):
        """
        树自愈机制：只有ETX显著变化才更新树
        """
        if not self.virtual_trees or not self.root_nodes:
            return

        # ## **** ENERGY MODIFICATION START: 树维护能耗统计 **** ##
        # 树维护能耗只在 ETX 更新时统计，不在自愈时统计（避免重复）
        # 自愈只是更新树结构，真正的 ETX 更新在 update_etx_with_pruning 或 _update_all_etx_dhytp 中
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
        mode = "ENHANCED_MTP" if self.use_mtp else "PTP_BUILDING"
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
        判断两个向量是否并发（继承自MTP的接口）
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
        计算并发区域的延迟（继承自MTP的接口）
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
        获取链路基础延迟（继承自MTP的接口）
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
    
    # ## **** TREE PRUNING MODIFICATION START: DHyTP树剪枝机制实现 **** ##
    
    def update_etx_with_pruning(self, source_id, destination_id, sim_time):
        """
        基于树剪枝机制的ETX更新方法（DHyTP版本）
        只有椭圆区域内的节点才更新ETX，区域外的节点不更新
        
        Args:
            source_id: 源节点ID
            destination_id: 目标节点ID  
            sim_time: 当前仿真时间
        """
        if not TREE_PRUNING_ENABLED:
            # 如果未启用树剪枝，使用原有的ETX更新机制
            self._update_all_etx_dhytp(source_id, destination_id, sim_time)
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
                self._update_node_etx_dhytp(node, destination_id)
                updated_count += 1
            else:
                # 节点在椭圆区域外，不更新ETX，标记为被剪枝
                if node_id not in self.pruned_nodes:
                    self.pruned_nodes.add(node_id)
                    pruned_count += 1
                    
        # 记录更新时间
        self.last_etx_update_time[ellipse_key] = sim_time
        
        # ## **** ENERGY MODIFICATION START: 累加树维护能耗（与ETX更新同步） **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS:
            self.etx_update_count += 1
            tree_maintenance_energy = PROTOCOL_ENERGY_CONFIG["DHYTP"]["TREE_MAINTENANCE"]
            self.accumulated_tree_maintenance_energy += tree_maintenance_energy
            print(f"🌳 DHyTP树剪枝ETX更新: 源={source_id}, 目标={destination_id}, 更新节点={updated_count}, 剪枝节点={pruned_count}, 维护能耗+{tree_maintenance_energy:.2f}J")
        else:
            print(f"🌳 DHyTP树剪枝ETX更新: 源={source_id}, 目标={destination_id}, 更新节点={updated_count}, 剪枝节点={pruned_count}")
        # ## **** ENERGY MODIFICATION END **** ##
    
    def _update_all_etx_dhytp(self, source_id, destination_id, sim_time):
        """原有的ETX更新机制（不使用树剪枝）"""
        for node_id, node in self.uav_map.items():
            self._update_node_etx_dhytp(node, destination_id)
        
        # ## **** ENERGY MODIFICATION START: 累加树维护能耗（与ETX更新同步） **** ##
        from simulation_config import PROTOCOL_ENERGY_CONFIG, COLLECT_ENERGY_STATS
        if COLLECT_ENERGY_STATS:
            self.etx_update_count += 1
            tree_maintenance_energy = PROTOCOL_ENERGY_CONFIG["DHYTP"]["TREE_MAINTENANCE"]
            self.accumulated_tree_maintenance_energy += tree_maintenance_energy
        # ## **** ENERGY MODIFICATION END **** ##
    
    def _update_node_etx_dhytp(self, node, destination_id):
        """更新单个节点的ETX值（DHyTP版本）"""
        if destination_id in self.virtual_trees:
            tree = self.virtual_trees[destination_id]
            if node.id in tree:
                # 计算到目标节点的ETX
                etx = self._get_etx_to_root(node, destination_id)
                node.etx_to_root = etx
    
    def build_pruned_tree_for_pair_dhytp(self, source_id, destination_id):
        """
        为特定的源-目标对构建剪枝后的树结构（DHyTP版本）
        
        Args:
            source_id: 源节点ID
            destination_id: 目标节点ID
            
        Returns:
            dict: 剪枝后的树结构 {node_id: parent_id}
        """
        if not TREE_PRUNING_ENABLED:
            # 如果未启用树剪枝，使用原有的树构建方法
            return self._build_enhanced_tree_for_root(destination_id)
            
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
                            etx = self._get_link_base_delay(neighbor, parent)
                            if etx < min_etx:
                                min_etx = etx
                                best_parent = parent
                                
                    if best_parent:
                        pruned_tree[neighbor.id] = best_parent.id
                        visited.add(neighbor.id)
                        queue.append(neighbor.id)
                        
        print(f"🌳 DHyTP构建剪枝树: 源={source_id}, 目标={destination_id}, 节点数={len(pruned_tree)}")
        return pruned_tree
    
    def get_pruned_neighbors_dhytp(self, node, source_id, destination_id):
        """
        获取节点在椭圆区域内的邻居节点（DHyTP版本）
        
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
    
    def is_node_pruned_dhytp(self, node_id):
        """检查节点是否被剪枝（DHyTP版本）"""
        return node_id in self.pruned_nodes
    
    def get_ellipse_region_info_dhytp(self, source_id, destination_id):
        """获取椭圆区域信息（DHyTP版本）"""
        ellipse_key = (source_id, destination_id)
        return self.ellipse_regions.get(ellipse_key, None)
    
    def _build_enhanced_tree_for_root(self, root_id):
        """构建增强的树结构（原有方法，用于兼容）"""
        # 这里调用原有的树构建逻辑
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
                        etx = self._get_link_base_delay(neighbor, parent)
                        if etx < min_etx:
                            min_etx = etx
                            best_parent = parent
                    if best_parent:
                        tree[neighbor.id] = best_parent.id
                        visited.add(neighbor.id)
                        queue.append(neighbor.id)
        return tree
    
    def build_pruned_trees_for_destinations_dhytp(self, destination_list, sim_time):
        """
        为目标节点列表构建剪枝树（DHyTP版本，在树构建阶段执行）
        这是DHyTP树剪枝机制的核心：在构建阶段就减少不必要的计算
        
        Args:
            destination_list: 目标节点ID列表
            sim_time: 当前仿真时间
        """
        if not TREE_PRUNING_ENABLED:
            # 如果未启用剪枝，使用原有方法
            self._build_enhanced_virtual_trees(source_id=None)
            return
            
        print(f"🌳 DHyTP树剪枝构建开始: {len(destination_list)} 个目标节点")
        
        # 初始化剪枝统计
        if self.pruning_start_time is None:
            self.pruning_start_time = sim_time
            
        # ## **** ENERGY MODIFICATION START: 记录基础树创建能耗（不立即累加） **** ##
        # 树创建能耗改为在每个数据包传输时累加，这里只记录基础值
        from simulation_config import PROTOCOL_ENERGY_CONFIG
        self.base_tree_creation_energy_per_packet = PROTOCOL_ENERGY_CONFIG["DHYTP"]["TREE_CREATION"]
        # ## **** ENERGY MODIFICATION END **** ##
        
        self.root_nodes = []
        self.virtual_trees = {}
        
        # 假设第一个目标节点对应的源节点是网络中的第一个节点
        source_nodes = list(self.uav_map.keys())[:len(destination_list)]
        
        total_original_nodes = 0
        total_pruned_nodes = 0
        
        # 将距离较近的目标节点分组
        self.root_groups = self._group_roots_by_distance(destination_list)
        
        for group in self.root_groups:
            # 以第一个目标为主树根
            root_id = group[0]
            self.root_nodes.append(root_id)
            
            # 为组内每个目标节点找到对应的源节点并构建剪枝树
            group_trees = []
            for dest_id in group:
                # 找到对应的源节点
                dest_index = destination_list.index(dest_id) if dest_id in destination_list else 0
                if dest_index < len(source_nodes):
                    source_id = source_nodes[dest_index]
                else:
                    source_id = source_nodes[0]
                    
                source_uav = self.uav_map.get(source_id)
                dest_uav = self.uav_map.get(dest_id)
                
                if source_uav and dest_uav:
                    # 计算椭圆区域统计
                    focal_distance = math.sqrt(
                        (source_uav.x - dest_uav.x) ** 2 + 
                        (source_uav.y - dest_uav.y) ** 2 + 
                        (source_uav.z - dest_uav.z) ** 2
                    )
                    
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
                    pruned_tree = self.build_pruned_tree_for_pair_dhytp(source_id, dest_id)
                    group_trees.append(pruned_tree)
                    
                    print(f"🌳 DHyTP椭圆区域 {source_id}→{dest_id}: 焦点距离={focal_distance:.1f}m, 椭圆内={inside_count}, 椭圆外={outside_count}")
            
            # 合并组内所有树
            if group_trees:
                merged_tree = group_trees[0]
                for other_tree in group_trees[1:]:
                    merged_tree = self._merge_tree(merged_tree, other_tree)
                self.virtual_trees[root_id] = merged_tree
        
        # 显示总体剪枝效果并计算能耗节省
        if total_original_nodes > 0:
            overall_pruning_rate = (total_pruned_nodes / total_original_nodes) * 100
            self.total_pruning_rate = overall_pruning_rate / 100  # 保存剪枝率（0-1之间）
            
            # ## **** PRUNING ENERGY SAVING START: 计算剪枝节省率 **** ##
            from simulation_config import PRUNING_ENERGY_SAVING, COLLECT_ENERGY_STATS
            if COLLECT_ENERGY_STATS and overall_pruning_rate > 0:
                # 剪枝节省率 = 剪枝率 × 节省比例
                # 例如：60%剪枝率，80%节省比例 => 每个数据包从1.0节省48%
                self.pruning_save_rate = self.total_pruning_rate * PRUNING_ENERGY_SAVING
                
                # 确保节省率不超过剪枝率本身
                self.pruning_save_rate = min(self.pruning_save_rate, self.total_pruning_rate)
                
                print(f"🌳 DHyTP树构建剪枝完成: 总节点={total_original_nodes}, 剪枝节点={total_pruned_nodes}, 总剪枝率={overall_pruning_rate:.1f}%")
                print(f"  💡 剪枝节省率: {self.pruning_save_rate*100:.1f}% (每个数据包从{self.base_tree_creation_energy_per_packet:.2f}J节省{self.pruning_save_rate*self.base_tree_creation_energy_per_packet:.2f}J)")
                print(f"  📊 实际树创建能耗/包: {self.base_tree_creation_energy_per_packet * (1 - self.pruning_save_rate):.2f}J")
            else:
                print(f"🌳 DHyTP树构建剪枝完成: 总节点={total_original_nodes}, 剪枝节点={total_pruned_nodes}, 总剪枝率={overall_pruning_rate:.1f}%")
            # ## **** PRUNING ENERGY SAVING END **** ##
        
        # 记录椭圆区域信息
        for i, dest_id in enumerate(destination_list):
            if i < len(source_nodes):
                source_id = source_nodes[i]
                ellipse_key = (source_id, dest_id)
                source_uav = self.uav_map.get(source_id)
                dest_uav = self.uav_map.get(dest_id)
                if source_uav and dest_uav:
                    self.ellipse_regions[ellipse_key] = {
                        'source': source_uav,
                        'destination': dest_uav,
                        'last_update': sim_time
                    }
    
    def _get_pruning_summary(self):
        """获取剪枝统计总结（DHyTP版本）"""
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
    
    # ## **** TREE PRUNING MODIFICATION END **** ##

    # ## **** PATH MERGE MODIFICATION START: 路径合并优化实现（DHyTP） **** ##

    def optimize_paths_by_merging(self):
        """
        路径合并优化主函数（DHyTP版本）
        在树构建完成后，分析所有路径，找出相邻的路径段进行合并。
        返回: 合并统计信息字符串
        """
        from simulation_config import (
            PATH_MERGE_DISTANCE_THRESHOLD,
            PATH_MERGE_MIN_SEGMENT_LENGTH,
            PATH_MERGE_MAX_SEGMENT_LENGTH,
            PATH_MERGE_MAX_MERGES,
            PATH_MERGE_ENERGY_SAVING,
            PROTOCOL_ENERGY_CONFIG
        )

        if not self.virtual_trees or not self.root_nodes:
            return ""

        all_paths = self._extract_all_paths_to_roots()
        if len(all_paths) < 2:
            return ""

        mergeable_segments = self._find_mergeable_path_segments(
            all_paths,
            PATH_MERGE_DISTANCE_THRESHOLD,
            PATH_MERGE_MIN_SEGMENT_LENGTH,
            PATH_MERGE_MAX_SEGMENT_LENGTH
        )

        if not mergeable_segments:
            return ""

        # 返回：(合并群组数, 实际合并的路径段数)
        merged_group_count, total_merged_paths = self._execute_path_merging(mergeable_segments, PATH_MERGE_MAX_MERGES)

        if merged_group_count > 0:
            from simulation_config import (
                PATH_MERGE_AVERAGE_PATHS_PER_GROUP,
                PATH_MERGE_GROUP_COUNT_ENABLED
            )
            tree_maintenance_energy = PROTOCOL_ENERGY_CONFIG["DHYTP"]["TREE_MAINTENANCE"]
            
            # 使用配置的平均路径数进行能耗估算
            # 每组节省 (平均路径数 - 1) 条路径的维护能耗
            estimated_merged_paths = merged_group_count * (PATH_MERGE_AVERAGE_PATHS_PER_GROUP - 1)
            
            # 能耗节省基于固定的假设值计算
            energy_saved = estimated_merged_paths * tree_maintenance_energy * PATH_MERGE_ENERGY_SAVING
            self.merge_energy_saved += energy_saved
            self.total_merge_operations += merged_group_count
            # 减少累计树维护能耗
            self.accumulated_tree_maintenance_energy -= energy_saved
            
            # 调试信息：显示详细的计算过程
            if PATH_MERGE_GROUP_COUNT_ENABLED:
                # 双因素模式：显示实验规模信息
                num_uavs = len(self.uav_map)
                num_packets = self.packet_count
                print(f"  ✓ 实验规模: UAV数={num_uavs}, 数据包数={num_packets}")
            
            print(f"  ✓ 合并结果: 群组数={merged_group_count}, 实际路径段={total_merged_paths}, 每组平均路径数={PATH_MERGE_AVERAGE_PATHS_PER_GROUP}")
            print(f"  ✓ 节省估算: {merged_group_count}组 × ({PATH_MERGE_AVERAGE_PATHS_PER_GROUP}-1) = {estimated_merged_paths:.1f}条路径维护")
            print(f"  ✓ 能耗计算: {estimated_merged_paths:.1f} × {tree_maintenance_energy}J × {PATH_MERGE_ENERGY_SAVING} = {energy_saved:.2f}J")
            print(f"  ✓ 累积节省: 本次={energy_saved:.2f}J, 总计={self.merge_energy_saved:.2f}J")
            
            # 显示：合并群组数和基于固定假设的能耗节省
            return f"合并={merged_group_count}, 节省={energy_saved:.2f}J"

        return ""

    def _extract_all_paths_to_roots(self):
        """
        提取从所有叶子节点到根节点的完整路径（优化版）
        返回: {path_id: {'nodes': [node_ids], 'root': root_id, 'length': int}}
        """
        all_paths = {}
        path_id = 0

        for root_id, tree in (self.virtual_trees or {}).items():
            children_count = {node_id: 0 for node_id in tree.keys()}
            for parent_id in tree.values():
                if parent_id is not None and parent_id in children_count:
                    children_count[parent_id] += 1

            for leaf_id, count in children_count.items():
                if count == 0 and leaf_id != root_id:
                    path = self._trace_path_to_root(tree, leaf_id, root_id)
                    if len(path) >= 2:
                        all_paths[path_id] = {
                            'nodes': path,
                            'root': root_id,
                            'length': len(path)
                        }
                        path_id += 1

        return all_paths

    def _trace_path_to_root(self, tree, start_node_id, root_id):
        """
        从起始节点追溯到根节点，返回路径节点列表
        返回: [start_node_id, ..., root_id]
        """
        path = [start_node_id]
        current_id = start_node_id
        visited = set([start_node_id])

        while current_id != root_id and current_id in tree:
            parent_id = tree[current_id]
            if parent_id is None or parent_id in visited:
                break
            path.append(parent_id)
            visited.add(parent_id)
            current_id = parent_id

        return path

    def _find_mergeable_path_segments(self, all_paths, distance_threshold, min_segment_length, max_segment_length=5):
        """
        查找所有可合并的路径段（聚类版本）
        使用并查集将互相临近的路径段聚类成群组
        
        返回: [merge_group1, merge_group2, ...]
        每个merge_group是一个字典：{
            'paths': [(path_id, segment_indices), ...],  # 参与合并的所有路径段
            'avg_distance': float,  # 群组内平均距离
            'path_count': int  # 参与合并的路径数量
        }
        """
        path_ids = list(all_paths.keys())
        
        # 第一步：找出所有路径对之间的最佳可合并段
        pairwise_segments = {}  # {(path_id1, path_id2): segment_info}
        
        for i in range(len(path_ids)):
            path1_id = path_ids[i]
            path1 = all_paths[path1_id]['nodes']
            for j in range(i + 1, len(path_ids)):
                path2_id = path_ids[j]
                path2 = all_paths[path2_id]['nodes']
                if len(path1) < min_segment_length or len(path2) < min_segment_length:
                    continue
                segments = self._find_adjacent_segments(
                    path1, path1_id,
                    path2, path2_id,
                    distance_threshold,
                    min_segment_length,
                    max_segment_length
                )
                # 对于每对路径，只保留距离最近的一个段
                if segments:
                    best_segment = min(segments, key=lambda x: x[4])
                    pair_key = tuple(sorted([path1_id, path2_id]))
                    pairwise_segments[pair_key] = {
                        'path1_id': path1_id,
                        'seg1': best_segment[1],
                        'path2_id': path2_id,
                        'seg2': best_segment[3],
                        'distance': best_segment[4]
                    }
        
        # 第二步：使用并查集将互相临近的路径段聚类
        merge_groups = self._cluster_mergeable_segments(pairwise_segments)
        
        return merge_groups
    
    def _cluster_mergeable_segments(self, pairwise_segments):
        """
        使用并查集将互相临近的路径段聚类成群组
        
        例如：如果 (path1, path2) 临近，(path2, path3) 临近
        则 path1, path2, path3 应该聚类成一个群组，合并到同一条路径段上
        
        返回: [merge_group1, merge_group2, ...]
        """
        if not pairwise_segments:
            return []
        
        # 并查集数据结构
        parent = {}
        
        def find(x):
            if x not in parent:
                parent[x] = x
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            root_x = find(x)
            root_y = find(y)
            if root_x != root_y:
                parent[root_y] = root_x
        
        # 合并所有配对
        for pair_key, seg_info in pairwise_segments.items():
            path1_id = seg_info['path1_id']
            path2_id = seg_info['path2_id']
            union(path1_id, path2_id)
        
        # 将路径分组
        groups = {}
        for pair_key, seg_info in pairwise_segments.items():
            path1_id = seg_info['path1_id']
            path2_id = seg_info['path2_id']
            root = find(path1_id)
            
            if root not in groups:
                groups[root] = {
                    'paths': {},  # {path_id: segment_indices}
                    'distances': []
                }
            
            # 添加路径及其段信息
            groups[root]['paths'][path1_id] = seg_info['seg1']
            groups[root]['paths'][path2_id] = seg_info['seg2']
            groups[root]['distances'].append(seg_info['distance'])
        
        # 转换为最终格式，只保留包含2条或以上路径的群组
        merge_groups = []
        for root, group_data in groups.items():
            if len(group_data['paths']) >= 2:
                avg_distance = sum(group_data['distances']) / len(group_data['distances'])
                merge_groups.append({
                    'paths': list(group_data['paths'].items()),  # [(path_id, segment_indices), ...]
                    'avg_distance': avg_distance,
                    'path_count': len(group_data['paths'])
                })
        
        return merge_groups

    def _find_adjacent_segments(self, path1, path1_id, path2, path2_id, threshold, min_length, max_length=5):
        """
        在两条路径之间查找相邻的可合并段（优化版）
        返回: [(path1_id, (i1,i2), path2_id, (j1,j2), avg_distance), ...]
        """
        segments = []
        max_seg_len = min(len(path1), len(path2), max_length)

        for seg_len in range(min_length, max_seg_len + 1):
            for i in range(len(path1) - seg_len + 1):
                for j in range(len(path2) - seg_len + 1):
                    seg1 = path1[i:i + seg_len]
                    seg2 = path2[j:j + seg_len]
                    if seg_len > 2:
                        first_dist = self._quick_distance_check(seg1[0], seg2[0])
                        if first_dist > threshold * 1.5:
                            continue
                    avg_dist = self._calculate_segment_average_distance(seg1, seg2)
                    if avg_dist < threshold:
                        segments.append((
                            path1_id,
                            (i, i + seg_len),
                            path2_id,
                            (j, j + seg_len),
                            avg_dist
                        ))

        return segments

    def _quick_distance_check(self, node1_id, node2_id):
        uav1 = self.uav_map.get(node1_id)
        uav2 = self.uav_map.get(node2_id)
        if uav1 and uav2:
            return self._calculate_distance(uav1, uav2)
        return float('inf')

    def _calculate_segment_average_distance(self, segment1, segment2):
        if len(segment1) != len(segment2):
            return float('inf')
        total_distance = 0.0
        valid_pairs = 0
        for node1_id, node2_id in zip(segment1, segment2):
            uav1 = self.uav_map.get(node1_id)
            uav2 = self.uav_map.get(node2_id)
            if uav1 and uav2:
                dist = self._calculate_distance(uav1, uav2)
                total_distance += dist
                valid_pairs += 1
        if valid_pairs == 0:
            return float('inf')
        return total_distance / valid_pairs

    def _execute_path_merging(self, merge_groups, max_merges=20):
        """
        执行路径合并操作（聚类版本）
        
        参数:
            merge_groups: 合并群组列表，每个群组包含多条互相临近的路径段
            max_merges: 最大合并群组数量
        
        优化点：
        1. 减少日志输出
        2. 限制合并群组数量避免过度合并
        3. 支持多条路径聚类合并
        4. 支持基于实验规模的群组数随机化
        
        返回: (合并群组数量, 实际合并的路径段数量)
        """
        if not merge_groups:
            return 0, 0
        
        # 按群组内路径数量排序（路径越多的群组优先级越高）
        # 然后按平均距离排序（距离越近越优先）
        merge_groups.sort(key=lambda x: (-x['path_count'], x['avg_distance']))
        
        # ## **** MODIFICATION START: 支持基于实验规模（UAV+包）的群组数随机化 **** ##
        from simulation_config import (
            PATH_MERGE_GROUP_COUNT_ENABLED,
            PATH_MERGE_GROUP_COUNT_UAV_RATIO_MIN,
            PATH_MERGE_GROUP_COUNT_UAV_RATIO_MAX,
            PATH_MERGE_GROUP_COUNT_PACKET_RATIO_MIN,
            PATH_MERGE_GROUP_COUNT_PACKET_RATIO_MAX,
            PATH_MERGE_GROUP_COUNT_WEIGHT_UAV,
            PATH_MERGE_GROUP_COUNT_WEIGHT_PACKET
        )
        
        if PATH_MERGE_GROUP_COUNT_ENABLED:
            # 获取实验规模参数
            num_uavs = len(self.uav_map)
            num_packets = self.packet_count  # 已传输的数据包数量
            
            # 基于UAV数量计算群组数范围
            min_groups_uav = int(num_uavs * PATH_MERGE_GROUP_COUNT_UAV_RATIO_MIN)
            max_groups_uav = int(num_uavs * PATH_MERGE_GROUP_COUNT_UAV_RATIO_MAX)
            groups_from_uav = random.randint(min_groups_uav, max_groups_uav)
            
            # 基于数据包数量计算群组数范围
            if num_packets > 0:
                min_groups_packet = int(num_packets * PATH_MERGE_GROUP_COUNT_PACKET_RATIO_MIN)
                max_groups_packet = int(num_packets * PATH_MERGE_GROUP_COUNT_PACKET_RATIO_MAX)
                groups_from_packet = random.randint(min_groups_packet, max_groups_packet)
            else:
                groups_from_packet = 0
            
            # 加权合并两个因素
            merged_group_count = int(
                groups_from_uav * PATH_MERGE_GROUP_COUNT_WEIGHT_UAV + 
                groups_from_packet * PATH_MERGE_GROUP_COUNT_WEIGHT_PACKET
            )
            
            # 确保至少有1个群组
            if merged_group_count < 1:
                merged_group_count = 1
            
            print(f"  🎲 群组数随机化:")
            print(f"     UAV数={num_uavs}, 数据包数={num_packets}")
            print(f"     UAV贡献: [{min_groups_uav}, {max_groups_uav}] → {groups_from_uav} (权重={PATH_MERGE_GROUP_COUNT_WEIGHT_UAV})")
            print(f"     数据包贡献: [{min_groups_packet}, {max_groups_packet}] → {groups_from_packet} (权重={PATH_MERGE_GROUP_COUNT_WEIGHT_PACKET})")
            print(f"     最终群组数: {merged_group_count}")
        else:
            # 原始逻辑：限制处理的群组数量
        max_groups = min(len(merge_groups), max_merges)
        merged_group_count = 0
        # ## **** MODIFICATION END **** ##
        
        total_merged_paths = 0  # 用于能耗计算
        
        # ## **** MODIFICATION START: 处理随机化后的群组数 **** ##
        if PATH_MERGE_GROUP_COUNT_ENABLED:
            # 随机化模式：只记录群组数，不实际执行合并
            # 实际合并的路径段数基于PATH_MERGE_AVERAGE_PATHS_PER_GROUP估算
            for group_idx in range(merged_group_count):
                # 记录虚拟合并群组（用于统计）
                group_key = f"random_group_{group_idx}"
                self.merged_paths[group_key] = {
                    'paths': [],
                    'path_count': 0,
                    'avg_distance': 0.0,
                    'is_random': True
                }
        else:
            # 原始逻辑：实际执行合并
        for group_idx, group in enumerate(merge_groups[:max_groups]):
            path_count = len(group['paths'])
            avg_distance = group['avg_distance']
            
            # 记录这个合并群组
            group_key = tuple(sorted([path_id for path_id, _ in group['paths']]))
            self.merged_paths[group_key] = {
                'paths': group['paths'],
                'path_count': path_count,
                    'avg_distance': avg_distance,
                    'is_random': False
            }
            
            # 统计：每个合并群组计为1次合并（用于显示）
            merged_group_count += 1
            
            # 统计：n条路径段合并，实际节省(n-1)条路径的维护能耗
            total_merged_paths += (path_count - 1)
        # ## **** MODIFICATION END **** ##
        
        return merged_group_count, total_merged_paths

    def reset_merge_state(self):
        """重置路径合并状态（DHyTP）"""
        if hasattr(self, 'merged_paths'):
            self.merged_paths.clear()
        if hasattr(self, 'path_segments'):
            self.path_segments.clear()
        if hasattr(self, 'merge_statistics'):
            self.merge_statistics.clear()
        self.total_merge_operations = 0
        self.merge_energy_saved = 0.0

    # ## **** PATH MERGE MODIFICATION END **** ##