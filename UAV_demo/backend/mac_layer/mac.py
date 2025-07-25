# 文件: backend/mac_layer/mac.py
# 描述: 重构MAC层以实现基于队列的冲突解决机制

from collections import deque
from models.communication_model import CommunicationModel
from simulation_config import (
    MAX_RETRANSMISSIONS, 
    POSITION_CHANGE_THRESHOLD, 
    USE_DHYTP_ROUTING_MODEL, 
    USE_CTMP_ROUTING_MODEL, 
    USE_PTP_ROUTING_MODEL,
    UAV_COMMUNICATION_RANGE,
    ENERGY_UNIT_SEND, ENERGY_RETRANSMISSION_PENALTY, COLLECT_ENERGY_STATS, ENERGY_UNIT_RECEIVE
)
import math
from protocols.ptp_protocol import PTPRoutingModel
from protocols.cmtp_protocol import CMTPRoutingModel
from protocols.dhytp_protocol import DHyTPRoutingModel

class MACLayer:
    def __init__(self, all_uavs, sim_manager):
        self.all_uavs = all_uavs
        self.uav_map = {uav.id: uav for uav in all_uavs}
        self.comm_model = CommunicationModel()
        # 根据配置初始化路由协议
        if USE_DHYTP_ROUTING_MODEL:
            self.routing_model = DHyTPRoutingModel(self.uav_map)
        elif USE_CTMP_ROUTING_MODEL:
            self.routing_model = CMTPRoutingModel(self.uav_map)
        elif USE_PTP_ROUTING_MODEL:
            self.routing_model = PTPRoutingModel(self.uav_map)
        else:
            self.routing_model = None

        # ## **** MODIFICATION START: 保存对sim_manager的引用 **** ##
        self.sim_manager = sim_manager 
        # ## **** MODIFICATION END **** ##

        # ## **** ENERGY MODIFICATION START: 添加能耗统计相关变量 **** ##
        self.total_energy_consumed = 0.0  # 所有数据包消耗的总能量
        # ## **** ENERGY MODIFICATION END **** ##

        self.total_hop_attempts = 0
        self.sim_time = 0
        self.packet_status_snapshot = []
        # 多对一冲突队列，键为接收者ID，值为发送者ID队列
        self.collision_queues = {}
        # 距离干扰队列，键为受干扰接收者ID，值为发送者ID队列
        self.distance_interference_queues = {}

    def reset_counters(self):
        self.total_hop_attempts = 0
        self.packet_status_snapshot.clear()
        self.collision_queues.clear() # 重置多对一冲突队列
        self.distance_interference_queues.clear() # 重置距离干扰队列

        # ## **** ENERGY MODIFICATION START: 重置能耗计数 **** ##
        self.total_energy_consumed = 0.0
        # ## **** ENERGY MODIFICATION END **** ##

        # 重置DHyTP协议状态
        if hasattr(self, 'routing_model') and self.routing_model is not None:
            if hasattr(self.routing_model, 'reset_protocol_state'):
                self.routing_model.reset_protocol_state()

    def update_uav_list(self, all_uavs):
        self.all_uavs = all_uavs
        self.uav_map = {uav.id: uav for uav in all_uavs}
        if hasattr(self, 'routing_model') and self.routing_model is not None:
            if isinstance(self.routing_model, DHyTPRoutingModel):
                self.routing_model.uav_map = self.uav_map
                self.routing_model.cmtp.uav_map = self.uav_map
                self.routing_model.ptp.uav_map = self.uav_map
            elif isinstance(self.routing_model, CMTPRoutingModel):
                self.routing_model.uav_map = self.uav_map
            else:
                self.routing_model.uav_map = self.uav_map

    # ## **** REFACTORED: 实现基于队列的冲突解决 **** ##
    def process_transmissions(self, sim_time):
        # 简化时间片输出，使用更明显的分隔符
        print(f"\n--- 时间片 {float(sim_time):.1f} ---")
        # self._log(f"--- 时间片 {float(sim_time)} ---")  # 写入日志
        self.sim_time = sim_time
        self.packet_status_snapshot.clear()
        import simulation_config
        self.use_ptp = getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False)
        
        # 添加一个集合，用于跟踪已经打印过等待日志的数据包ID
        self.logged_waiting_packets = set()
        
        # 首先更新CMTP协议状态，确保树构建状态在处理数据包前是最新的
        if isinstance(self.routing_model, CMTPRoutingModel) and self.routing_model.tree_construction_started:
            self.routing_model.update_protocol_status(None, sim_time)

        # 1. 识别所有有数据要发的无人机
        potential_senders = {uav for uav in self.all_uavs if uav.tx_queue}
        if not potential_senders:
            return

        # 删除之前添加的批量路由信息输出，改为分散显示

        # 所有包的当前跳等待+1，只增加一次
        for uav in potential_senders:
            packet = uav.tx_queue[0]
            if not hasattr(packet, 'concurrent_delay'):
                packet.concurrent_delay = 0  # 初始化并发延时
            if hasattr(packet, 'per_hop_waits') and packet.per_hop_waits:
                # 每个时间片只增加一次等待计数
                packet.per_hop_waits[-1] += 1
                
                # 只记录事件，不再增加等待计数(已经+1了)
                if hasattr(packet, 'add_event'):
                    # 只增加事件记录，但不使用waiting类型避免事件历史过于冗长
                    pass

        # 检查CMTP协议树构建状态，过滤掉树构建阶段的数据包
        filtered_senders = set()
        if isinstance(self.routing_model, CMTPRoutingModel) and self.routing_model.tree_construction_started and not self.routing_model.tree_ready:
            # 更新CMTP协议状态
            self.routing_model.update_protocol_status(None, sim_time)
            
            # 在树构建阶段，这些包不应该被处理冲突和干扰
            for sender in potential_senders:
                packet = sender.tx_queue[0]
                # 计算等待时间和剩余时间
                elapsed = sim_time - (self.routing_model.tree_build_start_time or sim_time)
                remaining = max(0, self.routing_model.min_tree_build_time - elapsed)
                
                # 只有在这个时间片内未打印过该包的等待日志时才打印
                if packet.id not in self.logged_waiting_packets:
                    # 显示每个数据包的等待状态
                    progress = self.routing_model.tree_build_progress
                    print(f"⏳ {packet.id}: 等待CMTP树构建完成 [{progress:.2f}], 已等待={elapsed:.1f}秒, 剩余≈{remaining:.1f}秒")
                    # 标记为已打印等待日志
                    self.logged_waiting_packets.add(packet.id)
                
                if hasattr(packet, 'add_event'):
                    info = f"等待CMTP树构建完成，当前进度={self.routing_model.tree_build_progress:.2f}, 已等待={elapsed:.1f}秒, 剩余≈{remaining:.1f}秒"
                    packet.add_event("cmtp_waiting_tree", sender.id, packet.current_hop_index, sim_time, info)
                
                # 将此发送者添加到过滤列表
                filtered_senders.add(sender)
            
            # 显示树构建进度
            elapsed = sim_time - (self.routing_model.tree_build_start_time or sim_time)
            print(f"◆ CMTP构建进度: {self.routing_model.tree_build_progress:.2f}, 已用时间: {elapsed:.1f}秒, 跳过{len(filtered_senders)}个数据包的传输")
                    
        # 只处理未被过滤的发送者
        active_senders = potential_senders - filtered_senders
                    
        # --- 距离干扰检测 ---
        # 统计本时间片所有接收节点及其对应的(发送者,包)
        receiver_to_senders = {}
        for sender in active_senders:
            packet = sender.tx_queue[0]
            receiver_id = packet.get_next_hop_id()
            if receiver_id:
                if receiver_id not in receiver_to_senders:
                    receiver_to_senders[receiver_id] = []
                receiver_to_senders[receiver_id].append((sender, packet))
        receivers = list(receiver_to_senders.keys())
        # 检查所有接收节点对，若距离<20，且都在接收包，则这些包都发生距离干扰
        interfered_receivers = set()
        for i in range(len(receivers)):
            for j in range(i+1, len(receivers)):
                r1 = self.uav_map[receivers[i]]
                r2 = self.uav_map[receivers[j]]
                dist = ((r1.x - r2.x)**2 + (r1.y - r2.y)**2 + (r1.z - r2.z)**2) ** 0.5
                if dist < 20:
                    interfered_receivers.add(receivers[i])
                    interfered_receivers.add(receivers[j])
                    # 简化距离干扰检测日志
        # 记录所有受距离干扰影响的(发送者,包)
        interfered_senders = set()
        for rid in interfered_receivers:
            for sender, packet in receiver_to_senders[rid]:
                interfered_senders.add((sender, packet))
                # 新增：记录并发事件
                if hasattr(packet, 'add_event'):
                    packet.add_event("concurrency_detected", packet.current_holder_id, packet.current_hop_index, self.sim_time, "distance_interference")
        # --- 多对一冲突检测 ---
        transmitters_this_step = set()
        collision_groups = []
        for receiver_id, senders in receiver_to_senders.items():
            if len(senders) > 1:
                # 简化多对一冲突检测日志 - 不再在这里输出，由_handle_new_collision处理
                collision_groups.append((receiver_id, senders))
                # 新增：为所有冲突包添加并发事件
                for sender, packet in senders:
                    if hasattr(packet, 'add_event'):
                        packet.add_event("concurrency_detected", packet.current_holder_id, packet.current_hop_index, self.sim_time, "collision")
            else:
                transmitters_this_step.add(senders[0][0])
        # --- 智能并发感知路由决策 ---
        # 仅并发（无硬冲突）
        for receiver_id, senders in receiver_to_senders.items():
            if len(senders) == 1 and receiver_id not in interfered_receivers and receiver_id not in [cg[0] for cg in collision_groups]:
                sender, packet = senders[0]
                uav1 = sender
                receiver = self.uav_map.get(receiver_id)
                # 检查是否存在并发（与其他链路）
                is_concurrent = False
                penalty = 0.0
                # 检查与所有其他包的当前跳是否并发
                for other_uav in self.all_uavs:
                    if other_uav.id == uav1.id or not other_uav.tx_queue:
                        continue
                    other_packet = other_uav.tx_queue[0]
                    other_receiver_id = other_packet.get_next_hop_id()
                    if other_receiver_id is None or other_receiver_id == receiver_id:
                        continue
                    other_receiver = self.uav_map.get(other_receiver_id)
                    # 修正：确保所有对象都不为None
                    if not (uav1 and receiver and other_uav and other_receiver):
                        continue
                    if self.routing_model is not None:
                        if isinstance(self.routing_model, DHyTPRoutingModel):
                            concurrent = self.routing_model.cmtp.are_vectors_concurrent(
                                (uav1.x, uav1.y), (receiver.x, receiver.y),
                                (other_uav.x, other_uav.y), (other_receiver.x, other_receiver.y))
                        else:
                            concurrent = self.routing_model.are_vectors_concurrent(
                                (uav1.x, uav1.y), (receiver.x, receiver.y),
                                (other_uav.x, other_uav.y), (other_receiver.x, other_receiver.y))
                    else:
                        concurrent = False
                    if concurrent:
                        is_concurrent = True
                        # 计算并发惩罚
                        if self.routing_model is not None:
                            if isinstance(self.routing_model, DHyTPRoutingModel):
                                penalty = self.routing_model.cmtp.calculate_concurrent_region_delay(
                                    (uav1.x, uav1.y), (receiver.x, receiver.y),
                                    (other_uav.x, other_uav.y), (other_receiver.x, other_receiver.y))
                            else:
                                penalty = self.routing_model.calculate_concurrent_region_delay(
                                    (uav1.x, uav1.y), (receiver.x, receiver.y),
                                    (other_uav.x, other_uav.y), (other_receiver.x, other_receiver.y))
                        else:
                            penalty = 0.0
                        break
                if is_concurrent:
                    # 1. 直闯并发区域的总时延
                    if self.routing_model is not None:
                        if isinstance(self.routing_model, DHyTPRoutingModel):
                            base_delay = self.routing_model.cmtp.get_link_base_delay(uav1, receiver)
                        else:
                            base_delay = self.routing_model.get_link_base_delay(uav1, receiver)
                    else:
                        base_delay = 0.0
                    direct_total_delay = base_delay + penalty
                    # 2. 遍历所有可用绕路路径，找最小时延
                    # 这里只找一条最短非并发路径（可扩展为多路径）
                    min_reroute_delay = float('inf')
                    best_reroute_path = None
                    # 用最短路径算法，临时屏蔽与当前并发链路重叠的路径
                    # 这里只做简单实现：如果有其他路径且不与并发链路重叠就选
                    # 获取所有路径的接口需你根据实际情况完善
                    # 这里用原有get_shortest_path作为示例
                    orig_path_ids, _ = self.sim_manager.get_shortest_path(uav1.id, receiver_id)
                    if orig_path_ids and len(orig_path_ids) > 1:
                        # 检查是否与并发链路重叠
                        overlap = False
                        for i in range(len(orig_path_ids) - 1):
                            u1 = self.uav_map.get(orig_path_ids[i])
                            u2 = self.uav_map.get(orig_path_ids[i+1])
                            if not (uav1 and receiver and u1 and u2):
                                continue
                            else:
                                seg1 = {(uav1.x, uav1.y), (receiver.x, receiver.y)}
                                seg2 = {(u1.x, u1.y), (u2.x, u2.y)}
                                if seg1 == seg2:
                                    overlap = True
                                    break
                        if not overlap:
                            reroute_delay = 0.0
                            for i in range(len(orig_path_ids) - 1):
                                u1 = self.uav_map.get(orig_path_ids[i])
                                u2 = self.uav_map.get(orig_path_ids[i+1])
                                if u1 is not None and u2 is not None:
                                    if self.routing_model is not None:
                                        if isinstance(self.routing_model, DHyTPRoutingModel):
                                            reroute_delay += self.routing_model.cmtp.get_link_base_delay(u1, u2)
                                        else:
                                            reroute_delay += self.routing_model.get_link_base_delay(u1, u2)
                                    else:
                                        reroute_delay = float('inf')
                                        break
                            if reroute_delay < min_reroute_delay:
                                min_reroute_delay = reroute_delay
                                best_reroute_path = orig_path_ids
                    # 3. 比较
                    if direct_total_delay <= min_reroute_delay:
                        # 选择直闯，记录并发惩罚
                        packet.last_concurrent_penalty = penalty
                        if hasattr(packet, 'add_event'):
                            packet.add_event("concurrent_decision", packet.current_holder_id, packet.current_hop_index, self.sim_time, f"Direct with penalty={penalty:.3f}")
                        # 继续正常发送流程
                    else:
                        # 选择绕路，切换路径
                        if best_reroute_path and len(best_reroute_path) > 1:
                            current_hop_idx = packet.current_hop_index
                            original_path = packet.path
                            new_full_path = original_path[:current_hop_idx] + best_reroute_path + original_path[current_hop_idx + 2:]
                            packet.path = new_full_path
                            if hasattr(packet, 'add_event'):
                                packet.add_event("reroute_success", sender.id, current_hop_idx, self.sim_time, f"Reroute via {best_reroute_path[1]}")
                            for i in range(current_hop_idx, len(new_full_path) - 1):
                                next_hop_id2 = new_full_path[i + 1]
                                next_hop_uav = self.uav_map.get(next_hop_id2)
                                if next_hop_uav:
                                    packet.record_next_hop_position(next_hop_id2, next_hop_uav.x, next_hop_uav.y, next_hop_uav.z)
                        # 继续正常发送流程
        # --- 队列合并规则 ---
        # 检查是否有节点同时在距离干扰和多对一冲突中
        collision_senders = set()
        for receiver_id, senders in collision_groups:
            for sender, packet in senders:
                collision_senders.add((sender, packet))
        overlap = {s for s, p in interfered_senders} & {s for s, p in collision_senders}
        if overlap:
            # 合并所有相关节点进 collision_queues（已有逻辑，无需变动）
            all_conflict = interfered_senders | collision_senders
            for receiver_id, senders in receiver_to_senders.items():
                if any((s, p) in all_conflict for s, p in senders):
                    self._handle_new_collision([s for s, p in senders], receiver_id)
        else:
            # 分别处理：多对一冲突和距离干扰分别入各自队列
            for receiver_id, senders in collision_groups:
                self._handle_new_collision([s for s, p in senders], receiver_id)
            for receiver_id in interfered_receivers:
                self._handle_new_distance_interference([s for s, p in receiver_to_senders[receiver_id]], receiver_id)
        # 重新识别本轮已被处理的发送者
        handled_senders = {s for s, p in interfered_senders} | {s for s, p in collision_senders}
        unhandled_senders = potential_senders - handled_senders
        # 额外排除所有在冲突队列和干扰队列中的包
        collision_queue_senders = set()
        for queue in self.collision_queues.values():
            for sender_id in queue:
                sender_obj = self.uav_map.get(sender_id)
                if sender_obj:
                    collision_queue_senders.add(sender_obj)
        distance_queue_senders = set()
        for queue in self.distance_interference_queues.values():
            for sender_id in queue:
                sender_obj = self.uav_map.get(sender_id)
                if sender_obj:
                    distance_queue_senders.add(sender_obj)
        unhandled_senders = unhandled_senders - collision_queue_senders - distance_queue_senders
        # 其余发送者正常尝试发送
        for sender in unhandled_senders:
            self.total_hop_attempts += 1
            packet = sender.tx_queue[0]
            receiver_id = packet.get_next_hop_id()
            receiver = self.uav_map.get(receiver_id)
            
            # 显示树构建进度（如果是第一个发送者）
            if isinstance(self.routing_model, DHyTPRoutingModel) and sender == next(iter(unhandled_senders), None):
                # 周期性显示树构建进度
                if hasattr(self.routing_model, 'tree_build_progress') and self.routing_model.tree_construction_started:
                    print(f"◆ 树构建进度: {self.routing_model.tree_build_progress:.2f}")
            
            # 显示路由选择信息（与传输日志交错）
            if isinstance(self.routing_model, DHyTPRoutingModel) and receiver:
                destination_id = packet.destination_id
                mode = "【CMTP】" if self.routing_model.use_cmtp else "【PTP】"
                progress = self.routing_model.tree_build_progress
                # 使用高亮显示格式
                print(f"\033[1;31;40m{mode} UAV-{sender.id}→{destination_id} 选择→UAV-{receiver.id} 进度:{progress:.2f}\033[0m")
            elif isinstance(self.routing_model, CMTPRoutingModel) and receiver and self.routing_model.tree_ready:
                destination_id = packet.destination_id
                mode = "【CMTP】"
                progress = self.routing_model.tree_build_progress
                # 使用高亮显示格式
                print(f"\033[1;31;40m{mode} UAV-{sender.id}→{destination_id} 选择→UAV-{receiver.id} 进度:{progress:.2f}\033[0m")
            
            is_successful, fail_reason = self._attempt_transmission(sender, receiver, list(potential_senders))
            if is_successful:
                self._handle_success(sender, receiver, packet)
            else:
                self._handle_failure(sender, packet, fail_reason or "Transmission_Error")

        # 新增：分别处理距离干扰队列和多对一冲突队列的队首
        self._process_distance_interference_queues()
        self._process_collision_queues()

        # 记录所有包的实时状态
        self._collect_packet_status()

    # 在_reroute_and_select_best_path方法中添加DHyTP支持
    def _reroute_and_select_best_path(self, sender, packet, next_hop_id):
        """重新规划路径并选择最佳下一跳，支持DHyTP和CMTP协议"""
        destination_id = packet.destination_id
        
        # 初始化变量以避免未定义错误
        current_hop_idx = packet.current_hop_index
        
        # ## **** ENERGY MODIFICATION START: 添加协议操作能耗 **** ##
        # PTP协议不需要额外计算能耗
        # 只有CMTP和DHYTP在树构建和维护时会产生额外能耗，在相应的协议类中处理
        # ## **** ENERGY MODIFICATION END **** ##
        
        # 使用DHyTP协议选择下一跳
        if isinstance(self.routing_model, DHyTPRoutingModel) and USE_DHYTP_ROUTING_MODEL:
            # 获取候选邻居
            candidate_neighbors = []
            for uav in self.all_uavs:
                if uav.id == sender.id:
                    continue
                dist = math.sqrt((sender.x - uav.x)**2 + (sender.y - uav.y)**2 + (sender.z - uav.z)**2)
                if dist <= UAV_COMMUNICATION_RANGE:
                    candidate_neighbors.append(uav)
            
            # 简化调试日志
            next_hop, metric = self.routing_model.select_next_hop(
                sender, candidate_neighbors, destination_id=destination_id,
                packet=packet, sim_time=self.sim_time
            )
            
            if next_hop:
                return next_hop, next_hop.id, "dhytp_routing"
        
        # 使用CMTP协议选择下一跳
        elif isinstance(self.routing_model, CMTPRoutingModel) and USE_CTMP_ROUTING_MODEL:
            # 获取候选邻居
            candidate_neighbors = []
            for uav in self.all_uavs:
                if uav.id == sender.id:
                    continue
                dist = math.sqrt((sender.x - uav.x)**2 + (sender.y - uav.y)**2 + (sender.z - uav.z)**2)
                if dist <= UAV_COMMUNICATION_RANGE:
                    candidate_neighbors.append(uav)
            
            # 简化调试日志
            next_hop, metric = self.routing_model.select_next_hop(
                sender, candidate_neighbors, layer=0,
                packet=packet, sim_time=self.sim_time
            )
            
            if next_hop:
                return next_hop, next_hop.id, "cmtp_routing"
        
        # 原有的重路由逻辑
        import simulation_config
        # 强制使用PTP模型
        old_flag = getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False)
        simulation_config.USE_PTP_ROUTING_MODEL = True
        ptp_path_ids, _ = self.sim_manager.get_shortest_path(
            sender.id, destination_id  # 直接使用目的地址而不是next_hop_id
        )
        simulation_config.USE_PTP_ROUTING_MODEL = old_flag
        ptp_delay = None
        if ptp_path_ids and len(ptp_path_ids) > 1:
            ptp_delay = 0.0
            for i in range(len(ptp_path_ids) - 1):
                uav1 = self.uav_map.get(ptp_path_ids[i])
                uav2 = self.uav_map.get(ptp_path_ids[i+1])
                if uav1 and uav2:
                    if self.routing_model is not None:
                        if isinstance(self.routing_model, DHyTPRoutingModel):
                            ptp_delay += self.routing_model.ptp.get_link_base_delay(uav1, uav2)
                        elif isinstance(self.routing_model, CMTPRoutingModel):
                            ptp_delay += self.routing_model.get_link_base_delay(uav1, uav2)
                        else:
                            ptp_delay += self.routing_model.get_link_base_delay(uav1, uav2)
                    else:
                        ptp_delay = float('inf')
                        break
        
        # 检查是否找到了有效路径
        new_full_path = None  # 初始化变量
        if ptp_path_ids and len(ptp_path_ids) > 1:
            # 获取下一跳
            next_hop_id = ptp_path_ids[1]  # 下一跳是路径中的第二个节点
            next_hop_uav = self.uav_map.get(next_hop_id)
            
            if next_hop_uav:
                # 更新路径
                original_path = packet.path
                new_full_path = original_path[:current_hop_idx] + ptp_path_ids
                packet.path = new_full_path
                
                # 记录事件和节点位置
                if hasattr(packet, 'add_event'):
                    packet.add_event("reroute_success", sender.id, current_hop_idx, self.sim_time, f"New path via {next_hop_id}")
                
                # 记录新路径中每个下一跳节点的位置
                for i in range(current_hop_idx, len(new_full_path) - 1):
                    next_hop_id2 = new_full_path[i + 1]
                    next_hop_uav2 = self.uav_map.get(next_hop_id2)
                    if next_hop_uav2:
                        packet.record_next_hop_position(next_hop_id2, next_hop_uav2.x, next_hop_uav2.y, next_hop_uav2.z)
                
                # 简化路由重算成功日志
                print(f"⟲ {packet.id}: 路由重算 {sender.id}→{next_hop_id}")
                return next_hop_uav, next_hop_id, "rerouted"
        
        # 简化路由失败日志
        print(f"✗ {packet.id}: 路由失败 {sender.id}→{destination_id}")
        return None, None, "no_valid_path"
        
    def _attempt_transmission(self, sender, receiver, all_transmitters):
        """执行单次传输尝试的所有检查，返回(is_successful, reason)"""
        if not sender.tx_queue:
            return False, "无数据包"
        
        packet = sender.tx_queue[0]
        
        # 检查CMTP协议树构建状态，如果树正在构建中且未完成，则不发送数据包
        if isinstance(self.routing_model, CMTPRoutingModel) and self.routing_model.tree_construction_started and not self.routing_model.tree_ready:
            # 更新协议状态，传递仿真时间
            self.routing_model.update_protocol_status(None, self.sim_time)
            
            # 再次检查，因为update_protocol_status可能会更新tree_ready状态
            if not self.routing_model.tree_ready:
                # 显示等待树构建完成的日志（只有在这个时间片内未打印过该包的等待日志时才打印）
                if packet.id not in getattr(self, 'logged_waiting_packets', set()):
                    elapsed = self.sim_time - (self.routing_model.tree_build_start_time or self.sim_time)
                    remaining = max(0, self.routing_model.min_tree_build_time - elapsed)
                    progress = self.routing_model.tree_build_progress
                    print(f"⏳ {packet.id}: 等待CMTP树构建完成 [{progress:.2f}], 已等待={elapsed:.1f}秒, 剩余≈{remaining:.1f}秒")
                    # 标记为已打印等待日志
                    if not hasattr(self, 'logged_waiting_packets'):
                        self.logged_waiting_packets = set()
                    self.logged_waiting_packets.add(packet.id)
                
                # 记录等待树构建完成的事件
                if hasattr(packet, 'add_event'):
                    elapsed = self.sim_time - (self.routing_model.tree_build_start_time or self.sim_time)
                    info = f"等待CMTP树构建完成，当前进度={self.routing_model.tree_build_progress:.2f}, 已等待={elapsed:.1f}秒"
                    packet.add_event("waiting_tree_construction", sender.id, packet.current_hop_index, self.sim_time, info)
                return False, "Waiting_Tree_Construction"
            else:
                # 树刚刚构建完成，输出通知消息
                print(f"⚡ {packet.id}: CMTP树已构建完成，开始传输...")
        
        # ## **** ENERGY MODIFICATION START: 传输能耗统计 **** ##
        # 移动到树构建检查之后，确保只有在真正传输时才计算能耗
        from simulation_config import COLLECT_ENERGY_STATS, ENERGY_UNIT_SEND, ENERGY_RETRANSMISSION_PENALTY
        if COLLECT_ENERGY_STATS and packet and receiver:
            # 基础传输能耗（固定值，不再与距离相关）
            tx_energy = ENERGY_UNIT_SEND
            
            # 检查是否为重传，如果是则应用惩罚系数
            if packet.retransmission_count > 0:
                # 重传能耗有惩罚系数
                retrans_energy = tx_energy * ENERGY_RETRANSMISSION_PENALTY
                packet.add_retransmission_energy(retrans_energy)
                print(f"⚡ {packet.id}: 重传能耗 +{retrans_energy:.2f}J [第{packet.retransmission_count}次重传]")
            else:
                # 正常传输能耗
                packet.add_transmission_energy(tx_energy)
                print(f"⚡ {packet.id}: 传输能耗 +{tx_energy:.2f}J")
        # ## **** ENERGY MODIFICATION END **** ##
        
        # 显示树构建进度
        if isinstance(self.routing_model, DHyTPRoutingModel) and self.routing_model.tree_construction_started:
            # 周期性显示树构建进度
            if hasattr(self.routing_model, 'tree_build_progress'):
                print(f"◆ DHyTP构建进度: {self.routing_model.tree_build_progress:.2f}")
        elif isinstance(self.routing_model, CMTPRoutingModel) and self.routing_model.tree_construction_started:
            # 周期性显示树构建进度
            if hasattr(self.routing_model, 'tree_build_progress'):
                elapsed = self.sim_time - (self.routing_model.tree_build_start_time or self.sim_time)
                print(f"◆ CMTP构建进度: {self.routing_model.tree_build_progress:.2f}, 已用时间: {elapsed:.1f}秒")
        
        # 智能处理Receiver_None问题 - 当接收者为空时尝试重新路由
        if receiver is None:
            # 获取目标节点ID
            destination_id = packet.destination_id
            
            # 尝试重新路由到目标节点
            print(f"⚡ {packet.id}: 接收者为空，尝试重新路由 {sender.id}→{destination_id}")
            
            # 获取候选邻居
            candidate_neighbors = []
            for uav in self.all_uavs:
                if uav.id == sender.id:
                    continue
                dist = math.sqrt((sender.x - uav.x)**2 + (sender.y - uav.y)**2 + (sender.z - uav.z)**2)
                if dist <= UAV_COMMUNICATION_RANGE:
                    candidate_neighbors.append(uav)
            
            # 如果没有邻居，直接失败
            if not candidate_neighbors:
                self._log_failure(packet, "No_Neighbors")
                return False, "No_Neighbors"
            
            # 使用DHyTP或CMTP或其他可用的路由协议重新计算路径
            if isinstance(self.routing_model, DHyTPRoutingModel) and USE_DHYTP_ROUTING_MODEL:
                # 使用DHyTP协议选择新的下一跳
                next_hop, _ = self.routing_model.select_next_hop(
                    sender, candidate_neighbors, destination_id=destination_id,
                    packet=packet, sim_time=self.sim_time
                )
                
                if next_hop:
                    # 更新路径
                    current_hop_idx = packet.current_hop_index
                    if current_hop_idx < len(packet.path) - 1:
                        packet.path[current_hop_idx + 1] = next_hop.id
                        # 记录新的下一跳位置
                        packet.record_next_hop_position(next_hop.id, next_hop.x, next_hop.y, next_hop.z)
                        # 更新receiver为新的下一跳
                        receiver = next_hop
                        
                        if hasattr(packet, 'add_event'):
                            packet.add_event("route_recovery", sender.id, packet.current_hop_index, 
                                           self.sim_time, f"恢复路由: {next_hop.id}")
                        print(f"⚡ {packet.id}: 路由恢复成功，新的下一跳: {next_hop.id}")
                    else:
                        # 如果是最后一跳，但接收者为空，说明目标节点可能已经移动
                        # 尝试完全重新路由
                        next_hop_uav, next_hop_id, reason = self._reroute_and_select_best_path(sender, packet, destination_id)
                        if next_hop_uav:
                            receiver = next_hop_uav
                        else:
                            self._log_failure(packet, "Route_Recovery_Failed")
                            return False, "Route_Recovery_Failed"
            elif isinstance(self.routing_model, CMTPRoutingModel) and USE_CTMP_ROUTING_MODEL:
                # 使用CMTP协议选择新的下一跳
                next_hop, _ = self.routing_model.select_next_hop(
                    sender, candidate_neighbors, layer=0,
                    packet=packet, sim_time=self.sim_time
                )
                
                if next_hop:
                    # 更新路径
                    current_hop_idx = packet.current_hop_index
                    if current_hop_idx < len(packet.path) - 1:
                        packet.path[current_hop_idx + 1] = next_hop.id
                        # 记录新的下一跳位置
                        packet.record_next_hop_position(next_hop.id, next_hop.x, next_hop.y, next_hop.z)
                        # 更新receiver为新的下一跳
                        receiver = next_hop
                        
                        if hasattr(packet, 'add_event'):
                            packet.add_event("route_recovery", sender.id, packet.current_hop_index, 
                                           self.sim_time, f"恢复路由: {next_hop.id}")
                        print(f"⚡ {packet.id}: 路由恢复成功，新的下一跳: {next_hop.id}")
                    else:
                        # 如果是最后一跳，但接收者为空，说明目标节点可能已经移动
                        # 尝试完全重新路由
                        next_hop_uav, next_hop_id, reason = self._reroute_and_select_best_path(sender, packet, destination_id)
                        if next_hop_uav:
                            receiver = next_hop_uav
                        else:
                            self._log_failure(packet, "Route_Recovery_Failed")
                            return False, "Route_Recovery_Failed"
            else:
                # 使用默认路由重算
                next_hop_uav, next_hop_id, reason = self._reroute_and_select_best_path(sender, packet, destination_id)
                if next_hop_uav:
                    receiver = next_hop_uav
                else:
                    self._log_failure(packet, "Route_Recovery_Failed")
                    return False, "Route_Recovery_Failed"
            
            # 如果经过恢复后receiver仍为None，则失败
            if receiver is None:
                self._log_failure(packet, "Recovery_Failed")
                return False, "Recovery_Failed"
        
        # 原有的传输逻辑继续
        # === PTP链路估算（用当前sender/receiver位置） ===
        if self.routing_model is not None:
            if isinstance(self.routing_model, DHyTPRoutingModel):
                link_delay = self.routing_model.cmtp.get_link_base_delay(sender, receiver)
            elif isinstance(self.routing_model, CMTPRoutingModel):
                link_delay = self.routing_model.get_link_base_delay(sender, receiver)
            else:
                link_delay = self.routing_model.get_link_base_delay(sender, receiver)
            self._log(f"Pkt:{packet.id} PTP link delay: {link_delay:.2f}")
            
        # === 位置变动检查 ===
        next_hop_id = packet.get_next_hop_id()
        if next_hop_id and receiver:
            position_changed, distance_change = packet.check_next_hop_position_change(
                next_hop_id, receiver.x, receiver.y, receiver.z, threshold=POSITION_CHANGE_THRESHOLD
            )
            if position_changed:
                # 记录位置变动事件
                if hasattr(packet, 'add_event'):
                    packet.add_event("position_change", sender.id, packet.current_hop_index, self.sim_time, 
                                   f"Next hop {next_hop_id} moved {distance_change:.2f}m")
                self._log(f"Pkt:{packet.id} Next hop {next_hop_id} position changed by {distance_change:.2f}m")
                # 立即修复路径
                next_hop_uav, next_hop_id, reason = self._reroute_and_select_best_path(sender, packet, next_hop_id)
                # 修复后，重新获取新路径的下一跳和receiver
                if next_hop_uav:
                    receiver = next_hop_uav
                else:
                    next_hop_id = packet.get_next_hop_id()
                    receiver = self.uav_map.get(next_hop_id)
                # 注意：如果修复失败，receiver可能为None，需处理
                if receiver is None:
                    self._log_failure(packet, "Receiver_None_AfterReroute")
                    return False, "Receiver_None_AfterReroute"
                    
        # PRR丢包判定
        import simulation_config
        if getattr(simulation_config, 'USE_PRR_FAILURE_MODEL', True):
            if self.comm_model.check_prr_failure(receiver):
                # 添加PRR失败事件，只记录一次
                if hasattr(packet, 'add_event'):
                    packet.add_event("prr_failure", packet.current_holder_id, packet.current_hop_index, 
                                   self.sim_time, "概率丢包(PRR)")
                self._log_failure(packet, "PRR(Packet_Loss)")
                return False, "PRR"
                
        # ## **** ENERGY MODIFICATION START: 计算传输能耗 **** ##
        if COLLECT_ENERGY_STATS and packet:
            # 计算基本传输能耗（固定值，不再与距离相关）
            tx_energy = ENERGY_UNIT_SEND
            
            # 如果是重传，添加惩罚系数
            if packet.retransmission_count > 0:
                tx_energy *= ENERGY_RETRANSMISSION_PENALTY
                packet.add_retransmission_energy(tx_energy)
            else:
                packet.add_transmission_energy(tx_energy)
        # ## **** ENERGY MODIFICATION END **** ##
                
        return True, None
        
    def _handle_failure(self, sender, packet, reason):
        """
        处理传输失败，根据reason更新包状态，但不从队列中移除
        """
        # 检查CMTP协议树构建状态
        if isinstance(self.routing_model, CMTPRoutingModel) and self.routing_model.tree_construction_started and not self.routing_model.tree_ready:
            # CMTP树构建阶段的失败不应增加重传计数，只记录等待事件
            # 计算等待和剩余时间
            elapsed = self.sim_time - (self.routing_model.tree_build_start_time or self.sim_time)
            remaining = max(0, self.routing_model.min_tree_build_time - elapsed)
            progress = self.routing_model.tree_build_progress
            
            # 只有在这个时间片内未打印过该包的等待日志时才打印
            if packet.id not in getattr(self, 'logged_waiting_packets', set()):
                print(f"⏳ {packet.id}: 等待CMTP树构建完成 [{progress:.2f}], 已等待={elapsed:.1f}秒, 剩余≈{remaining:.1f}秒")
                # 标记为已打印等待日志
                if not hasattr(self, 'logged_waiting_packets'):
                    self.logged_waiting_packets = set()
                self.logged_waiting_packets.add(packet.id)
            
            if hasattr(packet, 'add_event'):
                info = f"等待CMTP树构建完成，当前进度={self.routing_model.tree_build_progress:.2f}, 已等待={elapsed:.1f}秒, 剩余≈{remaining:.1f}秒"
                packet.add_event("waiting_tree_build", sender.id, packet.current_hop_index, self.sim_time, info)
            return

        # 常规失败处理
        if "Waiting_Tree_Construction" in reason:
            # 树构建等待不计入重传
            return
        
        # 非树构建等待失败，增加重传计数
        if reason != "Distance_Interference_Queue" and reason != "Collision_Queue":  # 这两种已经在队列中增加过了
            packet.retransmission_count += 1
            
        # 更新状态
        if "Distance_Interference" in reason:
            new_status = "distance_interference"
            if not hasattr(packet, 'last_concurrent_penalty'):
                packet.last_concurrent_penalty = 0.0
            # 记录事件
            if hasattr(packet, 'add_event'):
                packet.add_event("transmission_fail", sender.id, packet.current_hop_index, self.sim_time, f"距离干扰失败: {reason}")
        elif "Collision" in reason:
            new_status = "collision"
            if not hasattr(packet, 'last_concurrent_penalty'):
                packet.last_concurrent_penalty = 0.0
            # 记录事件
            if hasattr(packet, 'add_event'):
                packet.add_event("transmission_fail", sender.id, packet.current_hop_index, self.sim_time, f"碰撞失败: {reason}")
        elif "PRR" in reason:
            new_status = "prr_fail"
            # 记录事件
            if hasattr(packet, 'add_event'):
                packet.add_event("transmission_fail", sender.id, packet.current_hop_index, self.sim_time, f"PRR失败: {reason}")
        elif "No_Receiver" in reason or "No_Neighbors" in reason or "Route_Recovery_Failed" in reason:
            new_status = "routing_fail"
            # 记录事件
            if hasattr(packet, 'add_event'):
                packet.add_event("transmission_fail", sender.id, packet.current_hop_index, self.sim_time, f"路由失败: {reason}")
        else:
            new_status = f"failed_{reason}"
            # 记录事件
            if hasattr(packet, 'add_event'):
                packet.add_event("transmission_fail", sender.id, packet.current_hop_index, self.sim_time, f"其他失败: {reason}")
        
        packet.status = new_status
        self._log_failure(packet, reason)
        
        # 过多重传则丢弃
        if packet.retransmission_count >= MAX_RETRANSMISSIONS:
            self._handle_terminal_failure(sender, packet, f"Max_Retries({reason})")


    def _handle_success(self, sender, receiver, packet):
        """处理成功的传输"""
        # 简化日志输出
        self._log(f"Pkt:{packet.id} ({sender.id}->{receiver.id}) OK.")
        
        # ## **** ENERGY MODIFICATION START: 接收能耗统计 **** ##
        from simulation_config import COLLECT_ENERGY_STATS, ENERGY_UNIT_RECEIVE
        if COLLECT_ENERGY_STATS and packet:
            packet.add_transmission_energy(ENERGY_UNIT_RECEIVE)
            print(f"⚡ {packet.id}: 接收能耗 +{ENERGY_UNIT_RECEIVE:.2f}J")
        # ## **** ENERGY MODIFICATION END **** ##
        
        # 如果发送者在冲突队列中，将其移出
        receiver_id = packet.get_next_hop_id()
        if receiver_id in self.collision_queues and self.collision_queues[receiver_id]:
            if self.collision_queues[receiver_id][0] == sender.id:
                self.collision_queues[receiver_id].popleft()

        # 正常处理数据包和发送队列
        sender.tx_queue.popleft()
        
        # 新增：记录成功事件
        if hasattr(packet, 'add_event'):
            packet.add_event("success", packet.current_holder_id, packet.current_hop_index, self.sim_time)
        
        # 数据包前进到下一跳，更新当前持有者ID
        packet.advance_hop(self.sim_time)
        packet.actual_hops.append(packet.current_holder_id)
        
        # 简化传输成功日志，包含更多信息但减少输出量
        print(f"✓ {packet.id}: {sender.id}→{receiver.id} [跳:{packet.current_hop_index}]")
        
        if hasattr(packet, 'per_hop_waits'):
            packet.per_hop_waits.append(0)
        
        import simulation_config
        use_ptp = getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False)
        use_cmtp = getattr(simulation_config, 'USE_CTMP_ROUTING_MODEL', False)
        
        # 统一统计true_total_delay，基础时延和并发惩罚都用实际消耗的时间片累计
        if not hasattr(packet, 'true_total_delay'):
            packet.true_total_delay = 0.0
        if hasattr(packet, 'per_hop_waits') and len(packet.per_hop_waits) >= 2:
            # 统计上一跳实际等待的时间片数
            time_increment = getattr(simulation_config, 'DEFAULT_TIME_INCREMENT', 0.1)
            wait_steps = packet.per_hop_waits[-2]  # 刚完成的上一跳等待步数
            concurrent_penalty = getattr(packet, 'last_concurrent_penalty', 0.0)
            concurrent_penalty_steps = math.ceil(concurrent_penalty / time_increment)
            if use_ptp or use_cmtp:
                # PTP或CMTP协议下都不累计PTP的并发惩罚
                packet.true_total_delay += wait_steps * time_increment
            else:
                # 只有都未启用时才累计PTP的并发惩罚
                packet.true_total_delay += wait_steps * time_increment + concurrent_penalty_steps * time_increment
            if hasattr(packet, 'add_event'):
                penalty_val = concurrent_penalty if not (use_ptp or use_cmtp) else 0.0
                penalty_steps_val = concurrent_penalty_steps if not (use_ptp or use_cmtp) else 0
                packet.add_event("true_hop_delay", packet.current_holder_id, packet.current_hop_index, self.sim_time, f"Add wait_steps={wait_steps}, concurrent_penalty={penalty_val}, concurrent_penalty_steps={penalty_steps_val}")
            # 累计后及时清零，避免影响下一跳
            packet.last_concurrent_penalty = 0.0

        # 检查是否到达最终目的地
        if packet.current_holder_id == packet.destination_id:
            packet.status = "delivered"
            # 简化交付日志
            print(f"★ {packet.id} 已送达目的地 UAV-{receiver.id}! 总跳数:{packet.current_hop_index}")
            if hasattr(packet, 'add_event'):
                packet.add_event("delivered", packet.current_holder_id, packet.current_hop_index, self.sim_time)
        else:
            # 未到达最终目的地，添加到接收者的发送队列
            receiver.add_packet_to_queue(packet)
            # 删除冗余的队列日志

        # ## **** ENERGY MODIFICATION START: 添加接收能耗 **** ##
        from simulation_config import ENERGY_UNIT_RECEIVE, COLLECT_ENERGY_STATS
        
        if COLLECT_ENERGY_STATS and packet:
            # 添加接收能耗
            packet.add_transmission_energy(ENERGY_UNIT_RECEIVE)
        # ## **** ENERGY MODIFICATION END **** ##

    def _handle_new_distance_interference(self, senders, receiver_id):
        """
        处理新的距离干扰。类似于碰撞，但加入距离干扰队列而非碰撞队列。
        """
        # 检查CMTP协议树构建状态，如果在树构建阶段则不处理距离干扰
        if isinstance(self.routing_model, CMTPRoutingModel) and self.routing_model.tree_construction_started and not self.routing_model.tree_ready:
            # 在CMTP树构建阶段，不应该处理距离干扰，因为此时数据包应该被暂停发送
            print(f"⚠ 警告: CMTP树构建阶段检测到距离干扰，但应该被暂停发送。这可能是代码逻辑错误。")
            return
            
        # 简化日志输出
        receiver = self.uav_map.get(receiver_id)
        receiver_str = f"{receiver_id}" if receiver else f"{receiver_id}(无效)"
        sender_str = ", ".join([f"{s.id}" for s in senders])
        print(f"⚠ 距离干扰: 接收者-{receiver_str}, 发送者-[{sender_str}]")
        
        # 更新距离干扰队列
        if receiver_id not in self.distance_interference_queues or not self.distance_interference_queues[receiver_id]:
            self.distance_interference_queues[receiver_id] = deque(s.id for s in senders)
        else:
            # 保持原有队列，补充新 sender
            for s in senders:
                if s.id not in self.distance_interference_queues[receiver_id]:
                    self.distance_interference_queues[receiver_id].append(s.id)
                    
        # 添加距离干扰事件和更新统计
        for sender in senders:
            packet = sender.tx_queue[0]
            self.total_hop_attempts += 1
            
            # 更新统计和记录事件
            packet.retransmission_count += 1
            if hasattr(packet, 'add_event'):
                packet.add_event("distance_interference", sender.id, packet.current_hop_index, self.sim_time, f"距离干扰: 接收者-{receiver_id}")
                
            # 检查是否超过最大重传次数
            if packet.retransmission_count >= MAX_RETRANSMISSIONS:
                self._handle_terminal_failure(sender, packet, "Max_Retries(Distance_Interference_Init)")

    def _process_distance_interference_queues(self):
        """
        每个距离干扰队列只允许队首发送者尝试重传。
        若重传成功则移出队列，否则等待下次时间片继续尝试。
        队列为空时自动删除。
        """
        for receiver_id, queue in list(self.distance_interference_queues.items()):
            if not queue:
                continue
            sender_id = queue[0]
            sender = self.uav_map.get(sender_id)
            if sender and sender.tx_queue:
                packet = sender.tx_queue[0]
                receiver = self.uav_map.get(receiver_id)
                
                # 显示路由选择信息（与传输日志交错）
                if isinstance(self.routing_model, DHyTPRoutingModel) and receiver:
                    destination_id = packet.destination_id
                    mode = "【CMTP】" if self.routing_model.use_cmtp else "【PTP】"
                    progress = self.routing_model.tree_build_progress
                    # 使用高亮显示格式
                    print(f"\033[1;31;40m{mode} UAV-{sender.id}→{destination_id} 选择→UAV-{receiver.id} 进度:{progress:.2f}\033[0m")
                elif isinstance(self.routing_model, CMTPRoutingModel) and receiver and self.routing_model.tree_ready:
                    destination_id = packet.destination_id
                    mode = "【CMTP】"
                    progress = self.routing_model.tree_build_progress
                    # 使用高亮显示格式
                    print(f"\033[1;31;40m{mode} UAV-{sender.id}→{destination_id} 选择→UAV-{receiver.id} 进度:{progress:.2f}\033[0m")
                
                is_successful = self._attempt_transmission(sender, receiver, list(self.uav_map.values()))
                if is_successful:
                    self._handle_success(sender, receiver, packet)
                    queue.popleft()
                else:
                    self._handle_failure(sender, packet, "Distance_Interference_Queue")
            # 队列空则删除
            if not queue:
                del self.distance_interference_queues[receiver_id]

    def _process_collision_queues(self):
        """
        每个多对一冲突队列只允许队首发送者尝试重传。
        若重传成功则移出队列，否则等待下次时间片继续尝试。
        队列为空时自动删除。
        """
        for receiver_id, queue in list(self.collision_queues.items()):
            if not queue:
                continue
            sender_id = queue[0]
            sender = self.uav_map.get(sender_id)
            if sender and sender.tx_queue:
                packet = sender.tx_queue[0]
                receiver = self.uav_map.get(receiver_id)
                
                # 显示路由选择信息（与传输日志交错）
                if isinstance(self.routing_model, DHyTPRoutingModel) and receiver:
                    destination_id = packet.destination_id
                    mode = "【CMTP】" if self.routing_model.use_cmtp else "【PTP】"
                    progress = self.routing_model.tree_build_progress
                    # 使用高亮显示格式
                    print(f"\033[1;31;40m{mode} UAV-{sender.id}→{destination_id} 选择→UAV-{receiver.id} 进度:{progress:.2f}\033[0m")
                elif isinstance(self.routing_model, CMTPRoutingModel) and receiver and self.routing_model.tree_ready:
                    destination_id = packet.destination_id
                    mode = "【CMTP】"
                    progress = self.routing_model.tree_build_progress
                    # 使用高亮显示格式
                    print(f"\033[1;31;40m{mode} UAV-{sender.id}→{destination_id} 选择→UAV-{receiver.id} 进度:{progress:.2f}\033[0m")
                
                is_successful = self._attempt_transmission(sender, receiver, list(self.uav_map.values()))
                if is_successful:
                    self._handle_success(sender, receiver, packet)
                    # queue.popleft()  # 已移除，避免重复 pop
                else:
                    self._handle_failure(sender, packet, "Collision_Queue")
            # 队列空则删除
            if not queue:
                del self.collision_queues[receiver_id]

    def get_packet_status_snapshot(self):
        """
        返回当前时间片所有包的实时状态快照
        """
        return self.packet_status_snapshot

    def _collect_packet_status(self):
        """
        收集当前网络中所有包的状态，存入packet_status_snapshot
        """
        self.packet_status_snapshot.clear()
        for uav in self.all_uavs:
            for packet in getattr(uav, 'tx_queue', []):
                self.packet_status_snapshot.append({
                    'id': packet.id,
                    'source_id': packet.source_id,
                    'destination_id': packet.destination_id,
                    'current_holder_id': packet.current_holder_id,
                    'status': packet.status,
                    'current_hop_index': packet.current_hop_index,
                    'retransmission_count': packet.retransmission_count,
                    'actual_hops': list(packet.actual_hops),
                    'per_hop_waits': list(packet.per_hop_waits),
                    'event_history': list(packet.event_history),  # 新增：事件历史
                    'path': list(packet.path) if hasattr(packet, 'path') else [],  # 新增：完整路径
                    'concurrent_delay': getattr(packet, 'concurrent_delay', 0)  # 新增：并发延时
                })

    def collect_final_packet_status(self, all_packets):
        """
        收集所有包的最终状态（包括已送达/失败的包），用于实验结束后日志展示
        """
        self.packet_status_snapshot.clear()
        # ## **** AoI MODIFICATION START: 添加AoI统计变量 **** ##
        total_aoi = 0.0
        delivered_count = 0
        # ## **** AoI MODIFICATION END **** ##
        
        # ## **** ENERGY MODIFICATION START: 添加能耗统计变量 **** ##
        total_energy = 0.0
        # ## **** ENERGY MODIFICATION END **** ##
        
        for packet in all_packets:
            true_total_delay = getattr(packet, 'true_total_delay', 0.0)
            
            # ## **** AoI MODIFICATION START: 计算AoI并累计 **** ##
            if packet.status == "delivered" and hasattr(packet, 'aoi') and packet.aoi is not None:
                total_aoi += packet.aoi
                delivered_count += 1
            # ## **** AoI MODIFICATION END **** ##
            
            # ## **** ENERGY MODIFICATION START: 累计能耗 **** ##
            if packet.status == "delivered" and hasattr(packet, 'energy_consumed') and packet.energy_consumed is not None:
                total_energy += packet.energy_consumed
            # ## **** ENERGY MODIFICATION END **** ##
            
            self.packet_status_snapshot.append({
                'id': packet.id,
                'source_id': packet.source_id,
                'destination_id': packet.destination_id,
                'current_holder_id': packet.current_holder_id,
                'status': packet.status,
                'current_hop_index': packet.current_hop_index,
                'retransmission_count': packet.retransmission_count,
                'actual_hops': list(packet.actual_hops),
                'per_hop_waits': list(packet.per_hop_waits),
                'event_history': list(packet.event_history),
                'path': list(packet.path) if hasattr(packet, 'path') else [],
                'concurrent_delay': getattr(packet, 'concurrent_delay', 0),
                'true_total_delay': true_total_delay,
                'total_delay': true_total_delay,
                'delivery_time': getattr(packet, 'delivery_time', None),  # 修正：写入送达时间
                'aoi': getattr(packet, 'aoi', None),  # 添加AoI字段
                'energy': getattr(packet, 'energy_consumed', None)  # 添加能耗字段
            })
        
        # ## **** AoI MODIFICATION START: 保存AoI统计结果 **** ##
        self.total_aoi = total_aoi
        self.average_aoi = total_aoi / delivered_count if delivered_count > 0 else 0.0
        # ## **** AoI MODIFICATION END **** ##
        
        # ## **** ENERGY MODIFICATION START: 保存能耗统计结果 **** ##
        self.total_energy = total_energy
        self.average_energy = total_energy / delivered_count if delivered_count > 0 else 0.0
        # ## **** ENERGY MODIFICATION END **** ##

    def _log(self, message):
        # 追加日志到 transmission_log，带上当前时间片
        if not hasattr(self, "transmission_log"):
            self.transmission_log = []
        self.transmission_log.append(f"[T={self.sim_time}] {message}")

    def _handle_new_collision(self, senders, receiver_id):
        # 检查CMTP协议树构建状态，如果在树构建阶段则不处理冲突
        if isinstance(self.routing_model, CMTPRoutingModel) and self.routing_model.tree_construction_started and not self.routing_model.tree_ready:
            # 在CMTP树构建阶段，不应该处理冲突，因为此时数据包应该被暂停发送
            print(f"⚠ 警告: CMTP树构建阶段检测到冲突，但应该被暂停发送。这可能是代码逻辑错误。")
            return

        # 简化碰撞处理日志
        receiver = self.uav_map.get(receiver_id, None)
        receiver_str = f"{receiver_id}" if receiver else f"{receiver_id}(无效)"
        sender_str = ", ".join([f"{s.id}" for s in senders])
        print(f"⚠ 碰撞: 接收者-{receiver_str}, 发送者-[{sender_str}]")
        
        import simulation_config
        use_ptp = getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False)
        if receiver_id not in self.collision_queues or not self.collision_queues[receiver_id]:
            self.collision_queues[receiver_id] = deque(s.id for s in senders)
        else:
            # 保持原有队列，补充新 sender
            for s in senders:
                if s.id not in self.collision_queues[receiver_id]:
                    self.collision_queues[receiver_id].append(s.id)
        for sender in senders:
            self.total_hop_attempts += 1
            packet = sender.tx_queue[0]
            # 统计并发延时（仅在未用PTP模型时）
            if not use_ptp:
                if not hasattr(packet, 'concurrent_delay'):
                    packet.concurrent_delay = 0
                packet.concurrent_delay += 1
                # 计算本跳并发惩罚（秒），并赋值到last_concurrent_penalty
                if self.routing_model is not None:
                    uav1 = sender
                    receiver = self.uav_map.get(receiver_id)
                    if receiver:
                        if isinstance(self.routing_model, DHyTPRoutingModel):
                            penalty = self.routing_model.cmtp.calculate_concurrent_region_delay(
                                (uav1.x, uav1.y), (receiver.x, receiver.y),
                                (uav1.x, uav1.y), (receiver.x, receiver.y))
                        else:
                            penalty = self.routing_model.calculate_concurrent_region_delay(
                                (uav1.x, uav1.y), (receiver.x, receiver.y),
                                (uav1.x, uav1.y), (receiver.x, receiver.y))
                        packet.last_concurrent_penalty = penalty
            packet.retransmission_count += 1
            # 新增：记录冲突事件（明确区分于一般重传）
            if hasattr(packet, 'add_event'):
                packet.add_event("collision", packet.current_holder_id, packet.current_hop_index, 
                               self.sim_time, f"冲突: {len(senders)}个发送者→接收者{receiver_id}")
            if packet.retransmission_count >= MAX_RETRANSMISSIONS:
                self._handle_terminal_failure(sender, packet, "Max_Retries(Collision_Init)")

    def _log_failure(self, packet, reason):
        """只记录失败日志，不改变状态（状态由外层函数处理）"""
        # 分类 reason
        if "Collision" in reason:
            info = "Collision"
            event_type = "collision"
        elif "Distance_Interference" in reason:
            info = "Distance_Interference"
            event_type = "distance_interference"
        elif "PRR" in reason:
            info = "PRR"
            event_type = "prr_failure"
        else:
            info = reason  # 或 "Other"
            event_type = f"fail_{reason.lower()}"
        
        # 检查是否已经在当前时间片记录了相同类型的失败，避免重复打印
        if hasattr(packet, 'last_failure_log') and packet.last_failure_log == (self.sim_time, info):
            # 已经记录过相同的失败，不重复打印
            return
            
        # 记录本次失败日志信息，用于后续去重
        packet.last_failure_log = (self.sim_time, info)
        
        # 简化失败日志
        print(f"✗ {packet.id}: 失败[{info}] {packet.current_holder_id}→{packet.get_next_hop_id()}")
        self._log(f"Pkt:{packet.id} ({packet.current_holder_id}->{packet.get_next_hop_id()}) FAIL! info:{info}")
        
        # 注意：不在这里记录事件，事件记录由调用者负责，避免重复记录

    def _handle_terminal_failure(self, sender, packet, reason):
        """处理终止性失败，直接从队列中移除包"""
        sender.tx_queue.popleft()
        packet.status = f"failed_{reason}"
        print(f"✗✗ {packet.id} 丢弃: {reason}")
        self._log(f"Pkt:{packet.id} DROPPED. Reason: {reason}.")

    # 添加方法用于统计能耗信息
    def collect_energy_statistics(self):
        """收集能耗统计信息"""
        from simulation_config import COLLECT_ENERGY_STATS, ROUTING_MODEL
        
        if not COLLECT_ENERGY_STATS:
            return {}
        
        # 收集所有已送达的数据包
        delivered_packets = []
        for uav in self.all_uavs:
            for packet in uav.tx_queue:
                if packet.status == "delivered":
                    delivered_packets.append(packet)
        
        # 从sim_manager中获取所有数据包
        if hasattr(self, 'sim_manager') and hasattr(self.sim_manager, 'packets_in_network'):
            for packet in self.sim_manager.packets_in_network:
                if packet.status == "delivered":
                    delivered_packets.append(packet)
        
        # 计算统计信息
        total_packets = len(delivered_packets)
        if total_packets == 0:
            return {
                "total_energy": 0,
                "avg_energy_per_packet": 0,
                "protocol": ROUTING_MODEL,
                "delivered_packets": 0
            }
        
        # ## **** ENERGY MODIFICATION START: 获取协议能耗 **** ##
        # 获取CMTP或DHYTP协议的能耗数据
        tree_creation_energy = 0.0
        tree_maintenance_energy = 0.0
        phase_transition_energy = 0.0
        maintenance_per_packet = 0.0
        
        if ROUTING_MODEL == "CMTP" and hasattr(self.routing_model, "accumulated_tree_creation_energy"):
            from simulation_config import PROTOCOL_ENERGY_CONFIG
            # 获取配置中的树创建能耗，而不是累积值
            tree_creation_energy = PROTOCOL_ENERGY_CONFIG["CMTP"]["TREE_CREATION"]
            # 维护能耗仍然使用累积值，并平均分摊
            tree_maintenance_energy = self.routing_model.accumulated_tree_maintenance_energy
            maintenance_per_packet = tree_maintenance_energy / total_packets if total_packets > 0 else 0
            
            print(f"⚡ CMTP树创建能耗(每包): {tree_creation_energy:.2f}J")
            print(f"⚡ 累计CMTP树维护能耗: {tree_maintenance_energy:.2f}J")
            print(f"⚡ 每包分摊的维护能耗: {maintenance_per_packet:.2f}J")
        
        elif ROUTING_MODEL == "DHYTP" and hasattr(self.routing_model, "accumulated_tree_creation_energy"):
            from simulation_config import PROTOCOL_ENERGY_CONFIG
            # 获取配置中的树创建能耗和阶段转换能耗，而不是累积值
            tree_creation_energy = PROTOCOL_ENERGY_CONFIG["DHYTP"]["TREE_CREATION"]
            phase_transition_energy = PROTOCOL_ENERGY_CONFIG["DHYTP"]["PHASE_TRANSITION"]
            # 维护能耗仍然使用累积值，并平均分摊
            tree_maintenance_energy = self.routing_model.accumulated_tree_maintenance_energy
            maintenance_per_packet = tree_maintenance_energy / total_packets if total_packets > 0 else 0
            
            print(f"⚡ DHYTP树创建能耗(每包): {tree_creation_energy:.2f}J")
            print(f"⚡ DHYTP阶段转换能耗(每包): {phase_transition_energy:.2f}J")
            print(f"⚡ 累计DHYTP树维护能耗: {tree_maintenance_energy:.2f}J")
            print(f"⚡ 每包分摊的维护能耗: {maintenance_per_packet:.2f}J")
        
        # 为每个数据包添加协议能耗
        for packet in delivered_packets:
            # 每个数据包都添加完整的树创建能耗和阶段转换能耗
            packet.protocol_energy += tree_creation_energy + phase_transition_energy
            # 维护能耗仍然按比例分摊
            packet.protocol_energy += maintenance_per_packet
            # 更新总能耗
            packet.energy_consumed += tree_creation_energy + phase_transition_energy + maintenance_per_packet
        # ## **** ENERGY MODIFICATION END **** ##
        
        # 计算基础统计
        total_energy = sum(p.energy_consumed for p in delivered_packets)
        avg_energy = total_energy / total_packets
        
        transmission_energy = sum(p.transmission_energy for p in delivered_packets)
        protocol_energy = sum(p.protocol_energy for p in delivered_packets)
        retransmission_energy = sum(p.retransmission_energy for p in delivered_packets)
        
        return {
            "total_energy": total_energy,
            "avg_energy_per_packet": avg_energy,
            "transmission_energy": transmission_energy,
            "protocol_energy": protocol_energy,
            "retransmission_energy": retransmission_energy,
            "protocol": ROUTING_MODEL,
            "delivered_packets": total_packets
        }

