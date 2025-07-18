# 文件: backend/mac_layer/mac.py
# 描述: 重构MAC层以实现基于队列的冲突解决机制

from collections import deque
from models.communication_model import CommunicationModel, RoutingModel
from simulation_config import MAX_RETRANSMISSIONS, POSITION_CHANGE_THRESHOLD
import math

class MACLayer:
    def __init__(self, all_uavs, sim_manager):
        self.all_uavs = all_uavs
        self.uav_map = {uav.id: uav for uav in all_uavs}
        self.comm_model = CommunicationModel()
        self.routing_model = RoutingModel(self.uav_map)  # 新增：初始化PTP模型

        # ## **** MODIFICATION START: 保存对sim_manager的引用 **** ##
        self.sim_manager = sim_manager 
        # ## **** MODIFICATION END **** ##

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

    def update_uav_list(self, all_uavs):
        self.all_uavs = all_uavs
        self.uav_map = {uav.id: uav for uav in all_uavs}
        if hasattr(self, 'routing_model'):
            self.routing_model.uav_map = self.uav_map  # 保持同步

    # ## **** REFACTORED: 实现基于队列的冲突解决 **** ##
    def process_transmissions(self, sim_time):
        print(f"--- 时间片 {float(sim_time)} ---")  # 控制台输出
        # self._log(f"--- 时间片 {float(sim_time)} ---")  # 写入日志
        self.sim_time = sim_time
        self.packet_status_snapshot.clear()
        import simulation_config
        self.use_ptp = getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False)

        # 1. 识别所有有数据要发的无人机
        potential_senders = {uav for uav in self.all_uavs if uav.tx_queue}
        if not potential_senders:
            return

        # 新增：所有队首包的当前跳等待+1
        for uav in potential_senders:
            packet = uav.tx_queue[0]
            if not hasattr(packet, 'concurrent_delay'):
                packet.concurrent_delay = 0  # 初始化并发延时
            if hasattr(packet, 'per_hop_waits') and packet.per_hop_waits:
                packet.per_hop_waits[-1] += 1
                # 新增：记录等待事件
                if hasattr(packet, 'add_event'):
                    packet.add_event("waiting", packet.current_holder_id, packet.current_hop_index, self.sim_time)

        # --- 距离干扰检测 ---
        # 统计本时间片所有接收节点及其对应的(发送者,包)
        receiver_to_senders = {}
        for sender in potential_senders:
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
                    print(f"!!! 发生距离干扰: receiver {receivers[i]} 和 {receivers[j]} 距离: {dist}")
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
                print(f"!!! 发生多对一冲突: receiver {receiver_id}，senders: {[s[0].id for s in senders]}")
                # print(f"!!! 发生多对一冲突: receiver {receiver_id}，senders: {[s.id for s in senders]}")
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
                    if hasattr(self.routing_model, 'are_vectors_concurrent') and self.routing_model.are_vectors_concurrent(
                        (uav1.x, uav1.y), (receiver.x, receiver.y),
                        (other_uav.x, other_uav.y), (other_receiver.x, other_receiver.y)):
                        is_concurrent = True
                        # 计算并发惩罚
                        penalty = self.routing_model.calculate_concurrent_region_delay(
                            (uav1.x, uav1.y), (receiver.x, receiver.y),
                            (other_uav.x, other_uav.y), (other_receiver.x, other_receiver.y))
                        break
                if is_concurrent:
                    # 1. 直闯并发区域的总时延
                    base_delay = self.routing_model.get_link_base_delay(uav1, receiver)
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

    def _reroute_and_select_best_path(self, sender, packet, next_hop_id):
        """
        路径修复：始终使用PTP协议（RoutingModel）进行路径重算。
        """
        import simulation_config
        # 强制使用PTP模型
        old_flag = getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False)
        simulation_config.USE_PTP_ROUTING_MODEL = True
        ptp_path_ids, _ = self.sim_manager.get_shortest_path(
            sender.id, next_hop_id
        )
        simulation_config.USE_PTP_ROUTING_MODEL = old_flag
        ptp_delay = None
        if ptp_path_ids and len(ptp_path_ids) > 1:
            ptp_delay = 0.0
            for i in range(len(ptp_path_ids) - 1):
                uav1 = self.uav_map.get(ptp_path_ids[i])
                uav2 = self.uav_map.get(ptp_path_ids[i+1])
                if uav1 and uav2:
                    ptp_delay += self.routing_model.get_link_base_delay(uav1, uav2)
                else:
                    ptp_delay = float('inf')
                    break
        new_path = ptp_path_ids
        if new_path and len(new_path) > 1:
            original_path = packet.path
            current_hop_idx = packet.current_hop_index
            new_full_path = original_path[:current_hop_idx] + new_path + original_path[current_hop_idx + 2:]
            packet.path = new_full_path
            packet.add_event("reroute_success", sender.id, current_hop_idx, self.sim_time, f"New path via {new_path[1]}")
            for i in range(current_hop_idx, len(new_full_path) - 1):
                next_hop_id2 = new_full_path[i + 1]
                next_hop_uav = self.uav_map.get(next_hop_id2)
                if next_hop_uav:
                    packet.record_next_hop_position(next_hop_id2, next_hop_uav.x, next_hop_uav.y, next_hop_uav.z)
            self._log(f"Pkt:{packet.id} Use PTP path: {new_path} (delay={ptp_delay:.2f}) [PositionChange, PTPOnly]")
        else:
            self._log(f"Pkt:{packet.id} Reroute FAILED. No alternative path found. [PositionChange, PTPOnly]")

    def _attempt_transmission(self, sender, receiver, all_transmitters):
        """执行单次传输尝试的所有检查，返回(is_successful, reason)"""
        packet = sender.tx_queue[0]
        if receiver is None:
            self._log_failure(packet, "Receiver_None")
            return False, "Receiver_None"

        # === PTP链路估算（用当前sender/receiver位置） ===
        if hasattr(self, "routing_model") and self.routing_model is not None:
            link_delay = self.routing_model.get_link_base_delay(sender, receiver)
            self._log(f"Pkt:{packet.id} PTP link delay: {link_delay:.2f}")
        # === END PTP链路估算 ===

        # ## **** MODIFICATION START: 添加位置变动检查 **** ##
        # 检查下一跳节点位置是否有变动
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
                self._reroute_and_select_best_path(sender, packet, next_hop_id)
                # 修复后，重新获取新路径的下一跳和receiver
                next_hop_id = packet.get_next_hop_id()
                receiver = self.uav_map.get(next_hop_id)
                # 注意：如果修复失败，receiver可能为None，需处理
                if receiver is None:
                    self._log_failure(packet, "Receiver_None_AfterReroute")
                    return False, "Receiver_None_AfterReroute"
        # ## **** MODIFICATION END **** ##
        # 后续发送流程继续

        # ## **** MODIFICATION START: PRR丢包判定开关 **** ##
        import simulation_config
        if getattr(simulation_config, 'USE_PRR_FAILURE_MODEL', True):
            if self.comm_model.check_prr_failure(receiver):
                self._log_failure(packet, "PRR(Packet_Loss)")
                return False, "PRR"
        # ## **** MODIFICATION END **** ##
            
        return True, None

    def _log(self, message):
        # 追加日志到 transmission_log，带上当前时间片
        if not hasattr(self, "transmission_log"):
            self.transmission_log = []
        self.transmission_log.append(f"[T={self.sim_time}] {message}")

    def _handle_new_collision(self, senders, receiver_id):
        print(f"调用_handle_new_collision, receiver_id={receiver_id}, senders={[s.id for s in senders]}")
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
                if hasattr(self, 'routing_model'):
                    uav1 = sender
                    receiver = self.uav_map.get(receiver_id)
                    if receiver:
                        penalty = self.routing_model.calculate_concurrent_region_delay(
                            (uav1.x, uav1.y), (receiver.x, receiver.y),
                            (uav1.x, uav1.y), (receiver.x, receiver.y))
                        packet.last_concurrent_penalty = penalty
            packet.retransmission_count += 1
            # 新增：记录重传事件
            if hasattr(packet, 'add_event'):
                packet.add_event("retransmit", packet.current_holder_id, packet.current_hop_index, self.sim_time, "collision")
            if packet.retransmission_count >= MAX_RETRANSMISSIONS:
                self._handle_terminal_failure(sender, packet, "Max_Retries(Collision_Init)")

    def _log_failure(self, packet, reason):
        """只记录失败日志，不改变状态（状态由外层函数处理）"""
        # 分类 reason
        if "Collision" in reason:
            info = "Collision"
        elif "Distance_Interference" in reason:
            info = "Distance_Interference"
        elif "PRR" in reason:
            info = "PRR"
        else:
            info = reason  # 或 "Other"
        self._log(f"Pkt:{packet.id} ({packet.source_id}->{packet.get_next_hop_id()}) FAIL! info:{info}")

    def _handle_terminal_failure(self, sender, packet, reason):
        packet.status = f"failed_{reason}"
        if sender.tx_queue and sender.tx_queue[0].id == packet.id:
            sender.tx_queue.popleft()
        self._log(f"Pkt:{packet.id} DROPPED. Reason: {reason}.")

    def _handle_failure(self, sender, packet, reason):
        """处理非冲突导致的失败"""
        import simulation_config
        # ## **** MODIFICATION START: 实现本地路径修复逻辑 **** ##
        # if reason == "Mobility_Failure":
        #     if not getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False):
        #         self._log(f"Pkt:{packet.id} Mobility Failure, but reroute disabled. Will retry or drop.")
        #         # 不做路径修复，直接进入重传/丢包逻辑
        #     else:
        #         self._log(f"Pkt:{packet.id} Mobility Failure from {sender.id} to {packet.get_next_hop_id()}. Attempting local reroute...")
        #         # 尝试分别用传统模型和PTP模型计算新路径
        #         # 1. 传统模型
        #         orig_path_ids, _ = self.sim_manager.get_shortest_path(
        #             sender.id, packet.get_next_hop_id()
        #         )
        #         orig_delay = None
        #         if orig_path_ids and len(orig_path_ids) > 1:
        #             orig_delay = 0.0
        #             for i in range(len(orig_path_ids) - 1):
        #                 uav1 = self.uav_map.get(orig_path_ids[i])
        #                 uav2 = self.uav_map.get(orig_path_ids[i+1])
        #                 if uav1 and uav2:
        #                     # 传统模型用欧氏距离
        #                     orig_delay += math.sqrt((uav1.x - uav2.x)**2 + (uav1.y - uav2.y)**2 + (uav1.z - uav2.z)**2)
        #                 else:
        #                     orig_delay = float('inf')
        #                     break
        #         # 2. PTP模型
        #         # 临时切换全局变量
        #         old_flag = getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False)
        #         simulation_config.USE_PTP_ROUTING_MODEL = True
        #         ptp_path_ids, _ = self.sim_manager.get_shortest_path(
        #             sender.id, packet.get_next_hop_id()
        #         )
        #         simulation_config.USE_PTP_ROUTING_MODEL = old_flag
        #         ptp_delay = None
        #         if ptp_path_ids and len(ptp_path_ids) > 1:
        #             ptp_delay = 0.0
        #             for i in range(len(ptp_path_ids) - 1):
        #                 uav1 = self.uav_map.get(ptp_path_ids[i])
        #                 uav2 = self.uav_map.get(ptp_path_ids[i+1])
        #                 if uav1 and uav2:
        #                     # PTP模型用routing_model
        #                     ptp_delay += self.routing_model.get_link_base_delay(uav1, uav2)
        #                 else:
        #                     ptp_delay = float('inf')
        #                     break
        #         # 3. 选择更优路径
        #         if ptp_delay is not None and orig_delay is not None and ptp_delay < orig_delay:
        #             new_path = ptp_path_ids
        #             self._log(f"Pkt:{packet.id} Use PTP path: {new_path} (delay={ptp_delay:.2f})")
        #         else:
        #             new_path = orig_path_ids
        #             self._log(f"Pkt:{packet.id} Use original path: {new_path} (delay={orig_delay:.2f})")
        #         # 4. 焊接新路径
        #         if new_path and len(new_path) > 1:
        #             original_path = packet.path
        #             current_hop_idx = packet.current_hop_index
        #             # 新路径 = 原路径在当前节点之前的部分 + 新路径 + 原路径在目标节点之后的部分
        #             new_full_path = original_path[:current_hop_idx] + new_path + original_path[current_hop_idx + 2:]
        #             packet.path = new_full_path
        #             packet.add_event("reroute_success", sender.id, current_hop_idx, self.sim_time, f"New path via {new_path[1]}")
        #             # 记录新路径中每个下一跳节点的位置
        #             for i in range(current_hop_idx, len(new_full_path) - 1):
        #                 next_hop_id = new_full_path[i + 1]
        #                 next_hop_uav = self.uav_map.get(next_hop_id)
        #                 if next_hop_uav:
        #                     packet.record_next_hop_position(next_hop_id, next_hop_uav.x, next_hop_uav.y, next_hop_uav.z)
        #             return
        #         else:
        #             self._log(f"Pkt:{packet.id} Reroute FAILED. No alternative path found. Will retry.")
        #             # (接下来会执行下面的常规重传逻辑)
        # ## **** MODIFICATION END **** ##

        # 对于其他失败原因或绕行失败的情况，执行常规重传
        packet.retransmission_count += 1
        self._log_failure(packet, reason)
        # 新增：记录重传事件
        if hasattr(packet, 'add_event'):
            packet.add_event("retransmit", packet.current_holder_id, packet.current_hop_index, self.sim_time, reason)
        # 保持包在队首等待下次重传

        # 检查是否达到最大重传次数 (使用您提到的参数)
        if packet.retransmission_count >= MAX_RETRANSMISSIONS:
            self._handle_terminal_failure(sender, packet, f"Max_Retries({reason})")


    def _handle_success(self, sender, receiver, packet):
        """处理成功的传输"""
        self._log(f"Pkt:{packet.id} ({sender.id}->{receiver.id}) OK.")
        
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
        packet.advance_hop(self.sim_time)
        packet.actual_hops.append(packet.current_holder_id)
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

        if packet.status != "delivered":
            receiver.add_packet_to_queue(packet)
        else:
            self._log(f"Pkt:{packet.id} DELIVERED to final destination {receiver.id}!")
            if hasattr(packet, 'add_event'):
                packet.add_event("delivered", packet.current_holder_id, packet.current_hop_index, self.sim_time)

    def _handle_new_distance_interference(self, senders, receiver_id):
        """
        处理新发生的距离干扰，将所有受影响的发送者加入该接收者的距离干扰队列。
        该队列与多对一冲突队列互不影响，独立顺序重传。
        """
        print(f"调用_handle_new_distance_interference, receiver_id={receiver_id}, senders={[s.id for s in senders]}")
        import simulation_config
        use_ptp = getattr(simulation_config, 'USE_PTP_ROUTING_MODEL', False)
        if receiver_id not in self.distance_interference_queues or not self.distance_interference_queues[receiver_id]:
            self.distance_interference_queues[receiver_id] = deque(s.id for s in senders)
        else:
            for s in senders:
                if s.id not in self.distance_interference_queues[receiver_id]:
                    self.distance_interference_queues[receiver_id].append(s.id)
        for sender in senders:
            self.total_hop_attempts += 1
            packet = sender.tx_queue[0]
            # 统计并发延时（仅在未用PTP模型时）
            if not use_ptp:
                if not hasattr(packet, 'concurrent_delay'):
                    packet.concurrent_delay = 0
                packet.concurrent_delay += 1
                # 计算本跳并发惩罚（秒），并赋值到last_concurrent_penalty
                if hasattr(self, 'routing_model'):
                    uav1 = sender
                    receiver = self.uav_map.get(receiver_id)
                    if receiver:
                        penalty = self.routing_model.calculate_concurrent_region_delay(
                            (uav1.x, uav1.y), (receiver.x, receiver.y),
                            (uav1.x, uav1.y), (receiver.x, receiver.y))
                        packet.last_concurrent_penalty = penalty
            packet.retransmission_count += 1
            # 新增：记录距离干扰重传事件
            if hasattr(packet, 'add_event'):
                packet.add_event("retransmit", packet.current_holder_id, packet.current_hop_index, self.sim_time, "distance_interference")
            # 若重传次数超限则丢弃
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
        for packet in all_packets:
            true_total_delay = getattr(packet, 'true_total_delay', 0.0)
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
                'delivery_time': getattr(packet, 'delivery_time', None)  # 修正：写入送达时间
            })

