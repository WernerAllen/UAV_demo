# 文件: backend/mac_layer/mac.py
# 描述: 重构MAC层以实现基于队列的冲突解决机制

from collections import deque
from models.communication_model import CommunicationModel
from simulation_config import MAX_RETRANSMISSIONS

class MACLayer:
    def __init__(self, all_uavs):
        self.all_uavs = all_uavs
        self.uav_map = {uav.id: uav for uav in all_uavs}
        self.comm_model = CommunicationModel()
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

    # ## **** REFACTORED: 实现基于队列的冲突解决 **** ##
    def process_transmissions(self, sim_time):
        self.sim_time = sim_time
        self.packet_status_snapshot.clear()

        # 1. 识别所有有数据要发的无人机
        potential_senders = {uav for uav in self.all_uavs if uav.tx_queue}
        if not potential_senders:
            return

        # 新增：所有队首包的当前跳等待+1
        for uav in potential_senders:
            packet = uav.tx_queue[0]
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
        # 记录所有受距离干扰影响的(发送者,包)
        interfered_senders = set()
        for rid in interfered_receivers:
            for sender, packet in receiver_to_senders[rid]:
                interfered_senders.add((sender, packet))
        # --- 多对一冲突检测 ---
        transmitters_this_step = set()
        collision_groups = []
        for receiver_id, senders in receiver_to_senders.items():
            if len(senders) > 1:
                collision_groups.append((receiver_id, senders))
            else:
                transmitters_this_step.add(senders[0][0])
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

    def _attempt_transmission(self, sender, receiver, all_transmitters):
        """执行单次传输尝试的所有检查，返回(is_successful, reason)"""
        packet = sender.tx_queue[0]
        if receiver is None:
            self._log_failure(packet, "Receiver_None")
            return False, "Receiver_None"
        if self.comm_model.check_prr_failure(receiver):
            self._log_failure(packet, "PRR(Packet_Loss)")
            return False, "PRR"
        return True, None

    def _log(self, message):
        pass

    def _handle_new_collision(self, senders, receiver_id):
        """处理新发生的冲突，将发送者加入队列"""
        self._log(f"NEW COLLISION at receiver {receiver_id}! Senders: {[s.id for s in senders]}")
        self.collision_queues[receiver_id] = deque(s.id for s in senders)
        for sender in senders:
            self.total_hop_attempts += 1
            packet = sender.tx_queue[0]
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
        # 重传次数上限取消，不再丢弃包
        self._log_failure(packet, reason)
        # 新增：记录重传事件
        if hasattr(packet, 'add_event'):
            packet.add_event("retransmit", packet.current_holder_id, packet.current_hop_index, self.sim_time, reason)
        # 保持包在队首等待下次重传

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
        packet.advance_hop()
        packet.actual_hops.append(packet.current_holder_id)
        if hasattr(packet, 'per_hop_waits'):
            packet.per_hop_waits.append(0)
        if packet.status != "delivered":
            receiver.add_packet_to_queue(packet)
        else:
            self._log(f"Pkt:{packet.id} DELIVERED to final destination {receiver.id}!")
            # 新增：记录送达事件
            if hasattr(packet, 'add_event'):
                packet.add_event("delivered", packet.current_holder_id, packet.current_hop_index, self.sim_time)

    def _handle_new_distance_interference(self, senders, receiver_id):
        """
        处理新发生的距离干扰，将所有受影响的发送者加入该接收者的距离干扰队列。
        该队列与多对一冲突队列互不影响，独立顺序重传。
        """
        self._log(f"NEW DISTANCE INTERFERENCE at receiver {receiver_id}! Senders: {[s.id for s in senders]}")
        # 用新的队列覆盖旧队列，保证本轮所有受干扰发送者都入队
        self.distance_interference_queues[receiver_id] = deque(s.id for s in senders)
        for sender in senders:
            self.total_hop_attempts += 1
            packet = sender.tx_queue[0]
            packet.retransmission_count += 1
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
                    queue.popleft()
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
                    'path': list(packet.path) if hasattr(packet, 'path') else []  # 新增：完整路径
                })

    def collect_final_packet_status(self, all_packets):
        """
        收集所有包的最终状态（包括已送达/失败的包），用于实验结束后日志展示
        """
        self.packet_status_snapshot.clear()
        for packet in all_packets:
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
                'path': list(packet.path) if hasattr(packet, 'path') else []  # 新增：完整路径
            })

