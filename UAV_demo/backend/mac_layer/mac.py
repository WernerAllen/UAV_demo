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
        self.transmission_log = []
        self.total_hop_attempts = 0
        self.sim_time = 0
        
        # ## **** NEW FEATURE: 冲突队列 **** ##
        # 字典，键为接收者ID，值为一个发送者ID的队列(deque)
        self.collision_queues = {}

    def reset_counters(self):
        self.total_hop_attempts = 0
        self.transmission_log.clear()
        self.collision_queues.clear() # 重置时清空冲突队列

    def update_uav_list(self, all_uavs):
        self.all_uavs = all_uavs
        self.uav_map = {uav.id: uav for uav in all_uavs}

    # ## **** REFACTORED: 实现基于队列的冲突解决 **** ##
    def process_transmissions(self, sim_time):
        self.sim_time = sim_time
        self.transmission_log.clear()

        # 1. 识别所有有数据要发的无人机
        potential_senders = {uav for uav in self.all_uavs if uav.tx_queue}
        if not potential_senders:
            return

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
        # 找出所有本轮有接收包的接收节点
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
            # 合并所有相关节点进 collision_queues
            all_conflict = interfered_senders | collision_senders
            for receiver_id, senders in receiver_to_senders.items():
                if any((s, p) in all_conflict for s, p in senders):
                    self._handle_new_collision([s for s, p in senders], receiver_id)
        else:
            # 分别处理
            for receiver_id, senders in collision_groups:
                self._handle_new_collision([s for s, p in senders], receiver_id)
            for sender, packet in interfered_senders:
                # 距离干扰单独处理，直接重传
                self._log(f"DISTANCE INTERFERENCE at receiver {packet.get_next_hop_id()}! Sender: {sender.id}")
                self._handle_failure(sender, packet, "Distance_Interference")
        # 重新识别本轮可正常发送的发送者
        handled_senders = {s for s, p in interfered_senders} | {s for s, p in collision_senders}
        unhandled_senders = potential_senders - handled_senders
        # 其余发送者正常尝试发送
        for sender in unhandled_senders:
            self.total_hop_attempts += 1
            packet = sender.tx_queue[0]
            receiver_id = packet.get_next_hop_id()
            receiver = self.uav_map.get(receiver_id)
            is_successful = self._attempt_transmission(sender, receiver, list(potential_senders))
            if is_successful:
                self._handle_success(sender, receiver, packet)
            else:
                self._handle_failure(sender, packet, "Transmission_Error")

    def _attempt_transmission(self, sender, receiver, all_transmitters):
        """执行单次传输尝试的所有检查，返回True或False"""
        packet = sender.tx_queue[0]
        # 只保留PRR失败重传，其它如Mobility、SINR等相关代码全部删除
        if receiver is None:
            self._log_failure(packet, "Receiver_None")
            return False
        if self.comm_model.check_prr_failure(receiver):
            self._log_failure(packet, "PRR(Packet_Loss)")
            return False
        return True

    def _log(self, message):
        self.transmission_log.append(f"T={self.sim_time:.1f}s: {message}")

    def _handle_new_collision(self, senders, receiver_id):
        """处理新发生的冲突，将发送者加入队列"""
        self._log(f"NEW COLLISION at receiver {receiver_id}! Senders: {[s.id for s in senders]}")
        # 为这个接收者创建一个新的冲突队列
        self.collision_queues[receiver_id] = deque(s.id for s in senders)
        for sender in senders:
            self.total_hop_attempts += 1
            packet = sender.tx_queue[0]
            packet.retransmission_count += 1
            if packet.retransmission_count >= MAX_RETRANSMISSIONS:
                self._handle_terminal_failure(sender, packet, "Max_Retries(Collision_Init)")

    def _log_failure(self, packet, reason):
        """只记录失败日志，不改变状态（状态由外层函数处理）"""
        self._log(f"Pkt:{packet.id} ({packet.source_id}->{packet.get_next_hop_id()}) FAIL! Reason: {reason}.")

    def _handle_terminal_failure(self, sender, packet, reason):
        packet.status = f"failed_{reason}"
        if sender.tx_queue and sender.tx_queue[0].id == packet.id:
            sender.tx_queue.popleft()
        self._log(f"Pkt:{packet.id} DROPPED. Reason: {reason}.")

    def _handle_failure(self, sender, packet, reason):
        """处理非冲突导致的失败"""
        # 重传次数上限取消，不再丢弃包
        self._log_failure(packet, reason)
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
        packet.advance_hop()
        if packet.status != "delivered":
            receiver.add_packet_to_queue(packet)
        else:
            self._log(f"Pkt:{packet.id} DELIVERED to final destination {receiver.id}!")

