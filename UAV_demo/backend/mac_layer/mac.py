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

        transmitters_this_step = set()
        
        # 2. 优先处理已在冲突队列中的发送者
        # 对于每个冲突的接收者，只允许其队列头的发送者尝试发送
        for receiver_id, sender_queue in self.collision_queues.items():
            if sender_queue:
                head_sender_id = sender_queue[0]
                head_sender = self.uav_map.get(head_sender_id)
                if head_sender and head_sender in potential_senders:
                    transmitters_this_step.add(head_sender)
        
        # 3. 处理新的传输请求，并检测新的冲突
        # 将尚未处理的发送者按接收者分组
        unhandled_senders = potential_senders - transmitters_this_step
        receivers_map = {}
        for sender in unhandled_senders:
            next_hop_id = sender.tx_queue[0].get_next_hop_id()
            if next_hop_id:
                if next_hop_id not in receivers_map:
                    receivers_map[next_hop_id] = []
                receivers_map[next_hop_id].append(sender)

        for receiver_id, senders in receivers_map.items():
            if len(senders) > 1:
                # 新的冲突发生！将这些发送者加入该接收者的冲突队列
                self._handle_new_collision(senders, receiver_id)
            elif len(senders) == 1:
                # 没有新冲突，此发送者可以尝试发送
                transmitters_this_step.add(senders[0])

        # 4. 对本时间片最终确定的所有发送者进行传输处理
        for sender in transmitters_this_step:
            self.total_hop_attempts += 1
            packet = sender.tx_queue[0]
            receiver_id = packet.get_next_hop_id()
            receiver = self.uav_map.get(receiver_id)

            # 检查传输是否成功
            is_successful = self._attempt_transmission(sender, receiver, list(transmitters_this_step))
            
            # 根据结果更新状态
            if is_successful:
                self._handle_success(sender, receiver, packet)
            else:
                self._handle_failure(sender, packet, "Transmission_Error") # 失败原因在内部日志记录

    def _attempt_transmission(self, sender, receiver, all_transmitters):
        """执行单次传输尝试的所有检查，返回True或False"""
        packet = sender.tx_queue[0]
        if not receiver:
            self._log_failure(packet, "Mobility(Receiver_Gone)")
            return False
        if self.comm_model.check_mobility_failure(sender, receiver):
            self._log_failure(packet, "Mobility(Out_of_Range)")
            return False
        
        interferers = [u for u in all_transmitters if u.id != sender.id]
        if self.comm_model.check_interference_failure(sender, receiver, interferers):
            self._log_failure(packet, "Interference(SINR)")
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
        packet.retransmission_count += 1
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
        packet.advance_hop()
        if packet.status != "delivered":
            receiver.add_packet_to_queue(packet)
        else:
            self._log(f"Pkt:{packet.id} DELIVERED to final destination {receiver.id}!")

