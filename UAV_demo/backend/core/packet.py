# 文件: backend/core/packet.py
# 描述: 定义数据包的结构和行为

from simulation_config import POSITION_CHANGE_THRESHOLD

class Packet:
    _id_counter = 0

    @classmethod
    def _generate_id(cls):
        cls._id_counter += 1
        return f"pkt_{cls._id_counter}"

    @classmethod
    def reset_id_counter(cls):
        cls._id_counter = 0

    def __init__(self, source_id, destination_id, creation_time):
        self.id = self._generate_id()
        self.source_id = source_id
        self.destination_id = destination_id
        self.creation_time = creation_time
        self.path = []
        self.current_hop_index = 0
        self.current_holder_id = source_id
        self.status = "pending"  # pending, in_transit, delivered, failed_...
        self.retransmission_count = 0
        self.actual_hops = [source_id]
        self.per_hop_waits = [0]
        self.event_history = []  # 新增：事件历史
        self.delivery_time = None  # 新增：送达耗时
        # ## **** MODIFICATION START: 添加下一跳位置记录 **** ##
        self.next_hop_positions = {}  # 记录每个下一跳节点的位置 {hop_id: (x, y, z)}
        # ## **** MODIFICATION END **** ##

    def get_next_hop_id(self):
        if self.path and 0 <= self.current_hop_index < len(self.path) - 1:
            return self.path[self.current_hop_index + 1]
        return None

    # ## **** MODIFICATION START: 添加位置记录和检查方法 **** ##
    def record_next_hop_position(self, hop_id, x, y, z):
        """记录下一跳节点的位置"""
        self.next_hop_positions[hop_id] = (x, y, z)
    
    def check_next_hop_position_change(self, hop_id, current_x, current_y, current_z, threshold=POSITION_CHANGE_THRESHOLD):
        """检查下一跳节点位置是否有变动"""
        if hop_id not in self.next_hop_positions:
            # 首次记录位置
            self.record_next_hop_position(hop_id, current_x, current_y, current_z)
            return False, 0.0
        
        recorded_x, recorded_y, recorded_z = self.next_hop_positions[hop_id]
        distance_change = ((current_x - recorded_x)**2 + 
                          (current_y - recorded_y)**2 + 
                          (current_z - recorded_z)**2)**0.5
        
        if distance_change > threshold:
            # 更新记录的位置
            self.record_next_hop_position(hop_id, current_x, current_y, current_z)
            return True, distance_change
        
        return False, distance_change
    # ## **** MODIFICATION END **** ##

    def advance_hop(self, sim_time=None):
        self.current_hop_index += 1
        self.current_holder_id = self.path[self.current_hop_index]
        self.retransmission_count = 0
        if self.current_holder_id == self.destination_id:
            self.status = "delivered"
            if sim_time is not None:
                self.delivery_time = sim_time - self.creation_time

    def add_event(self, event_type, holder_id, hop_index, sim_time, extra_info=""):
        self.event_history.append({
            "event": event_type,
            "holder": holder_id,
            "hop": hop_index,
            "sim_time": sim_time,
            "info": extra_info
        })