# 文件: backend/core/packet.py
# 描述: 定义数据包的结构和行为

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

    def get_next_hop_id(self):
        if self.path and 0 <= self.current_hop_index < len(self.path) - 1:
            return self.path[self.current_hop_index + 1]
        return None

    def advance_hop(self):
        self.current_hop_index += 1
        self.current_holder_id = self.path[self.current_hop_index]
        self.retransmission_count = 0
        if self.current_holder_id == self.destination_id:
            self.status = "delivered"