# backend/core/uav.py
import random
from backend.simulation_config import MAX_X, MAX_Y, DEFAULT_UAV_COLOR  # 导入配置
from backend.models.mobility_model import RandomWaypointMobility  # 导入移动模型


class UAV:
    _id_counter = 0  # 用于生成唯一的UAV ID

    @classmethod
    def _generate_id(cls):
        cls._id_counter += 1
        return cls._id_counter

    @classmethod
    def reset_id_counter(cls):  # 在重置仿真时调用
        cls._id_counter = 0

    def __init__(self, x=None, y=None, z=None, color=None, mobility_profile="random_waypoint"):
        self.id = self._generate_id()

        self.x = x if x is not None else random.uniform(0, MAX_X)
        self.y = y if y is not None else random.uniform(0, MAX_Y)
        self.z = z if z is not None else random.uniform(0, 50)  # 简单给个高度

        self.x = self.prev_x
        self.y = self.prev_y
        self.z = self.prev_z

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.color = color if color is not None else DEFAULT_UAV_COLOR

        # 初始化移动模型
        if mobility_profile == "random_waypoint":
            self.mobility_model = RandomWaypointMobility(self)
        # else:
        # self.mobility_model = StationaryMobility(self) # 可以有其他模型

        # 未来可添加通信模块、路由协议模块实例等
        # self.communication_module = CommunicationModule(self)
        # self.routing_agent = RoutingAgent(self, protocol_type="MCA_OLSR")

    def update_state(self, time_increment):
        """更新无人机状态，例如移动，并计算速度"""
        self.prev_x, self.prev_y, self.prev_z = self.x, self.y, self.z  # 保存上一步位置

        self.mobility_model.move(time_increment)  # 移动模型更新 x, y, z

        # 未来: self.communication_module.update()
        # 未来: self.routing_agent.update()

        if time_increment > 0:  # 计算速度
            self.vx = (self.x - self.prev_x) / time_increment
            self.vy = (self.y - self.prev_y) / time_increment
            self.vz = (self.z - self.prev_z) / time_increment
        else:
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0

    def get_data_for_log(self):
        """返回用于数据库日志的数据"""
        return {
            "id": self.id,
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "z": round(self.z, 2),
            "vx": round(self.vx, 2),
            "vy": round(self.vy, 2),
            "vz": round(self.vz, 2),
            "color": self.color # color 也可以选择不存入日志，或存入单独的uav_info表
        }

    def get_data_for_api(self): # 原有的get_data可以改名为这个，或者保持并也包含速度
        """返回无人机数据，用于API实时状态传输"""
        data = self.get_data_for_log() # 包含id,x,y,z,vx,vy,vz,color
        return data

    def __repr__(self):
        return f"UAV(id={self.id}, pos=({self.x:.2f}, {self.y:.2f}, {self.z:.2f}), vel=({self.vx:.1f}, {self.vy:.1f}, {self.vz:.1f}))"