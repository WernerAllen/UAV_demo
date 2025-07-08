# 文件: backend/experiment_manager.py (已更新)
# 描述: 重构批处理实验逻辑以支持多个并发的源-目标对

import threading
import time
from simulation_manager import SimulationManager


class ExperimentManager:
    def __init__(self):
        self.sim_manager = SimulationManager()
        self.is_running = False
        self.experiment_thread = None
        self.simulation_lock = threading.Lock()

        # --- 实验状态重置 ---
        self.total_rounds_to_run = 0
        self.completed_rounds = 0
        self.total_hop_count = 0
        self.average_hops_per_round = 0.0
        self.current_status_message = "空闲"
        self.current_round_paths = [] # 存储当前轮次所有S-D对的路径
        self.all_round_actual_paths = []  # 新增：所有轮的实际路径

    # ## **** MODIFICATION START: 改变接口以接收S-D对列表 **** ##
    def start_experiment(self, total_rounds, pairs_data, num_uavs):
        if self.is_running:
            return False, "另一个实验正在进行中。"
        if not pairs_data:
            return False, "必须提供源-目标对才能开始实验。"

        self.is_running = True
        self.total_rounds_to_run = total_rounds
        self.completed_rounds = 0
        self.total_hop_count = 0
        self.average_hops_per_round = 0.0
        self.current_status_message = "实验已开始..."
        self.current_round_paths = []

        # 提取源-目标对，忽略初始路径
        sd_pairs = [{"source": p["source"], "destination": p["destination"]} for p in pairs_data]

        self.experiment_thread = threading.Thread(
            target=self._run_experiment_logic,
            args=(total_rounds, sd_pairs, num_uavs)
        )
        self.experiment_thread.daemon = True
        self.experiment_thread.start()

        return True, "多路径批处理实验已在后台启动。"
    # ## **** MODIFICATION END **** ##

    # ## **** MODIFICATION START: 重构实验核心逻辑 **** ##
    def _run_experiment_logic(self, total_rounds, sd_pairs, num_uavs):
        self.all_round_actual_paths = []  # 每次实验重置
        for i in range(total_rounds):
            self.current_status_message = f"正在运行第 {i + 1}/{total_rounds} 轮..."
            packets_this_round = []
            
            # -- 在一个原子锁内完成一轮的准备工作 --
            with self.simulation_lock:
                self.sim_manager.start_simulation(num_uavs=num_uavs) # 每轮开始时重置无人机位置
                self.sim_manager.mac_layer.reset_counters()
                
                # 为当前轮次的所有S-D对寻找新路径并创建数据包
                current_paths_this_round = []
                for pair in sd_pairs:
                    source_id, dest_id = pair["source"], pair["destination"]
                    
                    # 重新寻找路径，因为UAV位置已重置
                    path_ids, _ = self.sim_manager.get_shortest_path(source_id, dest_id)
                    
                    if path_ids:
                        current_paths_this_round.append({
                            "source": source_id,
                            "destination": dest_id,
                            "path": path_ids
                        })
                        # 每个S-D对只发送一个数据包
                        created_packets, _ = self.sim_manager.initiate_data_transfer(
                            source_id, dest_id, packet_count=1
                        )
                        if created_packets:
                            packets_this_round.extend(created_packets)

                # 更新状态，让前端可以查询到本轮的路径
                self.current_round_paths = current_paths_this_round

                if not packets_this_round:
                    self.current_status_message = f"第 {i + 1} 轮跳过: 未能为任何源-目标对找到有效路径。"
                    self.completed_rounds += 1
                    time.sleep(0.5)
                    continue

            # -- 运行仿真直到本轮结束 --
            round_start_time = time.time()
            while True:
                with self.simulation_lock:
                    if self._is_round_complete(packets_this_round):
                        break
                    self.sim_manager.step_simulation()

                time.sleep(0.001) 
                if time.time() - round_start_time > 60: # 超时保护
                    self.current_status_message = f"第 {i + 1} 轮超时。"
                    break
            
            # -- 在一个原子锁内完成统计更新 --
            with self.simulation_lock:
                # 新统计逻辑：每个包的总跳数为sum(p.per_hop_waits)，每轮取最大值
                max_hops_this_round = 0
                for p in packets_this_round:
                    hops = sum(getattr(p, 'per_hop_waits', []))
                    if hops > max_hops_this_round:
                        max_hops_this_round = hops
                self.total_hop_count += max_hops_this_round
                self.completed_rounds += 1
                if self.completed_rounds > 0:
                    self.average_hops_per_round = self.total_hop_count / self.completed_rounds
                # 新增：记录本轮所有包的实际路径
                self.all_round_actual_paths.append([
                    {
                        "id": p.id,
                        "source": p.source_id,
                        "destination": p.destination_id,
                        "actual_hops": list(p.actual_hops)
                    }
                    for p in packets_this_round
                ])
        
        self.current_status_message = f"实验完成！共执行 {self.completed_rounds} 轮。"
        self.is_running = False
        # 新增：实验结束后收集所有包的最终状态
        with self.simulation_lock:
            self.sim_manager.mac_layer.collect_final_packet_status(self.sim_manager.packets_in_network)

    def _is_round_complete(self, packets_to_check):
        """
        一轮被认为完成的条件是：所有为该轮创建的数据包都已送达或失败。
        根据需求，失败情况不会发生，所以主要是检查是否都'delivered'。
        """
        if not packets_to_check: return True
        return all(p.status == "delivered" or p.status.startswith("failed") for p in packets_to_check)

    # ## **** MODIFICATION START: 在状态中增加当前路径信息 **** ##
    def get_status(self):
        with self.simulation_lock:
            status = {
                "is_running": self.is_running,
                "total_rounds_to_run": self.total_rounds_to_run,
                "completed_rounds": self.completed_rounds,
                "total_hop_count": self.total_hop_count,
                "average_hops_per_round": self.average_hops_per_round,
                "message": self.current_status_message,
                "current_paths": self.current_round_paths, # 新增字段
                "final_paths": [
                    {
                        "id": pkt.id,
                        "source": pkt.source_id,
                        "destination": pkt.destination_id,
                        "actual_hops": list(pkt.actual_hops)
                    }
                    for pkt in self.sim_manager.packets_in_network
                ] if not self.is_running else None,
                "all_actual_paths": self.all_round_actual_paths if not self.is_running else None  # 新增
            }
        return status
    # ## **** MODIFICATION END **** ##