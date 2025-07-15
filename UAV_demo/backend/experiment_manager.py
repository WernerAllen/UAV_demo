# 文件: backend/experiment_manager.py (已更新)
# 描述: 重构批处理实验逻辑以支持多个并发的源-目标对

import threading
import time
import math
from simulation_manager import SimulationManager


def safe_float(val):
    try:
        if val is None:
            return 0.0
        if isinstance(val, str):
            return float(val) if val not in ("inf", "-inf", "nan", "Infinity", "-Infinity") else 0.0
        if math.isinf(val) or math.isnan(val):
            return 0.0
        return float(val)
    except Exception:
        return 0.0


class ExperimentManager:
    def __init__(self):
        self.sim_manager = SimulationManager()
        self.is_running = False
        self.experiment_thread = None
        self.simulation_lock = threading.Lock()

        # --- 实验状态重置 ---
        self.total_rounds_to_run = 0
        self.completed_rounds = 0
        self.total_time = 0
        self.average_time_per_round = 0.0
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
        self.total_time = 0
        self.average_time_per_round = 0.0
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
        MAX_STEPS_PER_ROUND = 20  # 最大步数保护，每步即为一个时间片
        valid_rounds = 0
        round_index = 0
        while valid_rounds < total_rounds:
            round_aborted = False  # 标记本轮是否异常终止
            round_index += 1
            self.current_status_message = f"正在运行第 {valid_rounds + 1}/{total_rounds} 轮..."
            print(f"[实验进度] 状态: {self.current_status_message}")
            print(f"[实验进度] 进度: {valid_rounds} / {total_rounds}")
            print(f"[实验进度] 总耗时: {self.total_time:.2f} 秒")
            if valid_rounds > 0:
                print(f"[实验进度] 平均耗时/轮: {self.average_time_per_round:.2f} 秒")
            else:
                print(f"[实验进度] 平均耗时/轮: 0.00 秒")
            print(f"正在运行第 {valid_rounds + 1}/{total_rounds} 轮... (实际第{round_index}次尝试)")
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
                    self.current_status_message = f"第 {valid_rounds + 1} 轮跳过: 未能为任何源-目标对找到有效路径。"
                    time.sleep(0.5)
                    continue

            # -- 运行仿真直到本轮结束 --
            round_start_time = time.time()
            step_count = 0
            while True:
                with self.simulation_lock:
                    if self._is_round_complete(packets_this_round):
                        break
                    self.sim_manager.step_simulation()
                step_count += 1
                if step_count > MAX_STEPS_PER_ROUND:
                    self.current_status_message += f"（本轮超出最大步数{MAX_STEPS_PER_ROUND}，强制终止，未计入统计）"
                    # 标记未送达包为失败
                    for pkt in packets_this_round:
                        if pkt.status != 'delivered':
                            pkt.status = 'failed_timeout'
                    round_aborted = True
                    break
                time.sleep(0.001)
            
            # -- 在一个原子锁内完成统计更新 --
            with self.simulation_lock:
                if not round_aborted:
                    # 只有正常轮次才统计
                    self.sim_manager.mac_layer.collect_final_packet_status(packets_this_round)
                    import math
                    delays = [pkt.get('total_delay', 0) for pkt in self.sim_manager.mac_layer.packet_status_snapshot]
                    # 过滤掉inf、nan和None
                    delays = [d for d in delays if d is not None and isinstance(d, (int, float)) and not (math.isinf(d) or math.isnan(d))]
                    if delays:
                        max_time_this_round = max(delays)
                        self.total_time += max_time_this_round
                        valid_rounds += 1
                        if valid_rounds > 0:
                            self.average_time_per_round = self.total_time / valid_rounds
                    else:
                        # 本轮无有效包，不计入统计
                        self.current_status_message += "（本轮无有效包，未计入统计）"
                else:
                    # 异常轮次不计入统计
                    pass
        self.current_status_message = f"实验完成！共执行 {valid_rounds} 轮。"
        self.is_running = False
        # 新增：实验结束后收集所有包的最终状态
        with self.simulation_lock:
            self.sim_manager.mac_layer.collect_final_packet_status(self.sim_manager.packets_in_network)
            
            # ## **** MODIFICATION START: 打印所有数据包的事件历史 **** ##
            print("\n" + "="*80)
            print("实验完成！所有数据包的事件历史：")
            print("="*80)
            
            for pkt in self.sim_manager.packets_in_network:
                print(f"\n数据包 {pkt.id} ({pkt.source_id} -> {pkt.destination_id}):")
                print(f"  状态: {pkt.status}")
                print(f"  实际路径: {pkt.actual_hops}")
                if hasattr(pkt, 'delivery_time') and pkt.delivery_time is not None:
                    print(f"  送达时间: {pkt.delivery_time:.2f}秒")
                print(f"  并发时延: {getattr(pkt, 'concurrent_delay', 0)} 时间片")
                
                if pkt.event_history:
                    print(f"  事件历史 ({len(pkt.event_history)} 个事件):")
                    for i, event in enumerate(pkt.event_history, 1):
                        print(f"    {i}. [{event['sim_time']:.2f}s] {event['event']} - {event['info']}")
                else:
                    print("  事件历史: 无")
            
            # 统计位置变动事件
            position_change_count = 0
            for pkt in self.sim_manager.packets_in_network:
                for event in pkt.event_history:
                    if event['event'] == 'position_change':
                        position_change_count += 1
            
            print(f"\n" + "="*80)
            print(f"位置变动事件统计: 共 {position_change_count} 次")
            print("="*80)
            # ## **** MODIFICATION END **** ##

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
                "total_time": safe_float(self.total_time),
                "average_time_per_round": safe_float(self.average_time_per_round),
                "message": self.current_status_message,
                "current_paths": self.current_round_paths, # 新增字段
                "final_paths": [
                    {
                        "id": pkt.id,
                        "source": pkt.source_id,
                        "destination": pkt.destination_id,
                        "actual_hops": list(pkt.actual_hops),
                        "delivery_time": safe_float(getattr(pkt, 'delivery_time', None))
                    }
                    for pkt in self.sim_manager.packets_in_network
                ] if not self.is_running else None,
                "all_actual_paths": self.all_round_actual_paths if not self.is_running else None  # 新增
            }
        return status
    # ## **** MODIFICATION END **** ##