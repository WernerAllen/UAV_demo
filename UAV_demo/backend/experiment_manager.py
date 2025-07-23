# 文件: backend/experiment_manager.py (已更新)
# 描述: 重构批处理实验逻辑以支持多个并发的源-目标对

import threading
import time
import math
from simulation_manager import SimulationManager
from simulation_config import USE_CTMP_ROUTING_MODEL


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
        # ## **** AoI MODIFICATION START: 添加AoI相关统计变量 **** ##
        self.total_aoi = 0.0
        self.average_aoi = 0.0
        # ## **** AoI MODIFICATION END **** ##

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
        self.total_delivery_time = 0.0
        self.average_delivery_time = 0.0
        # ## **** AoI MODIFICATION START: 重置AoI统计 **** ##
        self.total_aoi = 0.0
        self.average_aoi = 0.0
        # ## **** AoI MODIFICATION END **** ##
        while valid_rounds < total_rounds:
            round_aborted = False  # 标记本轮是否异常终止
            round_index += 1
            self.current_status_message = f"正在运行第 {valid_rounds + 1}/{total_rounds} 轮..."
            print(f"[实验进度] 状态: {self.current_status_message}")
            print(f"[实验进度] 进度: {valid_rounds} / {total_rounds}")
            print(f"[实验进度] 总送达时间: {self.total_delivery_time:.2f} 秒")
            # ## **** AoI MODIFICATION START: 显示AoI统计 **** ##
            print(f"[实验进度] 总AoI: {self.total_aoi:.2f}")
            # ## **** AoI MODIFICATION END **** ##
            if valid_rounds > 0:
                print(f"[实验进度] 平均送达时间/轮: {self.average_delivery_time:.2f} 秒")
                # ## **** AoI MODIFICATION START: 显示平均AoI **** ##
                print(f"[实验进度] 平均AoI/轮: {self.average_aoi:.2f}")
                # ## **** AoI MODIFICATION END **** ##
            else:
                print(f"[实验进度] 平均送达时间/轮: 0.00 秒")
                # ## **** AoI MODIFICATION START: 显示平均AoI初始值 **** ##
                print(f"[实验进度] 平均AoI/轮: 0.00")
                # ## **** AoI MODIFICATION END **** ##
            print(f"正在运行第 {valid_rounds + 1}/{total_rounds} 轮... (实际第{round_index}次尝试)")
            packets_this_round = []

            # -- CMTP协议统计建树+传输总时间 --
            if USE_CTMP_ROUTING_MODEL:
                round_start_time = time.time()
                tree_build_start = time.time()
            # -- 在一个原子锁内完成一轮的准备工作 --
            with self.simulation_lock:
                self.sim_manager.start_simulation(num_uavs=num_uavs) # 每轮开始时重置无人机位置
                self.sim_manager.mac_layer.reset_counters()
                current_paths_this_round = []
                for pair in sd_pairs:
                    source_id, dest_id = pair["source"], pair["destination"]
                    path_ids, _ = self.sim_manager.get_shortest_path(source_id, dest_id)
                    if path_ids:
                        current_paths_this_round.append({
                            "source": source_id,
                            "destination": dest_id,
                            "path": path_ids
                        })
                        created_packets, _ = self.sim_manager.initiate_data_transfer(
                            source_id, dest_id, packet_count=1
                        )
                        if created_packets:
                            packets_this_round.extend(created_packets)
                self.current_round_paths = current_paths_this_round
                if not packets_this_round:
                    self.current_status_message = f"第 {valid_rounds + 1} 轮跳过: 未能为任何源-目标对找到有效路径。"
                    time.sleep(0.5)
                    continue
            if USE_CTMP_ROUTING_MODEL:
                tree_build_end = time.time()
                tree_build_time = tree_build_end - tree_build_start

            # -- 运行仿真直到本轮结束 --
            if not USE_CTMP_ROUTING_MODEL:
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
                    for pkt in packets_this_round:
                        if pkt.status != 'delivered':
                            pkt.status = 'failed_timeout'
                    round_aborted = True
                    break
                time.sleep(0.001)

            round_end_time = time.time()

            # -- 在一个原子锁内完成统计更新 --
            with self.simulation_lock:
                if not round_aborted:
                    self.sim_manager.mac_layer.collect_final_packet_status(packets_this_round)
                    import math
                    delivery_times = [pkt.get('delivery_time', 0) for pkt in self.sim_manager.mac_layer.packet_status_snapshot if pkt.get('delivery_time', None) is not None]
                    delivery_times = [d for d in delivery_times if isinstance(d, (int, float)) and not (math.isinf(d) or math.isnan(d))]
                    
                    if delivery_times:
                        max_delivery_time_this_round = max(delivery_times)
                        # 在CMTP模式下，将树构建时间加到送达时间上
                        if USE_CTMP_ROUTING_MODEL:
                            max_delivery_time_this_round += tree_build_time
                        
                        self.total_delivery_time += max_delivery_time_this_round
                        
                        # ## **** AoI MODIFICATION START: 更新AoI统计 **** ##
                        # 使用MAC层计算的average_aoi而不是total_aoi
                        if hasattr(self.sim_manager.mac_layer, 'average_aoi'):
                            self.total_aoi += self.sim_manager.mac_layer.average_aoi
                        # ## **** AoI MODIFICATION END **** ##
                        
                        valid_rounds += 1
                        self.average_delivery_time = self.total_delivery_time / valid_rounds if valid_rounds > 0 else 0.0
                        # ## **** AoI MODIFICATION START: 计算平均AoI **** ##
                        self.average_aoi = self.total_aoi / valid_rounds if valid_rounds > 0 else 0.0
                        # ## **** AoI MODIFICATION END **** ##
                    else:
                        self.current_status_message += "（本轮无有效包，未计入统计）"
                else:
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
            if USE_CTMP_ROUTING_MODEL:
                print(f"[CMTP] 本轮建树时间: {tree_build_time:.4f} 秒")
            # ## **** AoI MODIFICATION START: 显示AoI统计结果 **** ##
            print(f"总AoI: {self.total_aoi:.2f}")
            print(f"平均AoI: {self.average_aoi:.2f}")
            # ## **** AoI MODIFICATION END **** ##
            for pkt in self.sim_manager.packets_in_network:
                print(f"\n数据包 {pkt.id} ({pkt.source_id} -> {pkt.destination_id}):")
                print(f"  状态: {pkt.status}")
                print(f"  实际路径: {pkt.actual_hops}")
                if hasattr(pkt, 'delivery_time') and pkt.delivery_time is not None:
                    print(f"  送达时间: {pkt.delivery_time:.2f}秒")
                # ## **** AoI MODIFICATION START: 显示单个包的AoI **** ##
                if hasattr(pkt, 'aoi') and pkt.aoi is not None:
                    print(f"  AoI: {pkt.aoi:.2f}")
                # ## **** AoI MODIFICATION END **** ##
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
                "total_delivery_time": getattr(self, 'total_delivery_time', 0.0),
                "average_delivery_time": getattr(self, 'average_delivery_time', 0.0),
                # ## **** AoI MODIFICATION START: 添加AoI相关信息 **** ##
                "total_aoi": getattr(self, 'total_aoi', 0.0),
                "average_aoi": getattr(self, 'average_aoi', 0.0),
                # ## **** AoI MODIFICATION END **** ##
                "message": self.current_status_message,
                "current_paths": self.current_round_paths, # 新增字段
                "final_paths": [
                    {
                        "id": pkt.id,
                        "source": pkt.source_id,
                        "destination": pkt.destination_id,
                        "actual_hops": list(pkt.actual_hops),
                        "delivery_time": safe_float(getattr(pkt, 'delivery_time', None)),
                        # ## **** AoI MODIFICATION START: 添加AoI字段 **** ##
                        "aoi": safe_float(getattr(pkt, 'aoi', None))
                        # ## **** AoI MODIFICATION END **** ##
                    }
                    for pkt in self.sim_manager.packets_in_network
                ] if not self.is_running else None,
                "all_actual_paths": self.all_round_actual_paths if not self.is_running else None  # 新增
            }
        return status
    # ## **** MODIFICATION END **** ##