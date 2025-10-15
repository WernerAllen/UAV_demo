# 文件: backend/app.py (已更新)
# 描述: 增加新API端点，并修改实验开始接口

import importlib

from flask import Flask, jsonify, request
from flask_cors import CORS

from experiment_manager import ExperimentManager
from simulation_config import API_DEFAULT_PORT
import simulation_config

app = Flask(__name__)
CORS(app, resources={r"/api/*": {"origins": "*"}})

exp_manager = ExperimentManager()
sim_manager = exp_manager.sim_manager

# ## **** NEW ENDPOINT: 设置路由模型 **** ##
@app.route('/api/simulation/set-routing-model', methods=['POST'])
def set_routing_model_endpoint():
    if exp_manager.is_running:
        return jsonify({"error": "实验进行中，无法更改路由模型"}), 409
    if not request.json:
        return jsonify({"error": "无效的JSON数据"}), 400
    
    model = request.json.get('model', '').upper()
    if model not in ["DHYTP", "MTP", "PTP", "NONE"]:
        return jsonify({"error": "无效的路由模型，必须是 DHYTP、MTP、PTP 或 NONE"}), 400
    
    # 更新路由模型配置
    simulation_config.ROUTING_MODEL = model
    simulation_config.USE_DHYTP_ROUTING_MODEL = model == "DHYTP"
    simulation_config.USE_MTP_ROUTING_MODEL = model == "MTP"
    simulation_config.USE_PTP_ROUTING_MODEL = model == "PTP"
    
    # 重新加载模块以确保更改生效
    importlib.reload(simulation_config)
    
    # 重启仿真以应用新的路由模型
    with exp_manager.simulation_lock:
        status_message = sim_manager.start_simulation()
        current_sim_state = sim_manager.get_simulation_state()
    
    return jsonify({
        "status_message": f"路由模型已设置为 {model}，仿真已重启",
        "current_state": current_sim_state,
        "routing_model": model
    })
# ## **** NEW ENDPOINT END **** ##


@app.route('/api/simulation/start', methods=['POST'])
def start_simulation_endpoint():
    if exp_manager.is_running:
        return jsonify({"error": "实验进行中，无法手动操作"}), 409
    if not request.json:
        return jsonify({"error": "无效的JSON数据"}), 400
    with exp_manager.simulation_lock:
        num_uavs = request.json.get('num_uavs')
        status_message = sim_manager.start_simulation(num_uavs=num_uavs)
        current_sim_state = sim_manager.get_simulation_state()
    return jsonify({"status_message": status_message, "current_state": current_sim_state})


@app.route('/api/simulation/step', methods=['POST'])
def step_simulation_endpoint():
    if exp_manager.is_running:
        return jsonify({"error": "实验进行中，无法手动操作"}), 409
    with exp_manager.simulation_lock:
        status_message = sim_manager.step_simulation()
        current_sim_state = sim_manager.get_simulation_state()
    return jsonify({"status_message": status_message, "current_state": current_sim_state})


@app.route('/api/simulation/state', methods=['GET'])
def get_simulation_state_endpoint():
    with exp_manager.simulation_lock:
        current_sim_state = sim_manager.get_simulation_state()
    return jsonify(current_sim_state)


@app.route('/api/simulation/stop', methods=['POST'])
def stop_simulation_endpoint():
    with exp_manager.simulation_lock:
        status_message = sim_manager.stop_simulation()
        current_sim_state = sim_manager.get_simulation_state()
    return jsonify({"status_message": status_message, "current_state": current_sim_state})


@app.route('/api/simulation/shortest-path', methods=['POST'])
def get_shortest_path_endpoint():
    if not request.json:
        return jsonify({"error": "无效的JSON数据"}), 400
    with exp_manager.simulation_lock:
        data = request.json
        source_id = int(data.get('source_id')) if data.get('source_id') is not None else None
        target_id = int(data.get('target_id')) if data.get('target_id') is not None else None

        path_ids, message = sim_manager.get_shortest_path(source_id, target_id)
        if path_ids:
            return jsonify({"path": path_ids, "message": message})
        else:
            return jsonify({"error": message, "path": None}), 404


@app.route('/api/simulation/send-packet', methods=['POST'])
def send_packet_endpoint():
    if not request.json:
        return jsonify({"error": "无效的JSON数据"}), 400
    with exp_manager.simulation_lock:
        data = request.json
        packets, message = sim_manager.initiate_data_transfer(
            int(data.get('source_id')),
            int(data.get('destination_id')),
            int(data.get('packet_count', 1))
        )
        if packets:
            return jsonify({"message": message, "packet_id": packets[0].id})
        else:
            return jsonify({"error": message}), 404

# ## **** NEW ENDPOINT: 生成随机源-目标对 **** ##
@app.route('/api/experiment/generate-pairs', methods=['POST'])
def generate_pairs_endpoint():
    if exp_manager.is_running:
        return jsonify({"error": "实验进行中，无法执行此操作"}), 409
    if not request.json:
        return jsonify({"error": "无效的JSON数据"}), 400
    
    with exp_manager.simulation_lock:
        pair_count = request.json.get('pair_count')
        if not isinstance(pair_count, int) or pair_count <= 0:
            return jsonify({"error": "无效的源-目标对数量"}), 400
        
        # 每次生成源-目标对时都重新初始化UAV位置
        print("🔄 重新随机分布UAV位置并生成源-目标对")
        sim_manager.start_simulation()

        pairs_data, message = sim_manager.generate_random_pairs_and_paths(pair_count)
        if pairs_data:
            return jsonify({"pairs": pairs_data, "message": message})
        else:
            return jsonify({"error": message, "pairs": []}), 404
# ## **** MODIFICATION END **** ##


# ## **** MODIFICATION START: 更新实验开始接口 **** ##
@app.route('/api/experiment/start', methods=['POST'])
def start_experiment_endpoint():
    if not request.json:
        return jsonify({"error": "无效的JSON数据"}), 400
    data = request.json
    try:
        params = {
            'total_rounds': int(data['total_rounds']),
            'pairs_data': data['pairs'], # 接收一个包含S-D对的列表
            'num_uavs': int(data.get('num_uavs', sim_manager.last_uav_count))
        }
        if not isinstance(params['pairs_data'], list):
             raise ValueError("pairs_data必须是一个列表")
    except (TypeError, ValueError, KeyError) as e:
        return jsonify({"error": f"无效或缺失的参数: {e}"}), 400

    success, message = exp_manager.start_experiment(**params)
    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500
# ## **** MODIFICATION END **** ##


@app.route('/api/experiment/status', methods=['GET'])
def get_experiment_status_endpoint():
    status = exp_manager.get_status()
    return jsonify(status)


# ## **** NEW ENDPOINT: 获取数据包事件历史 **** ##
@app.route('/api/simulation/packet-events', methods=['GET'])
def get_packet_events_endpoint():
    """获取所有数据包的事件历史"""
    with exp_manager.simulation_lock:
        if not sim_manager.packets_in_network:
            return jsonify({"message": "当前没有数据包", "packets": []})
        
        packets_data = []
        position_change_count = 0
        
        for pkt in sim_manager.packets_in_network:
            packet_info = {
                "id": pkt.id,
                "source_id": pkt.source_id,
                "destination_id": pkt.destination_id,
                "status": pkt.status,
                "actual_hops": list(pkt.actual_hops),
                "delivery_time": getattr(pkt, 'delivery_time', None),
                "event_history": []
            }
            
            # 统计位置变动事件
            for event in pkt.event_history:
                if event['event'] == 'position_change':
                    position_change_count += 1
                packet_info["event_history"].append({
                    "sim_time": event['sim_time'],
                    "event": event['event'],
                    "info": event['info']
                })
            
            packets_data.append(packet_info)
        
        return jsonify({
            "message": f"共 {len(packets_data)} 个数据包，{position_change_count} 次位置变动事件",
            "packets": packets_data,
            "position_change_count": position_change_count
        })
# ## **** MODIFICATION END **** ##

# ## **** ENERGY MODIFICATION START: 添加能耗统计API端点 **** ##
@app.route('/api/simulation/energy-stats', methods=['GET'])
def get_energy_stats_endpoint():
    with exp_manager.simulation_lock:
        energy_stats = sim_manager.get_energy_statistics()
    return jsonify({
        "energy_statistics": energy_stats,
        "current_protocol": simulation_config.ROUTING_MODEL
    })
# ## **** ENERGY MODIFICATION END **** ##

# ## **** NEW ENDPOINTS FOR MTP FRONTEND SUPPORT **** ##
@app.route('/api/simulation/config', methods=['GET'])
def get_simulation_config_endpoint():
    """获取仿真配置信息，包括协议类型"""
    return jsonify({
        "protocol": simulation_config.ROUTING_MODEL,
        "tree_pruning_enabled": simulation_config.TREE_PRUNING_ENABLED,
        "ellipse_eccentricity": simulation_config.ELLIPSE_ECCENTRICITY,
        "ellipse_expansion_factor": simulation_config.ELLIPSE_EXPANSION_FACTOR,
        "merge_distance_threshold": getattr(sim_manager.routing_model, 'MERGE_DISTANCE_THRESHOLD', 30) if sim_manager.routing_model else 30
    })

@app.route('/api/simulation/mtp-pruning-data', methods=['GET'])
def get_mtp_pruning_data_endpoint():
    """获取MTP协议的剪枝数据，包括椭圆区域和合并目标"""
    import math
    with exp_manager.simulation_lock:
        pruning_data = {
            "ellipses": [],
            "merge_targets": [],
            "tree_groups": []
        }
        
        # 只有MTP或DHyTP协议才返回剪枝数据
        if simulation_config.ROUTING_MODEL in ['MTP', 'DHYTP'] and sim_manager.routing_model:
            routing_model = sim_manager.routing_model
            
            # 获取椭圆区域数据
            if hasattr(routing_model, 'ellipse_regions'):
                for (source_id, dest_id), ellipse_info in routing_model.ellipse_regions.items():
                    # 重要：使用当前最新的UAV位置，而不是记录的位置
                    source_uav = sim_manager.mac_layer.uav_map.get(source_id)
                    dest_uav = sim_manager.mac_layer.uav_map.get(dest_id)
                    
                    if not source_uav or not dest_uav:
                        continue
                    
                    # 计算椭圆参数（基于当前实际位置）
                    focal_distance = math.sqrt(
                        (dest_uav.x - source_uav.x) ** 2 + 
                        (dest_uav.y - source_uav.y) ** 2
                    )
                    
                    # 椭圆的长半轴和短半轴
                    a = focal_distance * simulation_config.ELLIPSE_EXPANSION_FACTOR / 2
                    c = focal_distance / 2  # 半焦距
                    b = math.sqrt(a * a - c * c) if a > c else 0
                    
                    # 椭圆中心点
                    center_x = (source_uav.x + dest_uav.x) / 2
                    center_y = (source_uav.y + dest_uav.y) / 2
                    
                    # 椭圆旋转角度
                    rotation = math.atan2(dest_uav.y - source_uav.y, dest_uav.x - source_uav.x)
                    
                    pruning_data["ellipses"].append({
                        "source_id": source_id,
                        "dest_id": dest_id,
                        "center_x": center_x,
                        "center_y": center_y,
                        "a": a,  # 长半轴
                        "b": b,  # 短半轴
                        "rotation": rotation,  # 旋转角度（弧度）
                        "focal_distance": focal_distance
                    })
            
            # 获取合并目标数据和虚拟根节点信息
            if hasattr(routing_model, 'root_groups') and routing_model.root_groups:
                # 同时获取虚拟根节点信息
                virtual_roots_info = []
                
                for group_idx, group in enumerate(routing_model.root_groups):
                    if len(group) > 1:
                        # 计算虚拟根节点（与MTP协议中的逻辑一致）
                        center_x = sum(sim_manager.mac_layer.uav_map[id].x for id in group) / len(group)
                        center_y = sum(sim_manager.mac_layer.uav_map[id].y for id in group) / len(group)
                        center_z = sum(sim_manager.mac_layer.uav_map[id].z for id in group) / len(group)
                        
                        min_dist = float('inf')
                        virtual_root_id = group[0]
                        
                        for uav_id in group:
                            uav = sim_manager.mac_layer.uav_map[uav_id]
                            dist = (uav.x - center_x)**2 + (uav.y - center_y)**2 + (uav.z - center_z)**2
                            if dist < min_dist:
                                min_dist = dist
                                virtual_root_id = uav_id
                        
                        # 记录虚拟根节点信息
                        virtual_roots_info.append({
                            "group_index": group_idx,
                            "virtual_root_id": virtual_root_id,
                            "group_nodes": group
                        })
                        
                        # 所有组内节点都是合并目标（包括主根）
                        for target_node in group:
                            pruning_data["merge_targets"].append({
                                "uav_id": target_node,
                                "main_root_id": virtual_root_id,
                                "group_size": len(group),
                                "is_main_root": (target_node == virtual_root_id)
                            })
                
                pruning_data["tree_groups"] = routing_model.root_groups
                pruning_data["virtual_roots"] = virtual_roots_info
        
    return jsonify(pruning_data)

@app.route('/api/simulation/trigger-mtp-pruning', methods=['POST'])
def trigger_mtp_pruning_endpoint():
    """触发MTP协议的剪枝和合并算法"""
    if not request.json:
        return jsonify({"error": "无效的JSON数据"}), 400
    
    with exp_manager.simulation_lock:
        try:
            data = request.json
            destinations = data.get('destinations', [])
            source_dest_pairs = data.get('source_dest_pairs', [])
            
            # 确保使用MTP协议
            if simulation_config.ROUTING_MODEL != 'MTP':
                return jsonify({"error": "当前不是MTP协议，无法执行剪枝"}), 400
            
            if not sim_manager.routing_model:
                return jsonify({"error": "路由模型未初始化"}), 500
            
            # 获取所有源节点（假设使用第一个源节点作为代表）
            if source_dest_pairs:
                source_ids = [pair.get('source') for pair in source_dest_pairs if pair.get('source')]
                if source_ids:
                    representative_source = source_ids[0]
                    
                    # 执行MTP剪枝和合并算法
                    sim_manager.routing_model.build_virtual_tree_structures(
                        destination_ids=destinations,
                        source_id=representative_source
                    )
                    
                    # 重要：记录实际的源-目标对椭圆区域，确保与显示的路径匹配
                    if hasattr(sim_manager.routing_model, 'record_actual_source_dest_pairs'):
                        sim_manager.routing_model.record_actual_source_dest_pairs(source_dest_pairs)
                    
                    # 更新协议状态
                    if hasattr(sim_manager.routing_model, 'update_protocol_status'):
                        sim_manager.routing_model.update_protocol_status(
                            destinations, 
                            sim_manager.simulation_time
                        )
                    
                    return jsonify({
                        "success": True,
                        "message": f"MTP剪枝和合并算法执行完成，处理了{len(destinations)}个目标节点",
                        "destinations_count": len(destinations),
                        "source_pairs_count": len(source_dest_pairs)
                    })
                else:
                    return jsonify({"error": "未找到有效的源节点"}), 400
            else:
                return jsonify({"error": "未提供源-目标对数据"}), 400
                
        except Exception as e:
            return jsonify({"error": f"执行剪枝算法时发生错误: {str(e)}"}), 500

# ## **** NEW ENDPOINTS END **** ##


if __name__ == '__main__':
    print("Starting Flask server for UAV Simulation...")
    app.run(host='0.0.0.0', port=API_DEFAULT_PORT, debug=True)
