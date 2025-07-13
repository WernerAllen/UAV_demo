# 文件: backend/app.py (已更新)
# 描述: 增加新API端点，并修改实验开始接口

from flask import Flask, jsonify, request
from flask_cors import CORS
from experiment_manager import ExperimentManager
from simulation_config import API_DEFAULT_PORT

app = Flask(__name__)
CORS(app, resources={r"/api/*": {"origins": "*"}})

exp_manager = ExperimentManager()
sim_manager = exp_manager.sim_manager


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
        
        # 确保仿真已启动并有无人机
        if not sim_manager.uavs:
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


if __name__ == '__main__':
    print("Starting Flask server for UAV Simulation...")
    app.run(host='0.0.0.0', port=API_DEFAULT_PORT, debug=True)
