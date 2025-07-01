无人机网络仿真平台 (UAV Network Simulation Platform)
这是一个全栈Web应用程序，用于仿真无人机(UAV)自组网络中的通信、移动和数据传输。平台提供了一个可视化的仪表盘，允许用户实时观察仿真过程、控制仿真参数，并执行批处理实验以分析网络在不同条件下的性能。

✨ 主要功能
实时二维可视化: 在Web界面上通过HTML5 Canvas实时绘制无人机的位置、数据包的流动以及规划的通信路径。

动态仿真控制: 用户可以启动、重置、暂停、单步执行或自动运行仿真，并能动态调整无人机数量。

高级MAC层模拟: 后端mac.py实现了一个基于队列的冲突解决机制，用于处理多个无人机竞争同一信道资源的情况，而非简单的随机退避。

多模型通信信道:

距离限制: 超出通信范围 (UAV_COMMUNICATION_RANGE) 的无人机无法通信。

信号干扰 (SINR): 模拟网络中其他并发传输对当前通信链路的干扰，是决定传输成功与否的关键因素。

概率性丢包 (PRR): 根据无人机所在地理位置的不同，模拟不同区域的网络信号质量。

批处理实验框架:

支持并发执行多条源-目标（S-D）对的数据传输任务。

可自定义实验轮次，每一轮都会重新随机部署无人机，以测试在不同拓扑下的网络性能。

自动统计并展示关键性能指标（KPI），如总跳数和平均每轮跳数。

交互式路径查找: 用户可以手动选择任意两个无人机，系统会使用广度优先搜索（BFS）算法计算并显示它们之间的最短通信路径。

实验过程可视化: 在批处理实验结束后，前端会生成一个详细的时间轴，直观地展示每个时间片内所有路径的传输状态（成功、失败、等待），并复现后端的冲突队列解决机制。

📂 项目结构
.
├── backend/
│   ├── core/
│   │   ├── uav.py             # 定义无人机(UAV)对象及其行为
│   │   └── packet.py          # 定义数据包(Packet)对象
│   ├── mac_layer/
│   │   └── mac.py             # 核心：实现MAC层冲突解决机制
│   ├── models/
│   │   ├── communication_model.py # 实现SINR, PRR等通信失败模型
│   │   └── mobility_model.py    # 定义无人机的移动模型
│   ├── app.py                 # Flask应用入口，提供API
│   ├── simulation_manager.py  # 管理仿真状态和流程
│   ├── experiment_manager.py  # 管理批处理实验
│   ├── simulation_config.py   # 存放所有全局仿真参数
│   └── requirements.txt       # 后端依赖
│
└── frontend/
    ├── css/
    │   └── style.css          # 前端页面的样式表
    ├── js/
    │   └── script.js          # 前端核心逻辑，与后端API交互
    └── index.html             # 应用主页面

🚀 如何运行
先决条件
Python 3.7+

pip (Python包管理器)

一个现代的Web浏览器 (如 Chrome, Firefox)

1. 启动后端服务
首先，在您的终端中，导航到backend目录并启动Flask服务器。

# 1. 进入后端目录
cd path/to/your/project/backend

# 2. (建议) 创建并激活一个虚拟环境
python -m venv venv
# Windows
venv\Scripts\activate
# macOS/Linux
source venv/bin/activate

# 3. 安装依赖
# requirements.txt 文件应包含以下内容:
# Flask
# Flask-Cors
pip install -r requirements.txt

# 4. 启动后端Flask应用
python app.py

当您看到类似 * Running on http://127.0.0.1:5001/ 的输出时，表示后端服务已成功启动。

2. 打开前端界面
直接在您的Web浏览器中打开 frontend/index.html 文件。

注意: 无需为前端单独启动Web服务器，因为它通过HTTP请求与已在运行的后端服务进行通信。

📖 使用指南
前端界面分为三个主要控制区域：

仿真控制 (Simulation Control)

无人机数量: 设置仿真开始时的无人机总数。

开始/重置: 启动一个新的仿真实例或重置当前实例。

自动运行/暂停/单步: 在仿真开始后，控制其进程。

路径查找 (Pathfinding)

输入起始和目标无人机的ID，点击“查找路径”即可在画布上看到高亮显示的最短路径。

批处理实验 (Batch Experiment)

这是项目的核心功能，推荐的工作流程如下：

设置参数: 输入“执行轮次数”和“源-目标对个数”。

生成配对: 点击 “1. 生成随机源-目标对” 按钮。系统会随机选择无人机配对，并计算出它们在当前拓扑下的初始路径，显示在下方的“当前轮次路径”区域。

开始实验: 点击 “2. 开始执行实验” 按钮。此时所有手动控制按钮将被禁用，实验在后台运行。您可以在状态面板中看到实时的进度更新。
4