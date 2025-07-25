// 文件: frontend/js/script.js (已更新以完全匹配后端MAC层冲突队列逻辑)

document.addEventListener('DOMContentLoaded', () => {
    // --- 元素获取 ---
    const canvas = document.getElementById('simulationCanvas');
    if (!canvas) { console.error("Canvas element not found!"); return; }
    const ctx = canvas.getContext('2d');
    
    const startButton = document.getElementById('startButton');
    const stepButton = document.getElementById('stepButton');
    const autoStepButton = document.getElementById('autoStepButton');
    const stopButton = document.getElementById('stopButton');
    const numUAVsInput = document.getElementById('numUAVsInput');
    const currentTimeDisplay = document.getElementById('currentTime');
    const simulationStatusDisplay = document.getElementById('simulationStatus');
    const messagesDisplay = document.getElementById('messages');
    const sourceUAVIdInput = document.getElementById('sourceUAVIdInput');
    const targetUAVIdInput = document.getElementById('targetUAVIdInput');
    const showPathButton = document.getElementById('showPathButton');
    const clearPathButton = document.getElementById('clearPathButton');
    const pathMessageDisplay = document.getElementById('pathMessage');
    const macLogContainer = document.getElementById('macLogContainer');
    const expRoundsInput = document.getElementById('expRoundsInput');
    const expPairsCountInput = document.getElementById('expPairsCountInput');
    const generatePairsButton = document.getElementById('generatePairsButton');
    const startExperimentButton = document.getElementById('startExperimentButton');
    const expStatusMessage = document.getElementById('expStatusMessage');
    const expCompletedRounds = document.getElementById('expCompletedRounds');
    const expTotalRounds = document.getElementById('expTotalRounds');
    const expTotalTime = document.getElementById('expTotalTime');
    const expAvgTime = document.getElementById('expAvgTime');
    const experimentPathsDisplay = document.getElementById('experimentPathsDisplay');

    // 新的可视化面板元素
    const visualizationContainer = document.getElementById('visualization-container');
    const visualizationTimeline = document.getElementById('visualization-timeline');


    const API_BASE_URL = 'http://127.0.0.1:5001/api';
    const UAV_RADIUS = 5;

    // --- 状态变量 ---
    let currentUAVs = [], currentPackets = [], currentGridConfig = null;
    let uavMap = new Map(), autoSteppingInterval = null, experimentPollingInterval = null;
    let singleShortestPath = null;
    let staticExperimentPaths = [];
    let allExperimentRoundsData = [];
    let previousExperimentStatus = null; 

    // --- 核心功能 ---
    async function apiCall(endpoint, method = 'GET', body = null) {
        try {
            const options = { method, headers: { 'Content-Type': 'application/json' } };
            if (method.toUpperCase() !== 'GET' && method.toUpperCase() !== 'HEAD') {
                options.body = JSON.stringify(body || {});
            }
            const response = await fetch(`${API_BASE_URL}/${endpoint}`, options);
            const text = await response.text();
            const data = text ? JSON.parse(text) : null;
            return { ok: response.ok, status: response.status, data: data };
        } catch (error) {
            console.error(`API call to ${endpoint} failed:`, error);
            displayMessage(`连接后端失败: ${error.message}`, true);
            return { ok: false, status: 0, data: null };
        }
    }

    function updateUIFromState(stateData) {
        if (!stateData) return;
        if (experimentPollingInterval === null) {
            currentUAVs = stateData.uavs || [];
            updateUAVMap(currentUAVs);
            currentPackets = stateData.packets || [];
        }
        // 如果有网格配置则更新，否则设置为null（非PTP协议）
        currentGridConfig = stateData.grid_config || null;
        // 优先mac_packet_status，其次mac_log，展示格式不变
        if (stateData.mac_packet_status) {
            renderMacLog(stateData.mac_packet_status.map(pkt =>
                `Pkt:${pkt.id} [${pkt.status}] holder:${pkt.current_holder_id} hop:${pkt.current_hop_index} retrans:${pkt.retransmission_count} path:${pkt.actual_hops.join('→')}`
            ));
        } else if (stateData.mac_log) {
            renderMacLog(stateData.mac_log);
        }
        currentTimeDisplay.textContent = (stateData.time !== undefined ? stateData.time : 0.0).toFixed(2);
        simulationStatusDisplay.textContent = stateData.status || "未知";
        const isRunning = stateData.status === "running";
        const isExperimentRunning = experimentPollingInterval !== null;
        stepButton.disabled = !isRunning || isExperimentRunning;
        autoStepButton.disabled = !isRunning || isExperimentRunning;
        stopButton.disabled = !isRunning || isExperimentRunning;
    }

    function redrawCanvas() {
        if (!ctx) return;
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        // 只有当currentGridConfig存在时才绘制网格背景（PTP协议）
        if (currentGridConfig) drawGridBackground();
        if (currentUAVs) currentUAVs.forEach(drawUAV);
        if (staticExperimentPaths && staticExperimentPaths.length > 0) {
            drawMultiplePaths(staticExperimentPaths);
        } else if (singleShortestPath) {
            drawPath(singleShortestPath, 'rgba(255, 0, 0, 0.7)');
        }
        if (currentPackets) currentPackets.forEach(packet => {
            const holderUAV = uavMap.get(packet.current_holder_id);
            if (holderUAV) drawPacket(holderUAV, packet);
        });
    }

    function drawGridBackground() {
        const { rows, cols, prr_map, width, height } = currentGridConfig;
        if (!rows || !cols || !prr_map) return;
        const cellWidth = width / cols;
        const cellHeight = height / rows;
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                const prr = prr_map[r][c];
                const greenVal = Math.floor(255 * prr);
                const redVal = Math.floor(255 * (1 - prr));
                const alpha = 0.4 * (1 - prr) + 0.05;
                ctx.fillStyle = `rgba(${redVal}, ${greenVal}, 0, ${alpha})`;
                ctx.fillRect(c * cellWidth, r * cellHeight, cellWidth, cellHeight);
                ctx.fillStyle = '#555';
                ctx.font = '14px Arial';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(`${prr.toFixed(2)}`, c * cellWidth + cellWidth / 2, r * cellHeight + cellHeight / 2);
            }
        }
        ctx.textAlign = 'start';
        ctx.textBaseline = 'alphabetic';
    }

    function drawUAV(uav) {
        if (typeof uav.x !== 'number' || typeof uav.y !== 'number') return;
        ctx.beginPath();
        ctx.arc(uav.x, uav.y, UAV_RADIUS, 0, Math.PI * 2);
        ctx.fillStyle = uav.color || 'gray';
        ctx.fill();
        ctx.strokeStyle = 'black';
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.fillStyle = 'black';
        ctx.font = '12px Arial';
        ctx.fillText(uav.id, uav.x + UAV_RADIUS, uav.y - UAV_RADIUS);
    }

    function drawPacket(uav, packet) {
        const packetSize = 6;
        const packetOffsetX = -12;
        const packetOffsetY = -12;
        ctx.save();
        if (packet.status === 'delivered') ctx.fillStyle = 'lime';
        else if (packet.status.startsWith('failed')) ctx.fillStyle = 'black';
        else ctx.fillStyle = 'orange';
        ctx.fillRect(uav.x + packetOffsetX, uav.y + packetOffsetY, packetSize, packetSize);
        ctx.lineWidth = 1;
        ctx.strokeStyle = 'black';
        ctx.strokeRect(uav.x + packetOffsetX, uav.y + packetOffsetY, packetSize, packetSize);
        ctx.restore();
    }
    
    function drawMultiplePaths(pathsData) {
        const colors = ['#FF4136', '#0074D9', '#2ECC40', '#FFDC00', '#B10DC9', '#FF851B', '#7FDBFF', '#3D9970'];
        pathsData.forEach((pathInfo, index) => {
            const color = colors[index % colors.length];
            if (pathInfo.path) {
                drawPath(pathInfo.path, color);
            }
        });
    }

    function drawPath(pathNodeIds, color) {
        if (!pathNodeIds || pathNodeIds.length < 2) return;
        for (let i = 0; i < pathNodeIds.length - 1; i++) {
            const uav1 = uavMap.get(pathNodeIds[i]);
            const uav2 = uavMap.get(pathNodeIds[i+1]);
            if (uav1 && uav2) drawArrow(ctx, uav1.x, uav1.y, uav2.x, uav2.y, 10, color);
        }
    }

    function drawArrow(ctx, fromX, fromY, toX, toY, arrowSize = 10, color = 'rgba(255, 0, 0, 0.7)') {
        const angle = Math.atan2(toY - fromY, toX - fromX);
        ctx.save();
        ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(fromX, fromY); ctx.lineTo(toX, toY); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(toX, toY);
        ctx.lineTo(toX - arrowSize * Math.cos(angle - Math.PI / 6), toY - arrowSize * Math.sin(angle - Math.PI / 6));
        ctx.lineTo(toX - arrowSize * Math.cos(angle + Math.PI / 6), toY - arrowSize * Math.sin(angle + Math.PI / 6));
        ctx.closePath(); ctx.fill();
        ctx.restore();
    }

    function updateUAVMap(uavs) { uavMap.clear(); if(uavs) uavs.forEach(uav => uavMap.set(uav.id, uav)); }
    function displayMessage(text, isError=false) { if(messagesDisplay) { messagesDisplay.textContent = text; messagesDisplay.style.color = isError ? 'red' : 'green'; } }
    function renderMacLog(logEntries) { 
        if (macLogContainer && Array.isArray(logEntries)) 
            macLogContainer.textContent = logEntries.slice().reverse().join('\n'); 
        macLogContainer.scrollTop = 0; 
    }
    
    async function initializeApp() {
        displayMessage("正在连接到后端...", false);
        const result = await apiCall('simulation/state');
        if (result && result.ok && result.data) {
            updateUIFromState(result.data);
            displayMessage("连接成功。", false);
        } else {
            displayMessage(result ? (result.data?.error || "获取初始状态失败。") : "连接失败", true);
        }
        redrawCanvas();
    }

    async function startSimulation() {
        displayMessage("正在启动仿真...", false);
        const result = await apiCall('simulation/start', 'POST', { num_uavs: parseInt(numUAVsInput.value) });
        if (result && result.ok && result.data) {
            updateUIFromState(result.data.current_state);
            displayMessage(result.data.status_message, false);
            clearAllPathDisplays();
        } else {
            displayMessage(result.data?.error || "启动失败。", true);
        }
        redrawCanvas();
    }
    
    async function stopSimulation() {
        stopAutoStepping();
        const result = await apiCall('simulation/stop', 'POST');
        if (result && result.ok && result.data) {
            updateUIFromState(result.data.current_state);
            displayMessage(result.data.status_message, false);
        }
        redrawCanvas();
    }

    async function stepSimulation() {
        const result = await apiCall('simulation/step', 'POST', { time_increment: 0.1 });
        if (result && result.ok && result.data) {
            updateUIFromState(result.data.current_state);
            if (singleShortestPath) { // Simplified dynamic path update
                 await fetchSingleShortestPathLogic(singleShortestPath[0], singleShortestPath[singleShortestPath.length - 1], true);
            }
        } else {
            stopAutoStepping();
        }
        redrawCanvas();
    }

    function stopAutoStepping() { if(autoSteppingInterval) {clearInterval(autoSteppingInterval); autoSteppingInterval = null; autoStepButton.textContent = "自动运行";}}
    function toggleAutoStepping() {if(autoSteppingInterval){stopAutoStepping();}else{autoSteppingInterval=setInterval(stepSimulation,100);autoStepButton.textContent="停止自动";}}
    
    async function fetchSingleShortestPathLogic(sourceId, targetId, isDynamicUpdate = false) {
        if (!isDynamicUpdate) {
            pathMessageDisplay.textContent = "正在查找路径...";
            pathMessageDisplay.style.color = "blue";
        }
        const result = await apiCall('simulation/shortest-path', 'POST', { source_id: sourceId, target_id: targetId });
        if (result && result.ok && result.data.path) {
            singleShortestPath = result.data.path;
            const pathString = result.data.path.join(' → ');
            pathMessageDisplay.textContent = `路径: ${pathString} (长度: ${result.data.path.length - 1} 跳)`;
            pathMessageDisplay.style.color = "green";
        } else {
            singleShortestPath = null;
            if (!isDynamicUpdate) {
                pathMessageDisplay.textContent = result.data?.error || "未找到路径";
                pathMessageDisplay.style.color = "red";
            }
        }
    }
    
    async function handleShowPathButtonClick() {
        clearAllPathDisplays();
        const sourceId = parseInt(sourceUAVIdInput.value);
        const targetId = parseInt(targetUAVIdInput.value);
        if (isNaN(sourceId) || isNaN(targetId)) return;
        await fetchSingleShortestPathLogic(sourceId, targetId, false);
        redrawCanvas();
    }

    function clearSinglePathDisplay() {
        singleShortestPath = null;
        pathMessageDisplay.textContent = ""; 
        redrawCanvas();
    }
    
    function clearAllPathDisplays() {
        clearSinglePathDisplay();
        staticExperimentPaths = [];
        allExperimentRoundsData = [];
        previousExperimentStatus = null; 
        if(experimentPathsDisplay) experimentPathsDisplay.innerHTML = '';
        if(visualizationContainer) visualizationContainer.style.display = 'none';
        startExperimentButton.disabled = true;
    }

    function setManualControlsDisabled(disabled) {
        startButton.disabled = disabled;
        stepButton.disabled = disabled;
        autoStepButton.disabled = disabled;
        stopButton.disabled = disabled;
        showPathButton.disabled = disabled;
        clearPathButton.disabled = disabled;
        generatePairsButton.disabled = disabled;
    }
    
    function renderExperimentPaths(roundsData) {
        if (!experimentPathsDisplay) return;
        experimentPathsDisplay.innerHTML = ''; 
        if (!roundsData || roundsData.length === 0) {
            return;
        }
        roundsData.forEach(roundData => {
            const roundContainer = document.createElement('div');
            const roundHeader = document.createElement('p');
            roundHeader.style.fontWeight = 'bold';
            roundHeader.style.margin = '10px 0 5px 0';
            roundHeader.style.borderBottom = '1px solid #ccc';
            roundHeader.textContent = `--- 第 ${roundData.round} 轮 ---`;
            roundContainer.appendChild(roundHeader);
            if (roundData.paths && roundData.paths.length > 0) {
                roundData.paths.forEach(p => {
                    const pathString = p.path ? p.path.join(' → ') : 'N/A';
                    const el = document.createElement('div');
                    el.textContent = `[${p.source} → ${p.destination}]: ${pathString}`;
                    roundContainer.appendChild(el);
                });
            } else {
                 const el = document.createElement('div');
                 el.style.fontStyle = 'italic';
                 el.textContent = ' (无有效路径记录)';
                 roundContainer.appendChild(el);
            }
            experimentPathsDisplay.appendChild(roundContainer);
        });
    }

    function renderProcessVisualization(timelineData) {
        if (!visualizationTimeline || !staticExperimentPaths.length) return;
        visualizationTimeline.innerHTML = '';

        const staticPathsGrid = document.createElement('div');
        staticPathsGrid.className = 'paths-grid';
        staticExperimentPaths.forEach(pathInfo => {
            const pathRow = document.createElement('div');
            pathRow.className = 'path-row';
            pathInfo.path.forEach((nodeId, index) => {
                const nodeEl = document.createElement('div');
                nodeEl.className = 'path-node';
                nodeEl.textContent = nodeId;
                const uav = uavMap.get(nodeId);
                if (uav) {
                    nodeEl.style.backgroundColor = uav.color;
                }
                pathRow.appendChild(nodeEl);
                if (index < pathInfo.path.length - 1) {
                    const linkEl = document.createElement('div');
                    linkEl.className = 'path-link';
                    linkEl.id = `vis-link-tpl-${nodeId}-${pathInfo.path[index + 1]}`;
                    pathRow.appendChild(linkEl);
                }
            });
            staticPathsGrid.appendChild(pathRow);
        });

        const initialSlice = document.createElement('div');
        initialSlice.className = 'timeslice-container';
        initialSlice.innerHTML = `<div class="timeslice-label">初始路径:</div>`;
        initialSlice.appendChild(staticPathsGrid.cloneNode(true));
        visualizationTimeline.appendChild(initialSlice);

        timelineData.forEach(slice => {
            const sliceContainer = document.createElement('div');
            sliceContainer.className = 'timeslice-container';
            sliceContainer.innerHTML = `<div class="timeslice-label">T = ${slice.time.toFixed(1)}s:</div>`;
            const pathsGridForSlice = staticPathsGrid.cloneNode(true);
            slice.transmissions.forEach(tx => {
                const linkEl = pathsGridForSlice.querySelector(`#vis-link-tpl-${tx.from}-${tx.to}`);
                if (linkEl) {
                    const marker = document.createElement('div');
                    marker.className = `packet-marker ${tx.status}`;
                    linkEl.appendChild(marker);
                }
            });
            sliceContainer.appendChild(pathsGridForSlice);
            visualizationTimeline.appendChild(sliceContainer);
        });
    }

    // ## **** NEW: 可视化逻辑与后端MAC层完全对齐 **** ##
    // 该函数模拟了后端的冲突队列机制，以解决前端可视化中的死锁问题，并正确显示"等待中"状态。
    function getBackendAlignedVisualizationData() {
        if (!staticExperimentPaths || staticExperimentPaths.length === 0) return [];
        
        const timeline = [];
        let time = 0.0;
        
        // 初始化所有路径的状态
        const pathStates = staticExperimentPaths.map(p => ({
            path: [...p.path],
            currentIndex: 0,
            isFinished: false,
            // 为每个路径状态添加一个唯一的ID，以便查找
            id: `${p.source}-${p.destination}`
        }));
        
        // K: receiverId, V: [senderId1, senderId2, ...]
        const collisionQueues = new Map(); 
        const MAX_ITERATIONS = 100; // 防止无限循环
        let iteration = 0;

        while (pathStates.some(p => !p.isFinished) && iteration < MAX_ITERATIONS) {
            time += 1.0;
            const slice = { time: time, transmissions: [] };
            
            const activePaths = pathStates.filter(p => !p.isFinished);
            if (activePaths.length === 0) break;

            const transmittersThisStep = new Map(); // K: senderId, V: receiverId
            const newSendersByReceiver = new Map();
            const senderToPathState = new Map(); // 方便通过senderId查找其路径状态

            // 1. 优先处理已在冲突队列中的发送者，并标记等待者
            for (const [receiverId, queue] of collisionQueues.entries()) {
                if (queue.length > 0) {
                    const headSenderId = queue[0];
                    transmittersThisStep.set(headSenderId, receiverId);
                    // 标记队列中其他成员为"等待中"
                    for (let i = 1; i < queue.length; i++) {
                        const waitingSenderId = queue[i];
                        slice.transmissions.push({ from: waitingSenderId, to: receiverId, status: 'waiting' });
                    }
                }
            }
            
            // 2. 收集所有新的、不在任何队列中的传输请求
            for (const state of activePaths) {
                const senderId = state.path[state.currentIndex];
                senderToPathState.set(senderId, state);

                let isInAnyQueue = false;
                for(const q of collisionQueues.values()){
                    if(q.includes(senderId)) {
                        isInAnyQueue = true;
                        break;
                    }
                }

                if (!isInAnyQueue) {
                    const receiverId = state.path[state.currentIndex + 1];
                    if (!newSendersByReceiver.has(receiverId)) {
                        newSendersByReceiver.set(receiverId, []);
                    }
                    newSendersByReceiver.get(receiverId).push(senderId);
                }
            }

            // 3. 检测并处理新的冲突
            for (const [receiverId, senders] of newSendersByReceiver.entries()) {
                if (senders.length > 1) { // 发生新冲突
                    collisionQueues.set(receiverId, senders); // 创建新队列
                    senders.forEach(senderId => {
                         slice.transmissions.push({ from: senderId, to: receiverId, status: 'failure' });
                    });
                } else { // 无新冲突，可以尝试发送
                    const senderId = senders[0];
                    transmittersThisStep.set(senderId, receiverId);
                }
            }

            // 4. 对本轮最终确定的成功发送者进行处理
            for (const [senderId, receiverId] of transmittersThisStep.entries()) {
                 slice.transmissions.push({ from: senderId, to: receiverId, status: 'success' });
                 
                 // 从冲突队列中移除成功的发送者
                 if (collisionQueues.has(receiverId)) {
                     const queue = collisionQueues.get(receiverId);
                     if (queue[0] === senderId) {
                         queue.shift(); // 移除队首
                         if (queue.length === 0) {
                             collisionQueues.delete(receiverId);
                         }
                     }
                 }
                 
                 // 推进成功传输的路径状态
                 const state = senderToPathState.get(senderId);
                 if (state) {
                     state.currentIndex++;
                     if (state.currentIndex >= state.path.length - 1) {
                         state.isFinished = true;
                     }
                 }
            }
            
            if (slice.transmissions.length > 0) {
                timeline.push(slice);
            }
            
            iteration++;
        }

        if (iteration >= MAX_ITERATIONS && pathStates.some(p => !p.isFinished)) {
            console.warn("Visualization simulation reached max iterations. Some paths may be stuck.");
        }

        return timeline;
    }

    async function refreshFullStateAndRedraw() {
        const result = await apiCall('simulation/state');
        if (result.ok && result.data) {
            updateUIFromState(result.data);
            redrawCanvas();
        }
    }
    
    async function handleGeneratePairsClick() {
        clearAllPathDisplays();
        displayMessage("正在生成随机源-目标对...", false);
        const pairCount = parseInt(expPairsCountInput.value);
        if (isNaN(pairCount) || pairCount <= 0) {
            displayMessage("请输入有效的源-目标对数量。", true);
            return;
        }

        const result = await apiCall('experiment/generate-pairs', 'POST', { pair_count: pairCount });
        
        if (result && result.ok && result.data.pairs.length > 0) {
            // 深拷贝，防止后续被修改
            window.staticExperimentPaths = JSON.parse(JSON.stringify(result.data.pairs));
            staticExperimentPaths = window.staticExperimentPaths;
            displayMessage(result.data.message, false);
            await refreshFullStateAndRedraw();
            renderExperimentPaths([{ round: "初始", paths: staticExperimentPaths }]);
            startExperimentButton.disabled = false;
        } else {
            displayMessage(result.data?.error || "生成源-目标对失败。", true);
            staticExperimentPaths = [];
            startExperimentButton.disabled = true;
        }
    }

    async function handleStartExperimentClick() {
        const totalRounds = parseInt(expRoundsInput.value, 10) || 1;
        if (!staticExperimentPaths || staticExperimentPaths.length === 0) {
            displayMessage("请先生成源-目标对。", true);
            return;
        }
        
        setManualControlsDisabled(true);
        displayMessage("正在启动批处理实验...", false);
        
        // 更新总轮数显示
        document.getElementById('expTotalRounds').textContent = totalRounds;
        
        const result = await apiCall('experiment/start', 'POST', {
            total_rounds: totalRounds,
            pairs: staticExperimentPaths,
            num_uavs: parseInt(numUAVsInput.value, 10) || DEFAULT_NUM_UAVS
        });
        
        if (result && result.ok) {
            previousExperimentStatus = { completed_rounds: 0, current_paths: [] };
            displayMessage(result.data.message, false);
            experimentPollingInterval = setInterval(pollExperimentStatus, 200);
        } else {
            displayMessage(result.data?.error || "启动实验失败。", true);
            setManualControlsDisabled(false);
        }
    }
    
    async function pollExperimentStatus() {
        if (!previousExperimentStatus) return;

        const result = await apiCall('experiment/status');
        if (!result || !result.ok) {
            stopExperimentPolling("连接后端失败，停止轮询。");
            return;
        }
        const statusData = result.data;
        
        expStatusMessage.textContent = statusData.message;
        expCompletedRounds.textContent = statusData.completed_rounds;
        // 只显示送达时间统计
        if (statusData) {
            document.getElementById('expTotalDeliveryTime').textContent = (statusData.total_delivery_time || 0).toFixed(2);
            document.getElementById('expAvgDeliveryTime').textContent = (statusData.average_delivery_time || 0).toFixed(2);
            // ## **** AoI MODIFICATION START: 更新AoI统计显示 **** ##
            document.getElementById('expTotalAoI').textContent = (statusData.total_aoi || 0).toFixed(2);
            document.getElementById('expAvgAoI').textContent = (statusData.average_aoi || 0).toFixed(2);
            // ## **** AoI MODIFICATION END **** ##
            // ## **** ENERGY MODIFICATION START: 更新能耗统计显示 **** ##
            document.getElementById('expTotalEnergy').textContent = (statusData.total_energy || 0).toFixed(2);
            document.getElementById('expAvgEnergy').textContent = (statusData.average_energy || 0).toFixed(2);
            // ## **** ENERGY MODIFICATION END **** ##
        }
        
        if (statusData.completed_rounds > previousExperimentStatus.completed_rounds) {
            for (let i = previousExperimentStatus.completed_rounds + 1; i <= statusData.completed_rounds; i++) {
                const pathsForCompletedRound = previousExperimentStatus.current_paths || [];
                const roundExists = allExperimentRoundsData.some(d => d.round === i);
                if (!roundExists) {
                     allExperimentRoundsData.push({
                        round: i,
                        paths: pathsForCompletedRound
                    });
                }
            }
            renderExperimentPaths(allExperimentRoundsData);
        }
        
        previousExperimentStatus = statusData;

        if (!statusData.is_running) {
            stopExperimentPolling(statusData.message);
        }
    }

    async function refreshMacLogOnly() {
        const result = await apiCall('simulation/state');
        if (result.ok && result.data) {
            const stateData = result.data;
            if (stateData.mac_packet_status) {
                // 按时间片分组渲染事件历史
                const eventsByTime = {};
                stateData.mac_packet_status.forEach(pkt => {
                    if (Array.isArray(pkt.event_history) && pkt.event_history.length > 0) {
                        pkt.event_history.forEach(ev => {
                            if (ev.event === 'waiting') return; // 跳过waiting事件
                            const t = ev.sim_time !== undefined ? ev.sim_time : 0;
                            if (!eventsByTime[t]) eventsByTime[t] = [];
                            // 获取下一个hop（如果有）
                            let nextHop = undefined;
                            if (pkt.path && ev.hop !== undefined && ev.hop + 1 < pkt.path.length) {
                                nextHop = pkt.path[ev.hop + 1];
                            }
                            let holderStr = nextHop ? `${ev.holder}->${nextHop}` : `${ev.holder}`;
                            let desc = `Pkt:${pkt.id} [${ev.event}] ${holderStr} hop:${ev.hop + 1}`;
                            if (ev.info) desc += ` info:${ev.info}`;
                            eventsByTime[t].push(desc);
                        });
                    } else {
                        // 没有事件历史时，归入时间片0
                        if (!eventsByTime[0]) eventsByTime[0] = [];
                        eventsByTime[0].push(`Pkt:${pkt.id} [${pkt.status}] holder:${pkt.current_holder_id} hop:${pkt.current_hop_index} retrans:${pkt.retransmission_count} path:${pkt.actual_hops.join('→')}`);
                    }
                });
                // 按时间片升序渲染
                const sortedTimes = Object.keys(eventsByTime).map(Number).sort((a,b)=>a-b);
                const lines = [];
                sortedTimes.forEach(t => {
                    lines.push(`--- 时间片 ${t} ---`);
                    lines.push(...eventsByTime[t]);
                });
                renderMacLog(lines);
            } else if (stateData.mac_log) {
                renderMacLog(stateData.mac_log);
            } else {
                renderMacLog([]);
            }
        }
    }

    function stopExperimentPolling(finalMessage) {
        if (experimentPollingInterval) { clearInterval(experimentPollingInterval); experimentPollingInterval = null; }
        visualizationContainer.style.display = 'block';
        const alignedData = getBackendAlignedVisualizationData();
        renderProcessVisualization(alignedData);
        setManualControlsDisabled(false);
        startExperimentButton.disabled = true;
        previousExperimentStatus = null; 
        displayMessage(finalMessage || "实验已结束。", false);
        expStatusMessage.textContent = finalMessage;
        // 只刷新MAC层传输日志
        refreshMacLogOnly();
        // 新增：实验结束后用实际路径刷新右侧路径
        refreshFinalPathsDisplay();
    }

    async function refreshFinalPathsDisplay() {
        const result = await apiCall('experiment/status');
        if (result.ok && result.data && Array.isArray(result.data.final_paths)) {
            // 只用staticExperimentPaths渲染初始轮
            const initialPaths = (window.staticExperimentPaths && window.staticExperimentPaths.length > 0)
                ? window.staticExperimentPaths
                : [];
            const initialRound = {
                round: "初始",
                paths: initialPaths.map(p => ({
                    source: p.source,
                    destination: p.destination,
                    path: p.path
                }))
            };
            // 多轮实际路径
            let actualRounds = [];
            if (Array.isArray(result.data.all_actual_paths)) {
                actualRounds = result.data.all_actual_paths.map((roundPaths, idx) => ({
                    round: `${idx + 1}`,
                    paths: roundPaths.map(p => ({
                        source: p.source,
                        destination: p.destination,
                        path: p.actual_hops
                    }))
                }));
            }
            renderExperimentPaths([initialRound, ...actualRounds]);
        }
    }

    // --- 事件监听器绑定 ---
    startButton.addEventListener('click', startSimulation);
    stepButton.addEventListener('click', stepSimulation);
    autoStepButton.addEventListener('click', toggleAutoStepping);
    stopButton.addEventListener('click', stopSimulation);
    showPathButton.addEventListener('click', handleShowPathButtonClick);
    clearPathButton.addEventListener('click', clearSinglePathDisplay);
    generatePairsButton.addEventListener('click', handleGeneratePairsClick);
    startExperimentButton.addEventListener('click', handleStartExperimentClick);

    initializeApp();
});
