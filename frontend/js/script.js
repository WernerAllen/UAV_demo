// 文件: frontend/js/script.js (已更新以完全匹配后端MAC层冲突队列逻辑)

document.addEventListener('DOMContentLoaded', () => {
    // --- 元素获取 ---
    const canvas = document.getElementById('simulationCanvas');
    if (!canvas) { console.error("Canvas element not found!"); return; }
    const ctx = canvas.getContext('2d');
    
    // MTP协议对照组canvas
    const mtpCanvas = document.getElementById('mtpCanvas');
    const mtpCtx = mtpCanvas ? mtpCanvas.getContext('2d') : null;
    const canvasContainer = document.getElementById('canvasContainer');
    const mtpCanvasPanel = document.getElementById('mtpCanvasPanel');
    
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
    let currentProtocol = null; // 当前使用的协议
    let mtpPruningData = null; // MTP协议剪枝数据 

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
        
        // 更新协议信息和MTP剪枝数据
        if (stateData.protocol) {
            currentProtocol = stateData.protocol;
        }
        if (stateData.mtp_pruning_data) {
            mtpPruningData = stateData.mtp_pruning_data;
        }
        
        // 根据协议类型显示/隐藏MTP对照组canvas
        updateCanvasDisplay();
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
        if (currentGridConfig) drawGridBackground(ctx);
        if (currentUAVs) currentUAVs.forEach(uav => drawUAV(ctx, uav));
        if (staticExperimentPaths && staticExperimentPaths.length > 0) {
            drawMultiplePaths(ctx, staticExperimentPaths);
        } else if (singleShortestPath) {
            drawPath(ctx, singleShortestPath, 'rgba(255, 0, 0, 0.7)');
        }
        if (currentPackets) currentPackets.forEach(packet => {
            const holderUAV = uavMap.get(packet.current_holder_id);
            if (holderUAV) drawPacket(ctx, holderUAV, packet);
        });
        
        // 如果是MTP协议，同时更新MTP对照组canvas
        if (currentProtocol === 'MTP' && mtpCtx) {
            redrawMTPCanvas();
        }
    }

    // 添加canvas显示控制函数
    function updateCanvasDisplay() {
        if (!canvasContainer || !mtpCanvasPanel) return;
        
        if (currentProtocol === 'MTP') {
            // 显示双canvas布局
            canvasContainer.classList.remove('single-canvas');
            mtpCanvasPanel.style.display = 'flex';
        } else {
            // 显示单canvas布局
            canvasContainer.classList.add('single-canvas');
            mtpCanvasPanel.style.display = 'none';
        }
    }

    // MTP对照组canvas绘制函数
    function redrawMTPCanvas() {
        if (!mtpCtx) return;
        
        mtpCtx.clearRect(0, 0, mtpCanvas.width, mtpCanvas.height);
        
        // 绘制所有UAV节点（与原canvas保持一致的位置）
        if (currentUAVs) {
            currentUAVs.forEach(uav => drawUAV(mtpCtx, uav));
        }
        
        // 只有在有MTP剪枝数据时才绘制椭圆和标记
        if (mtpPruningData) {
            // 绘制剪枝椭圆区域
            if (mtpPruningData.ellipses && mtpPruningData.ellipses.length > 0) {
                const pathCount = staticExperimentPaths ? staticExperimentPaths.length : 0;
                console.log(`🔍 数据检查: 椭圆数量=${mtpPruningData.ellipses.length}, 路径数量=${pathCount}`);
                
                if (mtpPruningData.ellipses.length !== pathCount && pathCount > 0) {
                    console.warn(`⚠️ 椭圆数量与路径数量不匹配！椭圆:${mtpPruningData.ellipses.length} vs 路径:${pathCount}`);
                }
                
                mtpPruningData.ellipses.forEach((ellipse, index) => {
                    console.log(`🎨 绘制椭圆${index + 1}: ${ellipse.source_id}→${ellipse.dest_id}`);
                    drawPruningEllipse(mtpCtx, ellipse);
                });
            }
            
            // 标注合并的目的节点
            if (mtpPruningData.merge_targets && mtpPruningData.merge_targets.length > 0) {
                console.log(`🎯 标注${mtpPruningData.merge_targets.length}个合并目标`);
                mtpPruningData.merge_targets.forEach(target => {
                    highlightMergeTarget(mtpCtx, target);
                });
            }
        } else {
            console.log('🚫 没有MTP剪枝数据，只显示UAV节点');
        }
        
        // 如果有当前路径，也在MTP canvas上显示
        if (staticExperimentPaths && staticExperimentPaths.length > 0) {
            drawMultiplePaths(mtpCtx, staticExperimentPaths);
        } else if (singleShortestPath) {
            drawPath(mtpCtx, singleShortestPath, 'rgba(255, 0, 0, 0.7)');
        }
        
        // 绘制图例
        drawMTPLegend(mtpCtx);
    }
    
    // 绘制MTP协议图例
    function drawMTPLegend(context) {
        const legendX = 10;
        const legendY = 10;
        const lineHeight = 25;
        let currentY = legendY;
        
        context.save();
        context.fillStyle = 'rgba(255, 255, 255, 0.9)';
        context.strokeStyle = '#ccc';
        context.lineWidth = 1;
        
        // 绘制图例背景
        const legendHeight = 180;
        context.fillRect(legendX - 5, legendY - 5, 200, legendHeight);
        context.strokeRect(legendX - 5, legendY - 5, 200, legendHeight);
        
        // 图例标题
        context.fillStyle = '#333';
        context.font = 'bold 14px Arial';
        context.fillText('MTP协议图例', legendX + 5, currentY + 15);
        currentY += 30;
        
        // 普通路径
        context.font = '12px Arial';
        context.strokeStyle = '#FF4136';
        context.lineWidth = 2;
        context.setLineDash([]);
        context.beginPath();
        context.moveTo(legendX, currentY);
        context.lineTo(legendX + 25, currentY);
        context.stroke();
        context.fillStyle = '#333';
        context.fillText('普通路径', legendX + 35, currentY + 4);
        currentY += lineHeight;
        
        // 合并路径
        context.strokeStyle = '#0074D9';
        context.lineWidth = 4;
        context.setLineDash([8, 4]);
        context.shadowColor = '#0074D9';
        context.shadowBlur = 2;
        context.beginPath();
        context.moveTo(legendX, currentY);
        context.lineTo(legendX + 25, currentY);
        context.stroke();
        context.shadowBlur = 0;
        context.fillText('合并路径（粗虚线）', legendX + 35, currentY + 4);
        currentY += lineHeight;
        
        // 合并连接线
        context.strokeStyle = 'rgba(255, 165, 0, 0.8)';
        context.lineWidth = 2;
        context.setLineDash([3, 3]);
        context.beginPath();
        context.moveTo(legendX, currentY);
        context.lineTo(legendX + 25, currentY);
        context.stroke();
        context.fillText('合并组连接', legendX + 35, currentY + 4);
        currentY += lineHeight;
        
        // 合并目标节点
        context.setLineDash([]);
        context.strokeStyle = 'red';
        context.lineWidth = 3;
        context.beginPath();
        context.arc(legendX + 12, currentY, 8, 0, Math.PI * 2);
        context.stroke();
        context.beginPath();
        context.arc(legendX + 12, currentY, 4, 0, Math.PI * 2);
        context.stroke();
        context.fillStyle = 'red';
        context.font = 'bold 10px Arial';
        context.textAlign = 'center';
        context.fillText('T', legendX + 12, currentY + 3);
        context.textAlign = 'start';
        context.fillStyle = '#333';
        context.font = '12px Arial';
        context.fillText('合并目标', legendX + 35, currentY + 4);
        currentY += lineHeight;
        
        // 椭圆剪枝区域
        context.strokeStyle = 'orange';
        context.lineWidth = 2;
        context.setLineDash([5, 5]);
        context.fillStyle = 'rgba(255, 165, 0, 0.1)';
        context.beginPath();
        context.ellipse(legendX + 12, currentY, 15, 8, 0, 0, Math.PI * 2);
        context.fill();
        context.stroke();
        context.fillStyle = '#333';
        context.fillText('椭圆剪枝区域', legendX + 35, currentY + 4);
        
        context.restore();
    }

    function drawGridBackground(context) {
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
                context.fillStyle = `rgba(${redVal}, ${greenVal}, 0, ${alpha})`;
                context.fillRect(c * cellWidth, r * cellHeight, cellWidth, cellHeight);
                context.fillStyle = '#555';
                context.font = '14px Arial';
                context.textAlign = 'center';
                context.textBaseline = 'middle';
                context.fillText(`${prr.toFixed(2)}`, c * cellWidth + cellWidth / 2, r * cellHeight + cellHeight / 2);
            }
        }
        context.textAlign = 'start';
        context.textBaseline = 'alphabetic';
    }

    function drawUAV(context, uav) {
        if (typeof uav.x !== 'number' || typeof uav.y !== 'number') return;
        context.beginPath();
        context.arc(uav.x, uav.y, UAV_RADIUS, 0, Math.PI * 2);
        context.fillStyle = uav.color || 'gray';
        context.fill();
        context.strokeStyle = 'black';
        context.lineWidth = 1;
        context.stroke();
        context.fillStyle = 'black';
        context.font = '12px Arial';
        context.fillText(uav.id, uav.x + UAV_RADIUS, uav.y - UAV_RADIUS);
    }

    function drawPacket(context, uav, packet) {
        const packetSize = 6;
        const packetOffsetX = -12;
        const packetOffsetY = -12;
        context.save();
        if (packet.status === 'delivered') context.fillStyle = 'lime';
        else if (packet.status.startsWith('failed')) context.fillStyle = 'black';
        else context.fillStyle = 'orange';
        context.fillRect(uav.x + packetOffsetX, uav.y + packetOffsetY, packetSize, packetSize);
        context.lineWidth = 1;
        context.strokeStyle = 'black';
        context.strokeRect(uav.x + packetOffsetX, uav.y + packetOffsetY, packetSize, packetSize);
        context.restore();
    }
    
    function drawMultiplePaths(context, pathsData) {
        const colors = ['#FF4136', '#0074D9', '#2ECC40', '#FFDC00', '#B10DC9', '#FF851B', '#7FDBFF', '#3D9970'];
        
        // 获取合并信息用于路径分组显示
        const mergeInfo = getMergePathGroups(pathsData);
        
        pathsData.forEach((pathInfo, index) => {
            const color = colors[index % colors.length];
            if (pathInfo.path) {
                // 检查是否为合并路径
                const isMergedPath = mergeInfo.mergedPaths.includes(index);
                const mergeGroupId = mergeInfo.pathToGroup[index];
                
                if (isMergedPath) {
                    // 合并路径用特殊样式：更粗的线条 + 虚线效果
                    drawMergedPath(context, pathInfo.path, color, mergeGroupId);
                } else {
                    // 普通路径
                    drawPath(context, pathInfo.path, color);
                }
            }
        });
        
        // 绘制合并组连接线
        drawMergeGroupConnections(context, mergeInfo.groups, pathsData);
    }

    function drawPath(context, pathNodeIds, color) {
        if (!pathNodeIds || pathNodeIds.length < 2) return;
        for (let i = 0; i < pathNodeIds.length - 1; i++) {
            const uav1 = uavMap.get(pathNodeIds[i]);
            const uav2 = uavMap.get(pathNodeIds[i+1]);
            if (uav1 && uav2) drawArrow(context, uav1.x, uav1.y, uav2.x, uav2.y, 10, color);
        }
    }

    // 获取合并路径分组信息
    function getMergePathGroups(pathsData) {
        const mergeInfo = {
            mergedPaths: [],      // 被合并的路径索引
            pathToGroup: {},      // 路径索引 -> 组ID映射
            groups: []            // 合并组信息
        };
        
        if (!mtpPruningData || !mtpPruningData.merge_targets) {
            return mergeInfo;
        }
        
        // 根据merge_targets创建合并组
        const groupMap = new Map(); // main_root_id -> group info
        
        mtpPruningData.merge_targets.forEach(target => {
            const mainRootId = target.main_root_id;
            if (!groupMap.has(mainRootId)) {
                groupMap.set(mainRootId, {
                    id: mainRootId,
                    mainRoot: mainRootId,
                    targets: [],
                    pathIndices: []
                });
            }
            groupMap.get(mainRootId).targets.push(target.uav_id);
        });
        
        // 找出对应的路径索引
        pathsData.forEach((pathInfo, index) => {
            const destination = pathInfo.destination;
            
            // 检查是否为合并组的目标节点
            for (const [groupId, group] of groupMap) {
                if (destination === group.mainRoot || group.targets.includes(destination)) {
                    group.pathIndices.push(index);
                    mergeInfo.pathToGroup[index] = groupId;
                    
                    if (group.targets.includes(destination)) {
                        mergeInfo.mergedPaths.push(index);
                    }
                }
            }
        });
        
        mergeInfo.groups = Array.from(groupMap.values()).filter(group => group.pathIndices.length > 1);
        
        console.log('🔗 合并路径分析:', mergeInfo);
        return mergeInfo;
    }
    
    // 绘制合并路径（特殊样式）
    function drawMergedPath(context, pathNodeIds, color, groupId) {
        if (!pathNodeIds || pathNodeIds.length < 2) return;
        
        context.save();
        context.setLineDash([8, 4]); // 虚线样式
        context.lineWidth = 4; // 更粗的线条
        context.shadowColor = color;
        context.shadowBlur = 3;
        
        for (let i = 0; i < pathNodeIds.length - 1; i++) {
            const uav1 = uavMap.get(pathNodeIds[i]);
            const uav2 = uavMap.get(pathNodeIds[i + 1]);
            if (uav1 && uav2) {
                drawArrow(context, uav1.x, uav1.y, uav2.x, uav2.y, 12, color);
            }
        }
        
        context.restore();
    }
    
    // 绘制合并组之间的连接线
    function drawMergeGroupConnections(context, groups, pathsData) {
        if (!groups || groups.length === 0) return;
        
        groups.forEach(group => {
            if (group.pathIndices.length < 2) return;
            
            context.save();
            context.strokeStyle = 'rgba(255, 165, 0, 0.6)'; // 橙色连接线
            context.lineWidth = 2;
            context.setLineDash([3, 3]);
            
            // 找到组内路径的目标节点
            const targetNodes = group.pathIndices.map(pathIndex => {
                const pathInfo = pathsData[pathIndex];
                return uavMap.get(pathInfo.destination);
            }).filter(node => node);
            
            // 在合并目标之间绘制连接线
            for (let i = 0; i < targetNodes.length - 1; i++) {
                const node1 = targetNodes[i];
                const node2 = targetNodes[i + 1];
                if (node1 && node2) {
                    context.beginPath();
                    context.moveTo(node1.x, node1.y);
                    context.lineTo(node2.x, node2.y);
                    context.stroke();
                }
            }
            
            context.restore();
        });
    }

    function drawArrow(context, fromX, fromY, toX, toY, arrowSize = 10, color = 'rgba(255, 0, 0, 0.7)') {
        const angle = Math.atan2(toY - fromY, toX - fromX);
        context.save();
        context.strokeStyle = color; 
        context.fillStyle = color; 
        context.lineWidth = context.lineWidth || 2; // 保持当前线宽
        context.beginPath(); 
        context.moveTo(fromX, fromY); 
        context.lineTo(toX, toY); 
        context.stroke();
        context.beginPath(); 
        context.moveTo(toX, toY);
        context.lineTo(toX - arrowSize * Math.cos(angle - Math.PI / 6), toY - arrowSize * Math.sin(angle - Math.PI / 6));
        context.lineTo(toX - arrowSize * Math.cos(angle + Math.PI / 6), toY - arrowSize * Math.sin(angle + Math.PI / 6));
        context.closePath(); 
        context.fill();
        context.restore();
    }

    // MTP协议特有的绘制函数
    function drawPruningEllipse(context, ellipse) {
        if (!ellipse || !ellipse.center_x || !ellipse.center_y || !ellipse.a || !ellipse.b) return;
        
        context.save();
        context.setLineDash([5, 5]); // 虚线
        context.strokeStyle = 'rgba(255, 165, 0, 0.8)'; // 橙色
        context.lineWidth = 2;
        
        context.beginPath();
        context.ellipse(
            ellipse.center_x, 
            ellipse.center_y,
            ellipse.a, // 长半轴
            ellipse.b, // 短半轴
            ellipse.rotation || 0, // 旋转角度（弧度）
            0, 
            2 * Math.PI
        );
        context.stroke();
        
        // 填充半透明区域
        context.fillStyle = 'rgba(255, 165, 0, 0.1)';
        context.fill();
        
        context.restore();
    }

    function highlightMergeTarget(context, target) {
        const uav = uavMap.get(target.uav_id);
        if (!uav) return;
        
        context.save();
        
        // 绘制目标节点的特殊标记（双环圈）
        context.strokeStyle = 'red';
        context.lineWidth = 3;
        context.setLineDash([]);
        
        // 外环
        context.beginPath();
        context.arc(uav.x, uav.y, UAV_RADIUS + 8, 0, Math.PI * 2);
        context.stroke();
        
        // 内环
        context.beginPath();
        context.arc(uav.x, uav.y, UAV_RADIUS + 4, 0, Math.PI * 2);
        context.stroke();
        
        // 添加"T"标记表示Target
        context.fillStyle = 'red';
        context.font = 'bold 10px Arial';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText('T', uav.x, uav.y + UAV_RADIUS + 20);
        
        context.restore();
    }

    // 获取MTP剪枝数据的函数（带重试机制）
    async function fetchMTPPruningData(retryCount = 0) {
        try {
            const pathCount = staticExperimentPaths ? staticExperimentPaths.length : 0;
            console.log(`🔍 [重试${retryCount}] 获取MTP剪枝数据，当前路径数量: ${pathCount}`);
            
            const result = await apiCall('simulation/mtp-pruning-data');
            if (result && result.ok && result.data) {
                mtpPruningData = result.data;
                const ellipseCount = mtpPruningData.ellipses?.length || 0;
                const targetCount = mtpPruningData.merge_targets?.length || 0;
                
                console.log('🔍 获取到MTP剪枝数据:', mtpPruningData);
                console.log('🔍 椭圆区域数量:', ellipseCount);
                console.log('🔍 合并目标数量:', targetCount);
                console.log(`🔍 数据匹配检查: 椭圆=${ellipseCount} vs 路径=${pathCount}`);
                
                // 检查椭圆数量是否与路径数量匹配
                if (pathCount > 0 && ellipseCount !== pathCount) {
                    console.warn(`⚠️ 椭圆数量(${ellipseCount})与路径数量(${pathCount})不匹配！`);
                }
                
                // 如果没有数据且重试次数少于3次，等待一下再重试
                if (ellipseCount === 0 && retryCount < 3) {
                    console.log(`🔄 椭圆数据为空，等待500ms后重试 (${retryCount + 1}/3)`);
                    await new Promise(resolve => setTimeout(resolve, 500));
                    return await fetchMTPPruningData(retryCount + 1);
                }
                
                if (ellipseCount === pathCount) {
                    console.log('✅ 椭圆数量与路径数量匹配！');
                }
            } else {
                console.warn("MTP剪枝数据为空或格式错误:", result);
            }
        } catch (error) {
            console.warn("无法获取MTP剪枝数据:", error);
        }
    }

    // 触发MTP剪枝和合并算法
    async function triggerMTPPruningAndMerging() {
        if (!staticExperimentPaths || staticExperimentPaths.length === 0) {
            console.warn("没有源-目标对，跳过剪枝");
            return;
        }

        try {
            // 提取所有目的节点
            const destinations = staticExperimentPaths.map(path => path.destination);
            
            console.log('🚀 触发MTP剪枝算法，目标节点:', destinations);
            console.log('🚀 源-目标对数据:', staticExperimentPaths);
            
            const result = await apiCall('simulation/trigger-mtp-pruning', 'POST', {
                destinations: destinations,
                source_dest_pairs: staticExperimentPaths
            });
            
            if (result && result.ok) {
                console.log('✅ MTP剪枝和合并算法执行完成:', result.data);
            } else {
                console.error('❌ MTP剪枝执行失败:', result.data?.error);
                console.error('❌ 完整响应:', result);
            }
        } catch (error) {
            console.error("❌ 触发MTP剪枝失败:", error);
        }
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
        
        // 获取配置信息以确定协议类型
        try {
            const configResult = await apiCall('simulation/config');
            if (configResult && configResult.ok && configResult.data) {
                currentProtocol = configResult.data.protocol || 'UNKNOWN';
                console.log(`检测到协议类型: ${currentProtocol}`);
            }
        } catch (error) {
            console.warn("无法获取协议配置，使用默认值", error);
            currentProtocol = 'MTP'; // 默认假设是MTP
        }
        
        // 如果是MTP协议，获取剪枝数据
        if (currentProtocol === 'MTP') {
            await fetchMTPPruningData();
        }
        
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
        
        // 清除MTP剪枝数据和相关状态
        mtpPruningData = null;
        console.log('🧹 开始/重置: 清除MTP剪枝数据');
        
        const result = await apiCall('simulation/start', 'POST', { num_uavs: parseInt(numUAVsInput.value) });
        if (result && result.ok && result.data) {
            updateUIFromState(result.data.current_state);
            displayMessage(result.data.status_message, false);
            clearAllPathDisplays();
            
            // 重置后不获取剪枝数据，因为还没有源-目标对
            console.log('✅ 仿真重置完成，MTP剪枝数据已清除');
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
        
        // 清除MTP剪枝数据
        mtpPruningData = null;
        console.log('🧹 clearAllPathDisplays: 清除MTP剪枝数据');
        
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
            
            // 如果是MTP协议，立即执行剪枝和合并算法
            if (currentProtocol === 'MTP') {
                displayMessage("正在执行MTP剪枝和合并算法...", false);
                
                // 按顺序执行：剪枝 -> 获取数据 -> 刷新状态 -> 强制重绘
                await triggerMTPPruningAndMerging();
                await fetchMTPPruningData();
                await refreshFullStateAndRedraw();
                
                // 强制重绘两个canvas
                redrawCanvas();
                
                // 确保UI反馈
                console.log('🎨 MTP剪枝完成，强制重绘canvas');
                displayMessage("MTP剪枝和合并完成，对照组已更新", false);
            } else {
                // 非MTP协议，只需刷新状态
                await refreshFullStateAndRedraw();
            }
            
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
