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
    let currentProtocol = 'MTP'; // 当前使用的协议，默认MTP避免刷新时布局闪烁
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
        
        // 保存当前状态
        ctx.save();
        
        // 清除canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // 应用缩放变换（非MTP模式）
        if (currentProtocol !== 'MTP') {
            const scale = getScale(canvas);
            ctx.scale(scale, scale);
        }
        
        // 只有当currentGridConfig存在时才绘制网格背景（PTP协议）
        if (currentGridConfig) drawGridBackground(ctx);
        if (currentUAVs) currentUAVs.forEach(uav => drawUAV(ctx, uav));
        if (staticExperimentPaths && staticExperimentPaths.length > 0) {
            // 原始拓扑：所有路径都用普通方式绘制（不显示合并标记）
            drawMultiplePathsSimple(ctx, staticExperimentPaths);
        } else if (singleShortestPath) {
            drawPathSimple(ctx, singleShortestPath, 'rgba(255, 0, 0, 0.7)');
        }
        if (currentPackets) currentPackets.forEach(packet => {
            const holderUAV = uavMap.get(packet.current_holder_id);
            if (holderUAV) drawPacket(ctx, holderUAV, packet);
        });
        
        // 恢复状态
        ctx.restore();
        
        // 如果是MTP协议，同时更新MTP对照组canvas
        if (currentProtocol === 'MTP' && mtpCtx) {
            redrawMTPCanvas();
        }
    }

    // 添加canvas显示控制函数
    function updateCanvasDisplay() {
        if (!canvasContainer || !mtpCanvasPanel) return;
        
        if (currentProtocol === 'MTP') {
            // MTP模式：显示双canvas布局，添加mtp-mode类
            document.body.classList.add('mtp-mode');
            canvasContainer.classList.remove('single-canvas');
            mtpCanvasPanel.style.display = 'flex';
            // MTP模式：固定尺寸600x600
            setCanvasSize(canvas, 600, 600);
            if (mtpCanvas) setCanvasSize(mtpCanvas, 600, 600);
        } else {
            // 其他协议：显示单canvas布局，移除mtp-mode类
            document.body.classList.remove('mtp-mode');
            canvasContainer.classList.add('single-canvas');
            mtpCanvasPanel.style.display = 'none';
            // 其他协议：动态尺寸，根据容器大小调整
            updateCanvasSizeToContainer();
        }
    }

    // 设置canvas的实际绘制尺寸
    function setCanvasSize(canvasElement, width, height) {
        if (!canvasElement) return;
        canvasElement.width = width;
        canvasElement.height = height;
    }

    // 根据容器大小更新canvas尺寸（非MTP模式）
    function updateCanvasSizeToContainer() {
        if (currentProtocol === 'MTP') return; // MTP模式不调整
        
        const canvasPanel = canvas.closest('.canvas-panel');
        if (!canvasPanel) return;
        
        // 获取容器的实际宽度
        const containerWidth = canvasPanel.clientWidth;
        // 减去padding和border
        const actualWidth = Math.max(300, containerWidth - 20); // 最小300px
        const actualHeight = actualWidth; // 保持1:1比例
        
        setCanvasSize(canvas, actualWidth, actualHeight);
        
        // 重绘canvas内容
        redrawCanvas();
    }

    // 监听窗口大小变化，动态调整canvas（仅非MTP模式）
    window.addEventListener('resize', () => {
        if (currentProtocol !== 'MTP') {
            updateCanvasSizeToContainer();
        }
    });

    // MTP对照组canvas绘制函数
    function redrawMTPCanvas() {
        if (!mtpCtx) return;
        
        mtpCtx.clearRect(0, 0, mtpCanvas.width, mtpCanvas.height);
        
        // 绘制所有UAV节点（与原canvas保持一致的位置）
        if (currentUAVs) {
            currentUAVs.forEach(uav => drawUAV(mtpCtx, uav));
        }
        
        // 只有在有MTP剪枝数据时才绘制椭圆区域
        if (mtpPruningData && mtpPruningData.ellipses && mtpPruningData.ellipses.length > 0) {
            mtpPruningData.ellipses.forEach((ellipse, index) => {
                console.log(`🎨 绘制椭圆${index + 1}: ${ellipse.source_id}→${ellipse.dest_id}`);
                drawPruningEllipse(mtpCtx, ellipse);
            });
        }
        
        // 如果有当前路径，也在MTP canvas上显示
        if (staticExperimentPaths && staticExperimentPaths.length > 0) {
            drawMultiplePaths(mtpCtx, staticExperimentPaths);
        } else if (singleShortestPath) {
            drawPath(mtpCtx, singleShortestPath, 'rgba(255, 0, 0, 0.7)');
        }
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

    // 获取缩放比例（后端坐标基于600x600）
    function getScale(canvasElement) {
        if (!canvasElement) return 1;
        if (currentProtocol === 'MTP') return 1; // MTP模式不缩放
        return canvasElement.width / 600; // 基于600的缩放比例
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
    
    // 简单版本：原始拓扑用，所有路径都用实线绘制
    function drawMultiplePathsSimple(context, pathsData) {
        const colors = ['#FF4136', '#0074D9', '#2ECC40', '#FFDC00', '#B10DC9', '#FF851B', '#7FDBFF', '#3D9970'];
        
        pathsData.forEach((pathInfo, index) => {
            const color = colors[index % colors.length];
            if (pathInfo.path) {
                drawPathSimple(context, pathInfo.path, color);
            }
        });
    }

    // 完整版本：MTP对照组用，基于后端真实合并数据显示
    function drawMultiplePaths(context, pathsData) {
        const colors = ['#FF4136', '#0074D9', '#2ECC40', '#FFDC00', '#B10DC9', '#FF851B', '#7FDBFF', '#3D9970'];
        
        // 调试：显示所有路径的目标节点
        console.log('📍 所有路径的目标节点:', pathsData.map(p => p.destination));
        
        // 获取合并组中的目标节点集合
        const mergedDestinations = new Set();
        const mergedGroups = [];
        if (mtpPruningData && mtpPruningData.tree_groups) {
            console.log('📦 后端返回的所有分组:', mtpPruningData.tree_groups);
            
            mtpPruningData.tree_groups.forEach((group, idx) => {
                if (group.length > 1) {
                    mergedGroups.push(group);
                    group.forEach(nodeId => mergedDestinations.add(nodeId));
                    console.log(`  合并组${idx}: [${group.join(', ')}] (${group.length}个节点)`);
                } else {
                    console.log(`  单节点组${idx}: [${group[0]}]`);
                }
            });
            console.log('🔍 合并组节点集合:', Array.from(mergedDestinations));
        }
        
        // 检查哪些合并组节点有路径，哪些没有
        const destinationsSet = new Set(pathsData.map(p => p.destination));
        const missingNodes = Array.from(mergedDestinations).filter(nodeId => !destinationsSet.has(nodeId));
        if (missingNodes.length > 0) {
            console.warn('⚠️ 以下合并组节点没有对应的路径:', missingNodes);
        }
        
        // 绘制所有路径
        let mergedCount = 0;
        pathsData.forEach((pathInfo, index) => {
            const color = colors[index % colors.length];
            if (pathInfo.path) {
                // 检查路径的目标节点是否在合并组中
                const isMergedPath = mergedDestinations.has(pathInfo.destination);
                
                if (isMergedPath) {
                    // 合并组相关路径：粗虚线
                    mergedCount++;
                    console.log(`✅ 路径${index}: 源${pathInfo.source}→目标${pathInfo.destination} (合并组路径)`);
                    drawPathThickSolid(context, pathInfo.path, color);
                } else {
                    // 普通路径：细虚线
                    drawPath(context, pathInfo.path, color, false, false, null);
                }
            }
        });
        
        console.log(`📊 总路径数: ${pathsData.length}, 合并组路径: ${mergedCount}, 合并组节点: ${mergedDestinations.size}`);
        
        // 如果有后端返回的合并组数据，绘制虚拟根节点
        if (mtpPruningData && mtpPruningData.tree_groups) {
            drawVirtualRootMarkersFromBackend(context, mtpPruningData.tree_groups);
        }
    }
    
    // 绘制粗虚线路径（用于合并组相关路径）
    function drawPathThickSolid(context, pathNodeIds, color) {
        if (!pathNodeIds || pathNodeIds.length < 2) return;
        
        context.save();
        context.setLineDash([10, 6]); // 虚线
        const lineWidth = 4; // 粗线
        const arrowSize = 12;
        
        for (let i = 0; i < pathNodeIds.length - 1; i++) {
            const uav1 = uavMap.get(pathNodeIds[i]);
            const uav2 = uavMap.get(pathNodeIds[i+1]);
            if (uav1 && uav2) {
                drawArrow(context, uav1.x, uav1.y, uav2.x, uav2.y, arrowSize, color, lineWidth);
            }
        }
        
        context.restore();
    }
    
    function drawVirtualRootMarkersFromBackend(context, treeGroups) {
        // 基于后端返回的virtual_roots信息绘制虚拟根节点
        if (!mtpPruningData || !mtpPruningData.virtual_roots) {
            return;
        }
        
        mtpPruningData.virtual_roots.forEach((rootInfo) => {
            const group = rootInfo.group_nodes;
            if (group.length <= 1) return;
            
            // 获取虚拟根节点的位置（后端选择的实际节点）
            const virtualRootUav = uavMap.get(rootInfo.virtual_root_id);
            if (!virtualRootUav) return;
            
            const virtualRootX = virtualRootUav.x;
            const virtualRootY = virtualRootUav.y;
            
            // 获取组内所有节点的位置（用于绘制连接线）
            const nodePositions = group.map(nodeId => {
                const uav = uavMap.get(nodeId);
                return uav ? { x: uav.x, y: uav.y, id: nodeId } : null;
            }).filter(pos => pos !== null);
            
            if (nodePositions.length < 2) return;
            
            context.save();
            
            // 绘制从虚拟根到各节点的虚线连接
            context.setLineDash([5, 5]);
            context.strokeStyle = 'rgba(138, 43, 226, 0.4)';
            context.lineWidth = 1.5;
            
            nodePositions.forEach(pos => {
                context.beginPath();
                context.moveTo(virtualRootX, virtualRootY);
                context.lineTo(pos.x, pos.y);
                context.stroke();
            });
            
            // 绘制虚拟根节点标记
            context.setLineDash([]);
            
            // 外圈光晕
            context.fillStyle = 'rgba(255, 87, 34, 0.2)';
            context.beginPath();
            context.arc(virtualRootX, virtualRootY, 6.5, 0, Math.PI * 2);
            context.fill();
            
            // 中心大圆
            context.fillStyle = 'rgba(255, 87, 34, 0.95)';
            context.beginPath();
            context.arc(virtualRootX, virtualRootY, 6, 0, Math.PI * 2);
            context.fill();
            
            // 白色边框
            context.strokeStyle = 'white';
            context.lineWidth = 1;
            context.stroke();
            
            // 绘制"V"字标记
            context.fillStyle = 'white';
            context.font = 'bold 6px Arial';
            context.textAlign = 'center';
            context.textBaseline = 'middle';
            context.fillText('V', virtualRootX, virtualRootY);
            
            context.restore();
        });
    }
    

    // 简单版本：只绘制普通实线路径，无特殊标记
    function drawPathSimple(context, pathNodeIds, color) {
        if (!pathNodeIds || pathNodeIds.length < 2) return;
        
        for (let i = 0; i < pathNodeIds.length - 1; i++) {
            const uav1 = uavMap.get(pathNodeIds[i]);
            const uav2 = uavMap.get(pathNodeIds[i + 1]);
            if (uav1 && uav2) {
                drawArrowSimple(context, uav1.x, uav1.y, uav2.x, uav2.y, 10, color, 2);
            }
        }
    }
    
    // 完整版本：对照组中所有路径默认虚线，合并路径特殊处理
    function drawPath(context, pathNodeIds, color, isMerged = false, isPrimaryPath = false, groupInfo = null) {
        if (!pathNodeIds || pathNodeIds.length < 2) return;
        
        context.save();
        
        if (isMerged) {
            // 合并路径组
            if (isPrimaryPath) {
                // 主路径：使用实线，正常粗细
                context.setLineDash([]);
                const lineWidth = 3.5;
                const arrowSize = 11;
                
                for (let i = 0; i < pathNodeIds.length - 1; i++) {
                    const uav1 = uavMap.get(pathNodeIds[i]);
                    const uav2 = uavMap.get(pathNodeIds[i+1]);
                    if (uav1 && uav2) {
                        drawArrow(context, uav1.x, uav1.y, uav2.x, uav2.y, arrowSize, color, lineWidth);
                    }
                }
            } else {
                // 被合并路径：使用虚线，加粗加深
                context.setLineDash([10, 6]);
                const lineWidth = 4.5;
                const arrowSize = 12;
                
                // 颜色加深（降低透明度）
                const deepColor = color.includes('rgb') 
                    ? color.replace(')', ', 0.9)').replace('rgb', 'rgba')
                    : color;
                
                for (let i = 0; i < pathNodeIds.length - 1; i++) {
                    const uav1 = uavMap.get(pathNodeIds[i]);
                    const uav2 = uavMap.get(pathNodeIds[i+1]);
                    if (uav1 && uav2) {
                        drawArrow(context, uav1.x, uav1.y, uav2.x, uav2.y, arrowSize, deepColor, lineWidth);
                    }
                }
            }
        } else {
            // 非合并路径：对照组中使用细虚线
            context.setLineDash([8, 4]);
            const lineWidth = 2;
            const arrowSize = 8;
            
            for (let i = 0; i < pathNodeIds.length - 1; i++) {
                const uav1 = uavMap.get(pathNodeIds[i]);
                const uav2 = uavMap.get(pathNodeIds[i+1]);
                if (uav1 && uav2) {
                    drawArrow(context, uav1.x, uav1.y, uav2.x, uav2.y, arrowSize, color, lineWidth);
                }
            }
        }
        
        context.restore();
    }
    

    // 简单版本：普通实线箭头
    function drawArrowSimple(context, fromX, fromY, toX, toY, arrowSize = 10, color = 'rgba(255, 0, 0, 0.7)', lineWidth = 2) {
        const angle = Math.atan2(toY - fromY, toX - fromX);
        context.save();
        context.strokeStyle = color; 
        context.fillStyle = color; 
        context.lineWidth = lineWidth;
        context.setLineDash([]);
        
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
    
    // 完整版本：支持虚线
    function drawArrow(context, fromX, fromY, toX, toY, arrowSize = 10, color = 'rgba(255, 0, 0, 0.7)', lineWidth = 2) {
        const angle = Math.atan2(toY - fromY, toX - fromX);
        context.save();
        context.strokeStyle = color; 
        context.fillStyle = color; 
        context.lineWidth = lineWidth;
        
        // 绘制线条（保持当前的lineDash设置）
        context.beginPath(); 
        context.moveTo(fromX, fromY); 
        context.lineTo(toX, toY); 
        context.stroke();
        
        // 绘制箭头（始终用实线）
        context.setLineDash([]);
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
        
        // 立即应用协议对应的布局样式
        updateCanvasDisplay();
        
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

    // 页面加载时立即应用默认协议的布局样式（避免刷新时布局闪烁）
    updateCanvasDisplay();
    
    initializeApp();
});
