# 树剪枝机制实现说明

## 概述

根据图片中描述的树剪枝机制，我们在当前UAV仿真项目中实现了基于椭圆区域的树剪枝优化算法。该机制通过动态构建针对特定源-目标对的优化传输树，减少不必要的树结构，从而降低能耗成本。

## 核心思想

1. **椭圆区域定义**：以源节点和目标节点为椭圆的两个焦点
2. **动态树构建**：只为椭圆区域内的节点构建传输树
3. **ETX管理**：椭圆区域内的节点定期更新ETX，区域外的节点不更新

## 实现细节

### 1. 配置参数 (simulation_config.py)

```python
# 树剪枝机制配置参数
TREE_PRUNING_ENABLED = True  # 是否启用树剪枝机制
ELLIPSE_ECCENTRICITY = 0.8   # 椭圆偏心率 (0 < e < 1)
ELLIPSE_EXPANSION_FACTOR = 1.2  # 椭圆扩展因子，用于扩大搜索范围
PRUNING_UPDATE_INTERVAL = 0.5  # ETX更新间隔时间(秒)
ELLIPSE_BOUNDARY_TOLERANCE = 5.0  # 椭圆边界容差(米)
```

### 2. UAV类增强 (core/uav.py)

#### 椭圆区域判断方法
```python
def is_within_ellipse_region(self, source_uav, destination_uav):
    """
    判断当前UAV是否在源节点和目标节点定义的椭圆区域内
    """
    # 计算椭圆参数
    focal_distance = 源节点到目标节点的距离
    a = focal_distance / (2 * ELLIPSE_ECCENTRICITY)  # 半长轴
    b = a * sqrt(1 - ELLIPSE_ECCENTRICITY^2)  # 半短轴
    
    # 椭圆定义：到两个焦点距离之和 <= 2a
    return (dist_to_source + dist_to_destination) <= 2a
```

#### 效用值计算
```python
def calculate_ellipse_utility(self, source_uav, destination_uav):
    """
    计算当前节点在椭圆区域内的效用值，用于树构建时的优先级排序
    """
    # 使用调和平均数来平衡两个距离
    utility = 2.0 / (dist_to_source + dist_to_destination)
    return utility
```

### 3. MTP协议增强 (protocols/mtp_protocol.py)

#### 剪枝ETX更新机制
```python
def update_etx_with_pruning(self, source_id, destination_id, sim_time):
    """
    基于树剪枝机制的ETX更新方法
    只有椭圆区域内的节点才更新ETX，区域外的节点不更新
    """
    for node_id, node in self.uav_map.items():
        if node.is_within_ellipse_region(source_uav, destination_uav):
            # 节点在椭圆区域内，更新ETX
            self._update_node_etx(node, destination_id)
        else:
            # 节点在椭圆区域外，不更新ETX，标记为被剪枝
            self.pruned_nodes.add(node_id)
```

#### 剪枝树构建
```python
def build_pruned_tree_for_pair(self, source_id, destination_id):
    """
    为特定的源-目标对构建剪枝后的树结构
    """
    # 只考虑椭圆区域内的节点进行树构建
    for neighbor in self._get_neighbors(current_uav):
        if neighbor.is_within_ellipse_region(source_uav, destination_uav):
            # 添加到树结构中
```

### 4. DHyTP协议增强 (protocols/dhytp_protocol.py)

DHyTP协议集成了与MTP相同的树剪枝机制，包括：
- `update_etx_with_pruning()` - 剪枝ETX更新
- `build_pruned_tree_for_pair_dhytp()` - 剪枝树构建
- `get_pruned_neighbors_dhytp()` - 获取剪枝邻居

## 工作机制

### 1. 椭圆区域计算
- 以源节点和目标节点为椭圆的两个焦点
- 焦点距离 = 源节点到目标节点的欧几里得距离
- 椭圆半长轴 a = 焦点距离 / (2 × 偏心率)
- 椭圆半短轴 b = a × √(1 - 偏心率²)

### 2. 节点筛选
- 计算每个节点到两个焦点的距离之和
- 如果距离之和 ≤ 2a + 边界容差，则节点在椭圆区域内
- 椭圆区域内的节点参与树构建和ETX更新
- 椭圆区域外的节点被剪枝，不参与树构建

### 3. 动态更新
- 每隔 `PRUNING_UPDATE_INTERVAL` 时间间隔更新ETX
- 只有椭圆区域内的节点更新ETX值
- 区域外的节点保持原有ETX值不变

## 优势

1. **能耗优化**：减少不必要的树结构，降低树构建和维护的能耗
2. **计算效率**：只对相关节点进行ETX计算，减少计算开销
3. **网络优化**：专注于源-目标对之间的最优路径，提高传输效率
4. **动态适应**：根据源-目标对动态调整树结构

## 使用方式

1. 在 `simulation_config.py` 中设置 `TREE_PRUNING_ENABLED = True` 启用树剪枝
2. 调整椭圆参数（偏心率、扩展因子等）以适应不同的网络场景
3. 协议会自动使用剪枝机制进行树构建和ETX更新

## 配置建议

- **偏心率 (0.7-0.9)**：值越大，椭圆越扁，剪枝越激进
- **扩展因子 (1.1-1.5)**：值越大，椭圆区域越大，包含更多节点
- **更新间隔 (0.3-1.0秒)**：根据网络动态性调整
- **边界容差 (3-10米)**：考虑节点移动性的容错范围

这个实现完全符合图片中描述的树剪枝机制，通过椭圆区域限制和动态ETX管理，实现了高效的树构建优化。
















