# DWAPCBC路径规划系统使用指南

## 目录

1. [快速入门](#快速入门)
2. [命令行工具详解](#命令行工具详解)
3. [场景配置指南](#场景配置指南)
4. [参数调优技巧](#参数调优技巧)
5. [批量测试](#批量测试)
6. [Python API使用](#python-api使用)
7. [实战案例](#实战案例)

---

## 快速入门

### 第一步：测试预设场景

```bash
cd /home/ubuntu/dwapcbc-system/backend

# 运行示例场景1
python3 cli.py -f scenarios/example1.json --no-viz
```

**预期输出**：
```
============================================================
开始路径规划...
============================================================
起点: (5.0, 5.0)
终点: (95.0, 95.0)
障碍物数量: 3
环境大小: 100 x 100
------------------------------------------------------------

============================================================
✓ 路径规划成功!
============================================================

【路径信息】
  初始路径点数: 230
  优化后路径点数: 8
  最终轨迹点数: 98

【性能指标】
  安全距离: 2.3362 m
  平滑度: 159.1895°
  路径长度: 138.2430 m
  计算时间: 1.2217 s
  迭代次数: 1

【环境信息】
  环境复杂度: 0.1106
  障碍物数量: 3
============================================================
```

### 第二步：生成可视化结果

```bash
# 生成PNG图像
python3 cli.py -f scenarios/example1.json -o my_result.png --no-viz

# 查看生成的图像
ls -lh my_result.png
```

### 第三步：尝试不同场景

```bash
# 高障碍物密度场景
python3 cli.py -f scenarios/example2.json --no-viz

# 简单环境场景
python3 cli.py -f scenarios/example3.json --no-viz
```

---

## 命令行工具详解

### 基本语法

```bash
python3 cli.py [选项]
```

### 选项说明

| 选项 | 简写 | 说明 | 示例 |
|------|------|------|------|
| `--file` | `-f` | 场景配置文件路径 | `-f scenarios/example1.json` |
| `--interactive` | `-i` | 交互式模式 | `-i` |
| `--output` | `-o` | 输出图像路径 | `-o result.png` |
| `--no-viz` | - | 不显示可视化窗口 | `--no-viz` |
| `--w1` | - | 安全性权重 | `--w1 0.7` |
| `--w2` | - | 平滑性权重 | `--w2 0.3` |
| `--safety-margin` | - | 安全裕度 | `--safety-margin 2.5` |
| `--curvature-threshold` | - | 曲率阈值 | `--curvature-threshold 0.08` |
| `--max-iterations` | - | 最大迭代次数 | `--max-iterations 100` |

### 使用示例

#### 1. 基本使用

```bash
# 使用JSON配置文件
python3 cli.py -f scenarios/example1.json --no-viz
```

#### 2. 保存结果

```bash
# 保存为PNG图像
python3 cli.py -f scenarios/example1.json -o results/test1.png --no-viz
```

#### 3. 覆盖参数

```bash
# 安全优先模式
python3 cli.py -f scenarios/example1.json \
  --w1 0.7 --w2 0.3 \
  --safety-margin 2.5 \
  --no-viz

# 平滑优先模式
python3 cli.py -f scenarios/example1.json \
  --w1 0.4 --w2 0.6 \
  --safety-margin 1.5 \
  --no-viz
```

#### 4. 交互式模式

```bash
python3 cli.py -i
```

**交互流程**：
```
请输入起点坐标:
  X: 5
  Y: 5

请输入终点坐标:
  X: 95
  Y: 95

请输入障碍物数量:
  数量: 2

障碍物 1:
  类型 (rectangle/circle): rectangle
  请输入4个顶点坐标 (x y):
    顶点1: 20 20
    顶点2: 20 40
    顶点3: 40 40
    顶点4: 40 20

障碍物 2:
  类型 (rectangle/circle): circle
  圆心X: 60
  圆心Y: 60
  半径: 8
```

---

## 场景配置指南

### JSON配置文件结构

```json
{
  "name": "场景名称",
  "description": "场景描述",
  "start": {
    "x": 起点X坐标,
    "y": 起点Y坐标
  },
  "goal": {
    "x": 终点X坐标,
    "y": 终点Y坐标
  },
  "environment": {
    "width": 环境宽度,
    "height": 环境高度,
    "resolution": 网格分辨率
  },
  "obstacles": [
    障碍物列表
  ],
  "parameters": {
    算法参数
  }
}
```

### 障碍物配置

#### 矩形障碍物

```json
{
  "type": "rectangle",
  "vertices": [
    [x1, y1],
    [x2, y2],
    [x3, y3],
    [x4, y4]
  ]
}
```

**注意**：顶点需要按顺序（顺时针或逆时针）排列。

#### 圆形障碍物

```json
{
  "type": "circle",
  "center": [cx, cy],
  "radius": r
}
```

#### 多边形障碍物

```json
{
  "type": "polygon",
  "vertices": [
    [x1, y1],
    [x2, y2],
    [x3, y3],
    ...
  ]
}
```

### 完整示例

```json
{
  "name": "办公室导航场景",
  "description": "模拟办公室环境中的机器人导航",
  "start": {"x": 5, "y": 5},
  "goal": {"x": 95, "y": 95},
  "environment": {
    "width": 100,
    "height": 100,
    "resolution": 0.5
  },
  "obstacles": [
    {
      "type": "rectangle",
      "vertices": [[20, 30], [20, 50], [30, 50], [30, 30]]
    },
    {
      "type": "rectangle",
      "vertices": [[50, 20], [50, 40], [60, 40], [60, 20]]
    },
    {
      "type": "circle",
      "center": [70, 70],
      "radius": 10
    }
  ],
  "parameters": {
    "w1": 0.6,
    "w2": 0.4,
    "safety_margin": 2.0,
    "curvature_threshold": 0.1,
    "max_iterations": 50
  }
}
```

---

## 参数调优技巧

### 权重参数（w1, w2）

**w1 + w2 = 1.0**

#### 安全优先（w1 = 0.7, w2 = 0.3）

**适用场景**：
- 高障碍物密度环境
- 狭窄通道
- 对安全性要求极高的场景

**效果**：
- ✅ 更大的安全距离
- ❌ 可能路径较长
- ❌ 可能有更多转角

#### 平滑优先（w1 = 0.4, w2 = 0.6）

**适用场景**：
- 开阔环境
- 低障碍物密度
- 对运动平滑性要求高的场景

**效果**：
- ✅ 更平滑的轨迹
- ✅ 更少的急转弯
- ❌ 可能安全距离较小

#### 平衡模式（w1 = 0.6, w2 = 0.4）

**适用场景**：
- 一般应用场景
- 中等复杂度环境

**效果**：
- ✅ 安全性和平滑性兼顾
- ✅ 适应性强

### 安全裕度（safety_margin）

| 值 | 适用场景 | 效果 |
|---|---------|------|
| 0.5-1.0 m | 精确导航，空间受限 | 可能无法找到路径 |
| 1.5-2.0 m | 一般应用 | 推荐值 |
| 2.5-3.0 m | 高安全要求 | 路径可能较长 |
| 3.0+ m | 极高安全要求 | 可能无可行路径 |

### 曲率阈值（curvature_threshold）

| 值 | 效果 |
|---|------|
| 0.05-0.08 | 识别更多高曲率点，路径点更密集 |
| 0.10-0.12 | 平衡值 |
| 0.15+ | 识别较少高曲率点，路径点稀疏 |

### 最大迭代次数（max_iterations）

| 值 | 适用场景 |
|---|---------|
| 10-30 | 快速规划，对质量要求不高 |
| 50 | 推荐值 |
| 100+ | 追求极致优化 |

### 调优流程

1. **从默认参数开始**
   ```bash
   python3 cli.py -f your_scenario.json --no-viz
   ```

2. **分析结果**
   - 安全距离是否足够？
   - 平滑度是否满意？
   - 计算时间是否可接受？

3. **调整参数**
   ```bash
   # 如果安全距离不够
   python3 cli.py -f your_scenario.json \
     --w1 0.7 --safety-margin 2.5 --no-viz
   
   # 如果路径不够平滑
   python3 cli.py -f your_scenario.json \
     --w2 0.6 --curvature-threshold 0.08 --no-viz
   ```

4. **迭代优化**
   - 记录每次调整的结果
   - 逐步逼近最优参数

---

## 批量测试

### 基本用法

```bash
# 批量运行scenarios目录下的所有场景
python3 batch_test.py -d scenarios -o batch_results
```

### 输出文件

1. **test_results.csv** - CSV格式的详细结果
2. **summary.txt** - 文本格式的统计摘要
3. **comparison_charts.png** - 可视化对比图表

### 查看结果

```bash
# 查看摘要
cat batch_results/summary.txt

# 查看CSV（使用pandas）
python3 -c "import pandas as pd; print(pd.read_csv('batch_results/test_results.csv'))"
```

### 自定义批量测试

```bash
# 指定不同的场景目录
python3 batch_test.py -d my_scenarios -o my_results

# 测试特定场景集合
mkdir test_set
cp scenarios/example1.json test_set/
cp scenarios/example2.json test_set/
python3 batch_test.py -d test_set -o test_results
```

---

## Python API使用

### 基本示例

```python
from dwapcbc import DWAPCBCPlanner, DWAPCBCConfig

# 1. 创建配置
config = DWAPCBCConfig(
    environment_width=100,
    environment_height=100,
    grid_resolution=0.5,
    w1_safety=0.6,
    w2_smoothness=0.4,
    safety_margin=2.0
)

# 2. 创建规划器
planner = DWAPCBCPlanner(config)

# 3. 设置障碍物
obstacles = [
    {
        'type': 'rectangle',
        'vertices': [[20, 20], [20, 40], [40, 40], [40, 20]]
    },
    {
        'type': 'circle',
        'center': [60, 60],
        'radius': 8
    }
]
planner.set_obstacles(obstacles)

# 4. 执行规划
result = planner.plan((5, 5), (95, 95))

# 5. 处理结果
if result.success:
    print(f"规划成功!")
    print(f"安全距离: {result.safety_distance:.2f} m")
    print(f"平滑度: {result.smoothness:.2f}°")
    print(f"路径长度: {result.path_length:.2f} m")
    print(f"计算时间: {result.computation_time:.3f} s")
    
    # 获取轨迹点
    trajectory = result.final_trajectory
    print(f"轨迹点数: {len(trajectory)}")
else:
    print("规划失败")
```

### 高级用法

#### 1. 访问中间结果

```python
result = planner.plan((5, 5), (95, 95))

if result.success:
    # 初始路径
    initial_path = result.initial_path
    
    # 优化后的路径点
    waypoints = result.optimized_waypoints
    
    # 贝塞尔曲线对象
    bezier = result.bezier_curve
    
    # 计算特定位置的曲率
    curvature = bezier.compute_curvature_at(0, 0.5)
```

#### 2. 自定义可视化

```python
import matplotlib.pyplot as plt

result = planner.plan((5, 5), (95, 95))

if result.success:
    # 提取轨迹坐标
    x = [p[0] for p in result.final_trajectory]
    y = [p[1] for p in result.final_trajectory]
    
    # 绘图
    plt.figure(figsize=(10, 10))
    plt.plot(x, y, 'b-', linewidth=2, label='规划路径')
    plt.plot(5, 5, 'go', markersize=10, label='起点')
    plt.plot(95, 95, 'rs', markersize=10, label='终点')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('custom_plot.png')
```

#### 3. 批量规划

```python
import json

# 加载多个场景
scenarios = []
for i in range(1, 4):
    with open(f'scenarios/example{i}.json', 'r') as f:
        scenarios.append(json.load(f))

# 批量规划
results = []
for scenario in scenarios:
    planner = DWAPCBCPlanner(DWAPCBCConfig())
    planner.set_obstacles(scenario['obstacles'])
    
    start = tuple(scenario['start'].values())
    goal = tuple(scenario['goal'].values())
    
    result = planner.plan(start, goal)
    results.append(result)

# 统计分析
avg_safety = sum(r.safety_distance for r in results if r.success) / len(results)
print(f"平均安全距离: {avg_safety:.2f} m")
```

---

## 实战案例

### 案例1：仓库AGV导航

**场景描述**：
- 100m × 100m仓库
- 多个货架障碍物
- AGV需要从入口到达指定货位

**配置文件**：

```json
{
  "name": "仓库AGV导航",
  "start": {"x": 5, "y": 50},
  "goal": {"x": 95, "y": 50},
  "environment": {
    "width": 100,
    "height": 100,
    "resolution": 0.5
  },
  "obstacles": [
    {"type": "rectangle", "vertices": [[20, 30], [20, 70], [25, 70], [25, 30]]},
    {"type": "rectangle", "vertices": [[40, 30], [40, 70], [45, 70], [45, 30]]},
    {"type": "rectangle", "vertices": [[60, 30], [60, 70], [65, 70], [65, 30]]},
    {"type": "rectangle", "vertices": [[80, 30], [80, 70], [85, 70], [85, 30]]}
  ],
  "parameters": {
    "w1": 0.7,
    "w2": 0.3,
    "safety_margin": 2.5,
    "curvature_threshold": 0.08,
    "max_iterations": 50
  }
}
```

**运行**：
```bash
python3 cli.py -f warehouse_agv.json -o warehouse_result.png --no-viz
```

### 案例2：室内服务机器人

**场景描述**：
- 办公室环境
- 桌椅等不规则障碍物
- 需要平滑运动避免打扰

**配置文件**：

```json
{
  "name": "办公室服务机器人",
  "start": {"x": 10, "y": 10},
  "goal": {"x": 90, "y": 90},
  "environment": {
    "width": 100,
    "height": 100,
    "resolution": 0.5
  },
  "obstacles": [
    {"type": "circle", "center": [30, 30], "radius": 8},
    {"type": "circle", "center": [50, 50], "radius": 10},
    {"type": "circle", "center": [70, 70], "radius": 6},
    {"type": "rectangle", "vertices": [[40, 20], [40, 30], [60, 30], [60, 20]]}
  ],
  "parameters": {
    "w1": 0.5,
    "w2": 0.5,
    "safety_margin": 2.0,
    "curvature_threshold": 0.1,
    "max_iterations": 50
  }
}
```

### 案例3：参数对比实验

**目标**：对比不同参数设置的效果

```bash
# 安全优先
python3 cli.py -f scenarios/example1.json \
  --w1 0.7 --w2 0.3 \
  -o results/safety_priority.png --no-viz

# 平滑优先
python3 cli.py -f scenarios/example1.json \
  --w1 0.4 --w2 0.6 \
  -o results/smooth_priority.png --no-viz

# 平衡模式
python3 cli.py -f scenarios/example1.json \
  --w1 0.6 --w2 0.4 \
  -o results/balanced.png --no-viz
```

---

## 常见问题解答

**Q1: 如何提高规划速度？**

A: 
- 降低网格分辨率（如从0.5改为1.0）
- 减少最大迭代次数
- 简化障碍物表示

**Q2: 路径经常碰到障碍物怎么办？**

A:
- 增加安全裕度
- 提高w1权重
- 检查障碍物配置是否正确

**Q3: 如何生成更平滑的路径？**

A:
- 提高w2权重
- 降低曲率阈值
- 增加路径点数量（修改代码中的min/max_waypoints）

**Q4: 支持动态障碍物吗？**

A: 当前版本仅支持静态障碍物。动态障碍物支持在开发计划中。

---

**文档版本**：v1.0.0  
**最后更新**：2025年10月

