# DWAPCBC - Python路径规划库

[![Python Version](https://img.shields.io/badge/python-3.8%2B-blue)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Version](https://img.shields.io/badge/version-1.0.0-orange)](https://github.com/qwagrox/dwapcbc-system)

**Dynamic Waypoint Allocation Piecewise Cubic Bezier Curve (DWAPCBC)** - 商业级移动机器人路径规划Python库

---

## 🚀 快速安装

### 从PyPI安装（推荐）

```bash
pip install dwapcbc
```

### 从GitHub安装

```bash
pip install git+https://github.com/qwagrox/dwapcbc-system.git
```

### 从源码安装

```bash
git clone https://github.com/qwagrox/dwapcbc-system.git
cd dwapcbc-system
pip install -e .
```

---

## 📖 快速开始

### 基本使用

```python
from dwapcbc import DWAPCBCPlanner, DWAPCBCConfig

# 创建规划器
config = DWAPCBCConfig(
    environment_width=100,
    environment_height=100,
    w1_safety=0.6,
    w2_smoothness=0.4
)
planner = DWAPCBCPlanner(config)

# 设置障碍物
obstacles = [
    {'type': 'rectangle', 'vertices': [[20, 20], [20, 40], [40, 40], [40, 20]]},
    {'type': 'circle', 'center': [60, 60], 'radius': 8}
]
planner.set_obstacles(obstacles)

# 执行规划
result = planner.plan((5, 5), (95, 95))

if result.success:
    print(f"路径长度: {result.path_length:.2f} m")
    print(f"安全距离: {result.safety_distance:.2f} m")
    print(f"平滑度: {result.smoothness:.2f}°")
```

### 命令行工具

```bash
# 使用JSON配置文件
dwapcbc-plan -f scenario.json --no-viz

# 批量测试
dwapcbc-batch -d scenarios -o results
```

---

## ✨ 核心特性

- ✅ **安全性保障** - 动态安全间隙，确保与障碍物保持安全距离
- ✅ **平滑轨迹** - 分段三次贝塞尔曲线优化，保证C²连续性
- ✅ **智能优化** - 自适应路径点分配，根据环境复杂度动态调整
- ✅ **高效计算** - 基于Dijkstra算法的全局最优路径搜索
- ✅ **易于集成** - 简洁的Python API，支持多种应用场景

---

## 📊 性能指标

根据标准测试场景：

| 指标 | 平均值 |
|------|--------|
| 成功率 | 100% |
| 安全距离 | 1.31 m |
| 平滑度 | 160.56° |
| 计算时间 | 1.37 s |

---

## 🔧 API参考

### 核心类

#### DWAPCBCPlanner

主路径规划器类。

```python
planner = DWAPCBCPlanner(config)
planner.set_obstacles(obstacles)
result = planner.plan(start, goal)
```

#### DWAPCBCConfig

配置类，支持自定义参数。

```python
config = DWAPCBCConfig(
    environment_width=100.0,      # 环境宽度
    environment_height=100.0,     # 环境高度
    grid_resolution=0.5,          # 网格分辨率
    safety_margin=2.0,            # 安全裕度
    w1_safety=0.6,                # 安全性权重
    w2_smoothness=0.4,            # 平滑性权重
    curvature_threshold=0.1,      # 曲率阈值
    max_iterations=50             # 最大迭代次数
)
```

#### PlanningResult

规划结果类，包含完整的路径信息和性能指标。

```python
if result.success:
    trajectory = result.final_trajectory      # 最终轨迹点
    waypoints = result.optimized_waypoints    # 优化后的路径点
    safety = result.safety_distance           # 安全距离
    smoothness = result.smoothness            # 平滑度
    length = result.path_length               # 路径长度
    time = result.computation_time            # 计算时间
```

---

## 📚 使用示例

### 示例1: 机器人导航

```python
from dwapcbc import DWAPCBCPlanner, DWAPCBCConfig

class RobotNavigator:
    def __init__(self, robot_width=0.5):
        config = DWAPCBCConfig(
            w1_safety=0.7,
            w2_smoothness=0.3,
            safety_margin=robot_width * 2
        )
        self.planner = DWAPCBCPlanner(config)
    
    def navigate_to(self, current_pos, target_pos):
        result = self.planner.plan(current_pos, target_pos)
        if result.success:
            return result.final_trajectory
        return None

# 使用
robot = RobotNavigator()
path = robot.navigate_to((10, 10), (90, 90))
```

### 示例2: 参数对比

```python
configs = {
    '安全优先': DWAPCBCConfig(w1_safety=0.7, w2_smoothness=0.3),
    '平滑优先': DWAPCBCConfig(w1_safety=0.4, w2_smoothness=0.6),
    '平衡模式': DWAPCBCConfig(w1_safety=0.6, w2_smoothness=0.4)
}

for name, config in configs.items():
    planner = DWAPCBCPlanner(config)
    planner.set_obstacles(obstacles)
    result = planner.plan((5, 5), (95, 95))
    print(f"{name}: 安全距离={result.safety_distance:.2f}m")
```

### 示例3: 批量规划

```python
tasks = [
    ((10, 10), (90, 90)),
    ((10, 90), (90, 10)),
    ((50, 10), (50, 90))
]

planner = DWAPCBCPlanner(DWAPCBCConfig())
planner.set_obstacles(obstacles)

for start, goal in tasks:
    result = planner.plan(start, goal)
    if result.success:
        print(f"{start} → {goal}: {result.path_length:.2f}m")
```

---

## 🎯 应用场景

- **移动机器人导航** - 室内服务机器人、仓储AGV、清洁机器人
- **自动驾驶车辆** - 园区自动驾驶、停车场导航、低速无人车
- **无人机路径规划** - 室内无人机、避障飞行、巡检路径
- **工业自动化** - 工厂AGV、自动化仓库、生产线物流

---

## 📦 依赖要求

### 必需依赖

- Python >= 3.8
- numpy >= 1.20.0
- scipy >= 1.7.0
- matplotlib >= 3.4.0
- pandas >= 1.3.0
- networkx >= 2.6.0
- shapely >= 1.8.0

### 可选依赖

```bash
# API服务
pip install dwapcbc[api]

# 开发工具
pip install dwapcbc[dev]

# 完整安装
pip install dwapcbc[all]
```

---

## 📖 文档

- [安装指南](INSTALL.md)
- [使用教程](USAGE_GUIDE.md)
- [API参考](API_REFERENCE.md)
- [项目总结](PROJECT_SUMMARY.md)
- [研究改进方向](RESEARCH_IMPROVEMENTS.md)

---

## 🔬 理论基础

本库基于以下研究成果：

**论文**: Ahmad, J., & Wahab, M. N. A. (2025). Enhancing the safety and smoothness of path planning through an integration of Dijkstra's algorithm and piecewise cubic Bezier optimization. *Expert Systems With Applications*, 289, 128315.

**关键创新**:
1. 动态路径点分配策略（DWA）
2. 分段三次贝塞尔曲线优化（PCBC）
3. 自适应安全间隙机制
4. 多目标迭代优化

---

## 🤝 贡献

欢迎贡献代码、报告问题或提出建议！

1. Fork本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启Pull Request

---

## 📄 许可证

本项目采用MIT许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 📧 联系方式

- GitHub: [https://github.com/qwagrox/dwapcbc-system](https://github.com/qwagrox/dwapcbc-system)
- Issues: [https://github.com/qwagrox/dwapcbc-system/issues](https://github.com/qwagrox/dwapcbc-system/issues)

---

## 🙏 致谢

感谢Ahmad和Wahab的原创研究工作，为移动机器人路径规划领域做出了重要贡献。

---

**版本**: 1.0.0  
**更新日期**: 2025-10-16

