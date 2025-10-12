"""
DWAPCBC核心算法实现
Dynamic Waypoint Allocation Piecewise Cubic Bezier Curve
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import time

from dijkstra import DijkstraPathPlanner
from path_optimizer import PathOptimizer, PathMetrics
from bezier_curve import PiecewiseCubicBezierCurve


@dataclass
class DWAPCBCConfig:
    """DWAPCBC配置参数"""
    # 环境参数
    environment_width: float = 100.0
    environment_height: float = 100.0
    grid_resolution: float = 0.5
    
    # 安全参数
    safety_margin: float = 2.0
    w1_safety: float = 0.6  # 安全性权重
    w2_smoothness: float = 0.4  # 平滑性权重
    
    # 路径优化参数
    curvature_threshold: float = 0.1
    angle_threshold: float = 175.0
    
    # 动态路径点分配参数
    min_waypoints: int = 7
    max_waypoints: int = 10
    complexity_threshold: float = 0.3  # 环境复杂度阈值
    
    # 迭代优化参数
    max_iterations: int = 50
    convergence_threshold: float = 0.01
    
    # 轨迹生成参数
    trajectory_points: int = 100


@dataclass
class PlanningResult:
    """路径规划结果"""
    success: bool
    initial_path: List[Tuple[float, float]]
    optimized_waypoints: List[Tuple[float, float]]
    final_trajectory: List[Tuple[float, float]]
    bezier_curve: Optional[PiecewiseCubicBezierCurve]
    
    # 性能指标
    safety_distance: float
    smoothness: float
    path_length: float
    computation_time: float
    iterations: int
    
    # 额外信息
    environment_complexity: float
    num_waypoints: int


class DWAPCBCPlanner:
    """DWAPCBC路径规划器"""
    
    def __init__(self, config: DWAPCBCConfig = None):
        """
        初始化DWAPCBC规划器
        
        Args:
            config: 配置参数
        """
        self.config = config or DWAPCBCConfig()
        
        # 初始化子模块
        self.dijkstra = DijkstraPathPlanner(
            self.config.environment_width,
            self.config.environment_height,
            self.config.grid_resolution
        )
        
        self.optimizer = PathOptimizer(
            self.config.curvature_threshold,
            self.config.angle_threshold
        )
        
        self.obstacles = []
    
    def set_obstacles(self, obstacles: List[Dict]):
        """
        设置环境障碍物
        
        Args:
            obstacles: 障碍物列表
        """
        self.obstacles = obstacles
        
        # 清空Dijkstra规划器的障碍物
        self.dijkstra.obstacle_grid.fill(False)
        
        # 添加障碍物
        for obstacle in obstacles:
            self.dijkstra.add_obstacle(obstacle)
    
    def _assess_environment_complexity(self) -> float:
        """
        评估环境复杂度
        
        Returns:
            复杂度评分 [0, 1]
        """
        # 计算障碍物密度
        total_cells = self.dijkstra.grid_width * self.dijkstra.grid_height
        obstacle_cells = np.sum(self.dijkstra.obstacle_grid)
        
        obstacle_density = obstacle_cells / total_cells
        
        # 计算障碍物数量的影响
        num_obstacles = len(self.obstacles)
        obstacle_count_factor = min(1.0, num_obstacles / 20.0)
        
        # 综合评分
        complexity = 0.7 * obstacle_density + 0.3 * obstacle_count_factor
        
        return complexity
    
    def _determine_waypoint_count(self, complexity: float, path_length: int) -> int:
        """
        根据环境复杂度和路径长度确定路径点数量
        
        Args:
            complexity: 环境复杂度
            path_length: 初始路径长度
        
        Returns:
            推荐的路径点数量
        """
        # 基础路径点数量
        if complexity < self.config.complexity_threshold:
            # 低复杂度环境
            base_count = self.config.min_waypoints
        else:
            # 高复杂度环境
            base_count = self.config.max_waypoints
        
        # 根据路径长度调整
        if path_length < 20:
            base_count = max(self.config.min_waypoints, base_count - 2)
        elif path_length > 100:
            base_count = min(self.config.max_waypoints, base_count + 1)
        
        return base_count
    
    def _compute_safety_distance(self, path: List[Tuple[float, float]]) -> float:
        """
        计算路径的安全距离（到最近障碍物的最小距离）
        
        Args:
            path: 路径点列表
        
        Returns:
            最小安全距离
        """
        min_distance = float('inf')
        
        for px, py in path:
            # 转换为网格坐标
            grid_x = int(px / self.config.grid_resolution)
            grid_y = int(py / self.config.grid_resolution)
            
            # 搜索附近的障碍物
            search_radius = 20  # 搜索半径（网格单元）
            
            for dy in range(-search_radius, search_radius + 1):
                for dx in range(-search_radius, search_radius + 1):
                    nx = grid_x + dx
                    ny = grid_y + dy
                    
                    if (0 <= nx < self.dijkstra.grid_width and 
                        0 <= ny < self.dijkstra.grid_height):
                        
                        if self.dijkstra.obstacle_grid[ny, nx]:
                            # 计算距离
                            ox = nx * self.config.grid_resolution
                            oy = ny * self.config.grid_resolution
                            distance = np.sqrt((px - ox)**2 + (py - oy)**2)
                            min_distance = min(min_distance, distance)
        
        return min_distance if min_distance != float('inf') else 0.0
    
    def _evaluate_path(self, path: List[Tuple[float, float]], 
                      bezier: PiecewiseCubicBezierCurve) -> Tuple[float, float, float]:
        """
        评估路径质量
        
        Args:
            path: 路径点列表
            bezier: 贝塞尔曲线
        
        Returns:
            (安全性评分, 平滑度评分, 综合评分)
        """
        # 生成轨迹用于安全性检查
        trajectory = bezier.generate_trajectory(self.config.trajectory_points)
        
        # 计算安全距离
        safety_distance = self._compute_safety_distance(trajectory)
        
        # 安全性评分（距离越大越好）
        safety_score = safety_distance
        
        # 计算平滑度
        smoothness = bezier.compute_smoothness()
        
        # 平滑度评分（角度越小越好，所以取负值）
        smoothness_score = -smoothness
        
        # 综合评分
        total_score = (self.config.w1_safety * safety_score + 
                      self.config.w2_smoothness * smoothness_score)
        
        return safety_score, smoothness_score, total_score
    
    def _iterative_optimization(self, initial_waypoints: List[Tuple[float, float]], 
                               target_count: int) -> Tuple[List[Tuple[float, float]], int]:
        """
        迭代优化路径点配置
        
        Args:
            initial_waypoints: 初始路径点
            target_count: 目标路径点数量
        
        Returns:
            (最优路径点, 迭代次数)
        """
        best_waypoints = initial_waypoints
        best_score = float('-inf')
        
        iterations = 0
        
        for iteration in range(self.config.max_iterations):
            iterations += 1
            
            # 尝试不同的路径点配置
            candidates = []
            
            # 候选1: 基于曲率的关键点
            critical_indices = self.optimizer.identify_critical_points(
                initial_waypoints, target_count - 2
            )
            candidate1 = [initial_waypoints[i] for i in critical_indices]
            candidates.append(candidate1)
            
            # 候选2: 均匀分布
            if len(initial_waypoints) > target_count:
                step = len(initial_waypoints) / (target_count - 1)
                indices = [int(i * step) for i in range(target_count - 1)]
                indices.append(len(initial_waypoints) - 1)
                candidate2 = [initial_waypoints[i] for i in indices]
                candidates.append(candidate2)
            
            # 评估候选方案
            for candidate in candidates:
                if len(candidate) < 2:
                    continue
                
                try:
                    bezier = PiecewiseCubicBezierCurve(candidate, ensure_c2_continuity=True)
                    safety, smoothness, total = self._evaluate_path(candidate, bezier)
                    
                    if total > best_score:
                        best_score = total
                        best_waypoints = candidate
                
                except Exception:
                    continue
            
            # 检查收敛
            if iteration > 0 and abs(total - best_score) < self.config.convergence_threshold:
                break
        
        return best_waypoints, iterations
    
    def plan(self, start: Tuple[float, float], 
            goal: Tuple[float, float]) -> PlanningResult:
        """
        执行完整的DWAPCBC路径规划
        
        Args:
            start: 起点坐标
            goal: 终点坐标
        
        Returns:
            规划结果
        """
        start_time = time.time()
        
        try:
            # 步骤1: 使用Dijkstra算法生成初始路径
            self.dijkstra.build_graph(self.config.safety_margin)
            initial_path = self.dijkstra.plan(start, goal)
            
            if len(initial_path) < 2:
                raise ValueError("无法找到有效路径")
            
            # 步骤2: 评估环境复杂度
            complexity = self._assess_environment_complexity()
            
            # 步骤3: 确定目标路径点数量
            target_waypoints = self._determine_waypoint_count(complexity, len(initial_path))
            
            # 步骤4: 路径优化（移除冗余点）
            optimized_path, metrics = self.optimizer.optimize_path(
                initial_path, target_waypoints
            )
            
            # 步骤5: 迭代优化路径点配置
            if len(optimized_path) > target_waypoints:
                final_waypoints, iterations = self._iterative_optimization(
                    optimized_path, target_waypoints
                )
            else:
                final_waypoints = optimized_path
                iterations = 1
            
            # 步骤6: 生成贝塞尔曲线
            bezier = PiecewiseCubicBezierCurve(final_waypoints, ensure_c2_continuity=True)
            
            # 步骤7: 生成最终轨迹
            trajectory = bezier.generate_trajectory(self.config.trajectory_points)
            
            # 步骤8: 计算性能指标
            safety_distance = self._compute_safety_distance(trajectory)
            smoothness = bezier.compute_smoothness()
            path_length = bezier.get_total_length()
            
            computation_time = time.time() - start_time
            
            return PlanningResult(
                success=True,
                initial_path=initial_path,
                optimized_waypoints=final_waypoints,
                final_trajectory=trajectory,
                bezier_curve=bezier,
                safety_distance=safety_distance,
                smoothness=smoothness,
                path_length=path_length,
                computation_time=computation_time,
                iterations=iterations,
                environment_complexity=complexity,
                num_waypoints=len(final_waypoints)
            )
        
        except Exception as e:
            computation_time = time.time() - start_time
            print(f"规划失败，错误: {str(e)}")
            import traceback
            traceback.print_exc()
            
            return PlanningResult(
                success=False,
                initial_path=[],
                optimized_waypoints=[],
                final_trajectory=[],
                bezier_curve=None,
                safety_distance=0.0,
                smoothness=0.0,
                path_length=0.0,
                computation_time=computation_time,
                iterations=0,
                environment_complexity=0.0,
                num_waypoints=0
            )


if __name__ == "__main__":
    # 测试代码
    config = DWAPCBCConfig(
        environment_width=100,
        environment_height=100,
        grid_resolution=0.5,
        safety_margin=2.0,
        w1_safety=0.6,
        w2_smoothness=0.4
    )
    
    planner = DWAPCBCPlanner(config)
    
    # 设置障碍物（Layout 1的简化版本）
    obstacles = [
        {
            'type': 'rectangle',
            'vertices': [[20, 20], [20, 40], [40, 40], [40, 20]]
        },
        {
            'type': 'circle',
            'center': [60, 60],
            'radius': 8
        },
        {
            'type': 'rectangle',
            'vertices': [[70, 10], [70, 30], [85, 30], [85, 10]]
        }
    ]
    
    planner.set_obstacles(obstacles)
    
    # 规划路径
    result = planner.plan((5, 5), (95, 95))
    
    if result.success:
        print("=" * 60)
        print("DWAPCBC路径规划成功!")
        print("=" * 60)
        print(f"初始路径点数: {len(result.initial_path)}")
        print(f"优化后路径点数: {len(result.optimized_waypoints)}")
        print(f"最终轨迹点数: {len(result.final_trajectory)}")
        print(f"\n性能指标:")
        print(f"  安全距离: {result.safety_distance:.4f} m")
        print(f"  平滑度: {result.smoothness:.4f}°")
        print(f"  路径长度: {result.path_length:.4f} m")
        print(f"  计算时间: {result.computation_time:.4f} s")
        print(f"  迭代次数: {result.iterations}")
        print(f"\n环境信息:")
        print(f"  环境复杂度: {result.environment_complexity:.4f}")
        print(f"  障碍物数量: {len(obstacles)}")
        print("=" * 60)
    else:
        print("路径规划失败")

