# -*- coding: utf-8 -*-
"""
DWAPCBC核心算法实现
Dynamic Waypoint Allocation Piecewise Cubic Bezier Curve
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import time

from .dijkstra import DijkstraPathPlanner
from .path_optimizer import PathOptimizer, PathMetrics
from .bezier_curve import PiecewiseCubicBezierCurve


# =========================
# 配置
# =========================
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
    angle_threshold: float = 170.0

    # 动态路径点分配参数
    min_waypoints: int = 7
    max_waypoints: int = 12
    complexity_threshold: float = 0.3  # 环境复杂度阈值

    # 迭代优化参数
    max_iterations: int = 50
    convergence_threshold: float = 0.01

    # 轨迹生成参数
    trajectory_points: int = 200

    # 圆角化参数
    corner_angle_deg: float = 172.0
    corner_back_ratio: float = 0.25
    corner_min_back: float = 1.0
    corner_max_back: float = 4.0
    corner_clearance_keep: float = 0.9


# =========================
# 结果结构
# =========================
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


# =========================
# 主类
# =========================
class DWAPCBCPlanner:
    """DWAPCBC路径规划器"""

    def __init__(self, config: DWAPCBCConfig = None):
        self.config = config or DWAPCBCConfig()

        # 子模块
        self.dijkstra = DijkstraPathPlanner(
            self.config.environment_width,
            self.config.environment_height,
            self.config.grid_resolution,
        )
        self.optimizer = PathOptimizer(
            self.config.curvature_threshold, self.config.angle_threshold
        )

        self.obstacles: List[Dict] = []

    # ---------- 障碍物 ----------
    def set_obstacles(self, obstacles: List[Dict]):
        """设置环境障碍物"""
        self.obstacles = obstacles

        # 同步给 Dijkstra（栅格膨胀在其内部处理）
        self.dijkstra.obstacle_grid.fill(False)
        for obstacle in obstacles:
            self.dijkstra.add_obstacle(obstacle)

    # ---------- 栅格距离 / 碰撞（离散） ----------
    def _nearest_obstacle_distance_grid(self, px: float, py: float, search_cells: int = 24) -> float:
        """基于栅格的近似点到障碍的最小距离"""
        gx = int(px / self.config.grid_resolution)
        gy = int(py / self.config.grid_resolution)
        H, W = self.dijkstra.obstacle_grid.shape
        best = float('inf')
        for dy in range(-search_cells, search_cells + 1):
            y = gy + dy
            if y < 0 or y >= H:
                continue
            for dx in range(-search_cells, search_cells + 1):
                x = gx + dx
                if x < 0 or x >= W:
                    continue
                if self.dijkstra.obstacle_grid[y, x]:
                    ox = (x + 0.5) * self.config.grid_resolution
                    oy = (y + 0.5) * self.config.grid_resolution
                    d = np.hypot(px - ox, py - oy)
                    if d < best:
                        best = d
        return best if best != float('inf') else 1e9

    def _cell_occupied(self, x: float, y: float) -> bool:
        gx = int(x / self.config.grid_resolution)
        gy = int(y / self.config.grid_resolution)
        H, W = self.dijkstra.obstacle_grid.shape
        if gx < 0 or gy < 0 or gx >= W or gy >= H:
            return True  # 越界视为不可行
        return bool(self.dijkstra.obstacle_grid[gy, gx])

    def _trajectory_collides_grid(self, traj: List[Tuple[float, float]]) -> bool:
        """按网格分辨率采样，命中占用格即碰撞"""
        if len(traj) < 2:
            return False
        step = self.config.grid_resolution * 0.5
        for i in range(len(traj) - 1):
            x1, y1 = traj[i]
            x2, y2 = traj[i + 1]
            L = max(1e-9, np.hypot(x2 - x1, y2 - y1))
            n = int(np.ceil(L / step))
            for k in range(n + 1):
                t = k / n if n > 0 else 0.0
                x = x1 + (x2 - x1) * t
                y = y1 + (y2 - y1) * t
                if self._cell_occupied(x, y):
                    return True
        return False

    # ---------- 连续几何碰撞（严格） ----------
    @staticmethod
    def _dot(a, b): return a[0]*b[0] + a[1]*b[1]

    def _segment_circle_intersect(self, p1, p2, center, radius) -> bool:
        """线段-圆 相交"""
        p1 = np.array(p1, float); p2 = np.array(p2, float)
        c = np.array(center, float)
        v = p2 - p1
        w = c - p1
        vv = self._dot(v, v)
        if vv < 1e-12:
            return np.linalg.norm(c - p1) <= radius
        t = np.clip(self._dot(w, v) / vv, 0.0, 1.0)
        closest = p1 + t * v
        return np.linalg.norm(closest - c) <= radius - 1e-9

    @staticmethod
    def _ccw(a, b, c):
        return (c[1]-a[1])*(b[0]-a[0]) > (b[1]-a[1])*(c[0]-a[0])

    def _segment_segment_intersect(self, a1, a2, b1, b2) -> bool:
        """线段-线段 相交"""
        return (self._ccw(a1, b1, b2) != self._ccw(a2, b1, b2)) and (self._ccw(a1, a2, b1) != self._ccw(a1, a2, b2))

    def _segment_rect_intersect(self, p1, p2, rect_vertices) -> bool:
        """线段-轴对齐矩形 相交（rect_vertices: 4点，顺序不限，坐标为左下/左上/右上/右下某种排列）"""
        xs = [v[0] for v in rect_vertices]
        ys = [v[1] for v in rect_vertices]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        # 端点在矩形内
        for p in (p1, p2):
            if xmin - 1e-9 <= p[0] <= xmax + 1e-9 and ymin - 1e-9 <= p[1] <= ymax + 1e-9:
                return True

        # 与四条边检测
        edges = [((xmin, ymin), (xmin, ymax)),
                 ((xmin, ymax), (xmax, ymax)),
                 ((xmax, ymax), (xmax, ymin)),
                 ((xmax, ymin), (xmin, ymin))]
        for e1, e2 in edges:
            if self._segment_segment_intersect(p1, p2, e1, e2):
                return True
        return False

    def _trajectory_collides_geometry(self, traj: List[Tuple[float, float]]) -> bool:
        """连续几何层面对圆/矩形精确相交"""
        if len(traj) < 2:
            return False

        for i in range(len(traj) - 1):
            a = traj[i]; b = traj[i + 1]
            for obs in self.obstacles:
                if obs['type'] == 'circle':
                    if self._segment_circle_intersect(a, b, obs['center'], float(obs['radius'])):
                        return True
                elif obs['type'] in ('rectangle', 'polygon'):
                    if self._segment_rect_intersect(a, b, obs['vertices']):
                        return True
        return False

    def _trajectory_collides(self, traj: List[Tuple[float, float]]) -> bool:
        """统一硬约束：几何 ∪ 栅格 任一命中即视为碰撞"""
        return self._trajectory_collides_geometry(traj) or self._trajectory_collides_grid(traj)

    # ---------- 圆角化 & 工具 ----------
    def _angle_at(self, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        v1 = a - b; v2 = c - b
        n1 = np.linalg.norm(v1); n2 = np.linalg.norm(v2)
        if n1 < 1e-8 or n2 < 1e-8:
            return 180.0
        cosv = np.clip(np.dot(v1, v2)/(n1*n2), -1.0, 1.0)
        return np.degrees(np.arccos(cosv))

    def _backoff_point(self, src: np.ndarray, dst: np.ndarray, dist: float) -> np.ndarray:
        v = dst - src; L = np.linalg.norm(v)
        if L < 1e-8: return src.copy()
        r = max(0.0, min(1.0, dist/L))
        return (1.0 - r) * src + r * dst

    def _round_corners(self, waypoints: List[Tuple[float, float]], back_ratio: Optional[float] = None) -> List[Tuple[float, float]]:
        if len(waypoints) < 3:
            return waypoints

        pts = [np.array(p, float) for p in waypoints]
        out = [pts[0]]

        sm = float(self.config.safety_margin)
        keep_ratio = float(self.config.corner_clearance_keep)
        ang_th = float(self.config.corner_angle_deg)
        back_ratio = self.config.corner_back_ratio if back_ratio is None else back_ratio
        min_back = float(self.config.corner_min_back)
        max_back = float(self.config.corner_max_back)

        for i in range(1, len(pts) - 1):
            a, b, c = pts[i - 1], pts[i], pts[i + 1]
            ang = self._angle_at(a, b, c)
            if ang < ang_th:
                L1 = np.linalg.norm(b - a)
                L2 = np.linalg.norm(c - b)
                back1 = np.clip(back_ratio * L1, min_back, max_back)
                back2 = np.clip(back_ratio * L2, min_back, max_back)

                pL = self._backoff_point(b, a, back1)
                pR = self._backoff_point(b, c, back2)

                d_b = self._nearest_obstacle_distance_grid(b[0], b[1])
                d_L = self._nearest_obstacle_distance_grid(pL[0], pL[1])
                d_R = self._nearest_obstacle_distance_grid(pR[0], pR[1])

                okL = (d_L + 1e-6 >= min(d_b, keep_ratio * sm))
                okR = (d_R + 1e-6 >= min(d_b, keep_ratio * sm))

                if okL and okR:
                    out.append(pL); out.append(pR)
                else:
                    out.append(b)
            else:
                out.append(b)

        out.append(pts[-1])
        return [tuple(p.tolist()) for p in out]

    # ---------- 复杂度 / 评估 ----------
    def _assess_environment_complexity(self) -> float:
        total_cells = self.dijkstra.grid_width * self.dijkstra.grid_height
        obstacle_cells = np.sum(self.dijkstra.obstacle_grid)
        obstacle_density = obstacle_cells / total_cells if total_cells > 0 else 0.0
        num_obstacles = len(self.obstacles)
        obstacle_count_factor = min(1.0, num_obstacles / 20.0)
        return 0.7 * obstacle_density + 0.3 * obstacle_count_factor

    def _determine_waypoint_count(self, complexity: float, path_length: int) -> int:
        base = self.config.min_waypoints if complexity < self.config.complexity_threshold else self.config.max_waypoints
        if path_length < 20:
            base = max(self.config.min_waypoints, base - 2)
        elif path_length > 100:
            base = min(self.config.max_waypoints, base + 1)
        return base

    def _compute_safety_distance(self, path: List[Tuple[float, float]]) -> float:
        if not path:
            return 0.0
        best = float("inf")
        for (x, y) in path:
            d = self._nearest_obstacle_distance_grid(x, y)
            best = min(best, d)
        return 0.0 if best == float("inf") else best

    def _evaluate_path(self, path: List[Tuple[float, float]], bezier: PiecewiseCubicBezierCurve):
        traj = bezier.generate_trajectory(self.config.trajectory_points)
        safety_distance = self._compute_safety_distance(traj)
        safety_score = safety_distance
        smoothness = bezier.compute_smoothness()
        smoothness_score = -smoothness
        total_score = self.config.w1_safety * safety_score + self.config.w2_smoothness * smoothness_score
        return safety_score, smoothness_score, total_score

    def _iterative_optimization(self, initial_waypoints: List[Tuple[float, float]], target_count: int):
        best_waypoints = initial_waypoints
        best_score = float('-inf')
        iterations = 0

        for iteration in range(self.config.max_iterations):
            iterations += 1
            candidates: List[List[Tuple[float, float]]] = []

            # 候选1：关键点（曲率驱动）
            critical_idx = self.optimizer.identify_critical_points(initial_waypoints, target_count - 2)
            candidates.append([initial_waypoints[i] for i in critical_idx])

            # 候选2：均匀抽样
            if len(initial_waypoints) > target_count:
                step = len(initial_waypoints) / (target_count - 1)
                idx = [int(i * step) for i in range(target_count - 1)]
                idx.append(len(initial_waypoints) - 1)
                candidates.append([initial_waypoints[i] for i in idx])

            for cand in candidates:
                if len(cand) < 2:
                    continue
                try:
                    bez = PiecewiseCubicBezierCurve(cand, ensure_c2_continuity=True)
                    _, _, score = self._evaluate_path(cand, bez)
                    if score > best_score:
                        best_score = score
                        best_waypoints = cand
                except Exception:
                    continue

            if iteration > 0 and abs(score - best_score) < self.config.convergence_threshold:
                break

        return best_waypoints, iterations

    # ---------- 主流程 ----------
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> PlanningResult:
        t0 = time.time()
        try:
            # 1) Dijkstra 初始路径（已含 safety_margin）
            self.dijkstra.build_graph(self.config.safety_margin)
            initial_path = self.dijkstra.plan(start, goal)
            if len(initial_path) < 2:
                raise ValueError("无法找到有效路径")

            # 2) 环境复杂度
            complexity = self._assess_environment_complexity()

            # 3) 目标路径点数量
            target_waypoints = self._determine_waypoint_count(complexity, len(initial_path))

            # 4) 减冗余
            optimized_path, _ = self.optimizer.optimize_path(initial_path, target_waypoints)

            # 4.5) 圆角化 + 自适应回退（几何+栅格双检）
            trials = [self.config.corner_back_ratio, 0.20, 0.16, 0.12, 0.10]
            candidate_waypoints = None
            for br in trials:
                wps = self._round_corners(optimized_path, back_ratio=br)
                bez = PiecewiseCubicBezierCurve(wps, ensure_c2_continuity=True)
                traj = bez.generate_trajectory(self.config.trajectory_points)
                if not self._trajectory_collides(traj):
                    candidate_waypoints = wps
                    break
            if candidate_waypoints is None:
                candidate_waypoints = optimized_path  # 取消圆角

            # 5) 迭代点选
            if len(candidate_waypoints) > target_waypoints:
                final_waypoints, iterations = self._iterative_optimization(candidate_waypoints, target_waypoints)
            else:
                final_waypoints = candidate_waypoints
                iterations = 1

            # 6) 生成贝塞尔
            bezier = PiecewiseCubicBezierCurve(final_waypoints, ensure_c2_continuity=True)

            # 7) 轨迹 + 最终硬校验
            trajectory = bezier.generate_trajectory(self.config.trajectory_points)
            if self._trajectory_collides(trajectory):
                # 放弃贝塞尔，输出折线（一定不穿障）
                bezier = None
                trajectory = final_waypoints

            # 8) 指标
            safety_distance = self._compute_safety_distance(trajectory)
            if bezier is not None:
                smoothness = bezier.compute_smoothness()
                path_length = bezier.get_total_length()
            else:
                # 折线口径
                if len(trajectory) < 2:
                    path_length = 0.0
                else:
                    path_length = float(sum(
                        np.hypot(trajectory[i+1][0]-trajectory[i][0],
                                 trajectory[i+1][1]-trajectory[i][1])
                        for i in range(len(trajectory)-1)
                    ))
                # 折线平滑度（平均转角）
                if len(trajectory) < 3:
                    smoothness = 0.0
                else:
                    angs = []
                    for i in range(len(trajectory)-2):
                        p1 = np.array(trajectory[i]); p2 = np.array(trajectory[i+1]); p3 = np.array(trajectory[i+2])
                        v1, v2 = p2-p1, p3-p2
                        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
                        if n1 < 1e-6 or n2 < 1e-6: 
                            continue
                        cos = np.clip(np.dot(v1, v2)/(n1*n2), -1.0, 1.0)
                        ang = 180.0 - np.degrees(np.arccos(cos))
                        angs.append(abs(ang))
                    smoothness = float(np.mean(angs)) if angs else 0.0

            t_cost = time.time() - t0

            return PlanningResult(
                success=True,
                initial_path=initial_path,
                optimized_waypoints=final_waypoints,
                final_trajectory=trajectory,
                bezier_curve=bezier,
                safety_distance=safety_distance,
                smoothness=smoothness,
                path_length=path_length,
                computation_time=t_cost,
                iterations=iterations,
                environment_complexity=complexity,
                num_waypoints=len(final_waypoints),
            )

        except Exception as e:
            t_cost = time.time() - t0
            print(f"规划失败，错误: {e}")
            import traceback; traceback.print_exc()
            return PlanningResult(
                success=False,
                initial_path=[],
                optimized_waypoints=[],
                final_trajectory=[],
                bezier_curve=None,
                safety_distance=0.0,
                smoothness=0.0,
                path_length=0.0,
                computation_time=t_cost,
                iterations=0,
                environment_complexity=0.0,
                num_waypoints=0,
            )


# ============ 简单自测 ============
if __name__ == "__main__":
    cfg = DWAPCBCConfig(
        environment_width=100,
        environment_height=100,
        grid_resolution=0.5,
        safety_margin=2.0,
        w1_safety=0.6,
        w2_smoothness=0.4,
        trajectory_points=200,
    )
    planner = DWAPCBCPlanner(cfg)

    obstacles = [
        {"type": "rectangle", "vertices": [[20, 20], [20, 40], [40, 40], [40, 20]]},
        {"type": "circle", "center": [60, 60], "radius": 8},
        {"type": "rectangle", "vertices": [[70, 10], [70, 30], [85, 30], [85, 10]]},
        {"type": "circle", "center": [60, 20], "radius": 9},
        {"type": "rectangle", "vertices": [[73, 28], [73, 48], [88, 48], [88, 28]]},
    ]
    planner.set_obstacles(obstacles)

    res = planner.plan((5, 5), (95, 95))
    if res.success:
        print("=" * 60)
        print("DWAPCBC路径规划成功!")
        print("=" * 60)
        print(f"初始路径点数: {len(res.initial_path)}")
        print(f"优化后路径点数: {len(res.optimized_waypoints)}")
        print(f"最终轨迹点数: {len(res.final_trajectory)}")
        print("\n性能指标:")
        print(f"  安全距离: {res.safety_distance:.3f} m")
        print(f"  平滑度: {res.smoothness:.2f}°")
        print(f"  路径长度: {res.path_length:.2f} m")
        print(f"  计算时间: {res.computation_time:.3f} s")
        print(f"  迭代次数: {res.iterations}")
        print("\n环境信息:")
        print(f"  环境复杂度: {res.environment_complexity:.3f}")
        print(f"  路径点数: {res.num_waypoints}")
        print("=" * 60)
    else:
        print("路径规划失败")
