"""
Dijkstra算法实现
用于在加权图中寻找最短路径
"""

import heapq
import numpy as np
from typing import List, Tuple, Dict
from dataclasses import dataclass


@dataclass
class Node:
    """图节点（使用格心坐标）  # CHG: 说明采用格心"""
    x: float
    y: float

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        """用于优先队列的比较"""
        if self.x != other.x:
            return self.x < other.x
        return self.y < other.y

    def distance_to(self, other: "Node") -> float:
        """计算到另一个节点的欧几里得距离"""
        return np.hypot(self.x - other.x, self.y - other.y)


class DijkstraPathPlanner:
    """Dijkstra路径规划器"""

    def __init__(self, environment_width: float, environment_height: float, resolution: float = 1.0):
        """
        初始化Dijkstra规划器

        Args:
            environment_width: 环境宽度
            environment_height: 环境高度
            resolution: 网格分辨率
        """
        self.width = environment_width
        self.height = environment_height
        self.resolution = resolution

        # 构建网格
        self.grid_width = int(environment_width / resolution)
        self.grid_height = int(environment_height / resolution)

        # 障碍物网格（True表示障碍物）
        self.obstacle_grid = np.zeros((self.grid_height, self.grid_width), dtype=bool)

        # CHG: 保存“膨胀后的障碍网格”，供建图与后续视线检测复用
        # WHY: 让拉直/直连遵守与建图一致的安全裕度口径
        self.expanded_obstacles = None

        # 图结构：节点 -> [(邻居节点, 边权重)]
        self.graph: Dict[Node, List[Tuple[Node, float]]] = {}

    # ---------------- 障碍离散化（全部以“格心”判定） ----------------
    def add_obstacle(self, obstacle: Dict):
        """
        添加障碍物到环境中

        Args:
            obstacle: 障碍物字典，包含type和相关参数
        """
        if obstacle["type"] == "rectangle":
            self._add_rectangle_obstacle(obstacle["vertices"])
        elif obstacle["type"] == "circle":
            self._add_circle_obstacle(obstacle["center"], obstacle["radius"])
        elif obstacle["type"] == "polygon":
            self._add_polygon_obstacle(obstacle["vertices"])

    def _cell_center(self, i: int, j: int) -> Tuple[float, float]:
        """返回网格(i,j)的格心世界坐标  # ADD"""
        return (j + 0.5) * self.resolution, (i + 0.5) * self.resolution

    def _add_rectangle_obstacle(self, vertices: List[List[float]]):
        """添加矩形障碍物（格心判定）  # CHG"""
        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        for i in range(self.grid_height):
            for j in range(self.grid_width):
                x, y = self._cell_center(i, j)  # CHG: 用格心
                if (min_x <= x <= max_x) and (min_y <= y <= max_y):
                    self.obstacle_grid[i, j] = True

    def _add_circle_obstacle(self, center: List[float], radius: float):
        """添加圆形障碍物（格心判定）  # CHG"""
        cx, cy = center
        r2 = radius * radius
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                x, y = self._cell_center(i, j)  # CHG: 用格心
                if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2:
                    self.obstacle_grid[i, j] = True

    def _add_polygon_obstacle(self, vertices: List[List[float]]):
        """添加多边形障碍物（格心+边界视为在内）  # CHG"""
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                x, y = self._cell_center(i, j)  # CHG: 用格心
                if self._point_in_polygon(x, y, vertices, include_boundary=True):  # CHG
                    self.obstacle_grid[i, j] = True

    def _point_in_polygon(
        self, x: float, y: float, vertices: List[List[float]], include_boundary: bool = True
    ) -> bool:
        """射线法判断点是否在多边形内；可选把边界视为在内  # CHG"""
        n = len(vertices)
        inside = False
        j = n - 1

        for i in range(n):
            xi, yi = vertices[i]
            xj, yj = vertices[j]

            # 边界判定（点在线段上的情况）
            if include_boundary:
                # 判断 (x,y) 是否在线段 [(xi,yi)-(xj,yj)] 上
                denom = np.hypot(xj - xi, yj - yi)
                if denom > 1e-9:
                    proj = ((x - xi) * (xj - xi) + (y - yi) * (yj - yi)) / (denom * denom)
                    if 0.0 <= proj <= 1.0:
                        # 最近点到线段距离
                        px = xi + proj * (xj - xi)
                        py = yi + proj * (yj - yi)
                        if np.hypot(px - x, py - y) < 1e-9:
                            return True

            intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
            if intersect:
                inside = not inside
            j = i

        return inside

    # ---------------- 图构建（基于膨胀障碍 + 格心） ----------------
    def build_graph(self, safety_margin: float = 2.0):
        """
        构建加权图

        Args:
            safety_margin: 安全裕度（距离障碍物的最小距离）
        """
        self.graph.clear()

        # 扩展障碍物（考虑安全裕度）
        margin_cells = int(max(0.0, safety_margin) / self.resolution)
        self.expanded_obstacles = self._expand_obstacles(margin_cells)  # CHG: 保存下来

        # 8连通性：上下左右+对角线
        directions = [
            (0, 1),
            (1, 0),
            (0, -1),
            (-1, 0),  # 上下左右
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),  # 对角线
        ]

        for i in range(self.grid_height):
            for j in range(self.grid_width):
                if self.expanded_obstacles[i, j]:
                    continue

                cx, cy = self._cell_center(i, j)  # CHG: 节点放在格心
                current_node = Node(cx, cy)
                neighbors = []

                for di, dj in directions:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < self.grid_height and 0 <= nj < self.grid_width:
                        if not self.expanded_obstacles[ni, nj]:
                            nx, ny = self._cell_center(ni, nj)  # CHG
                            neighbor_node = Node(nx, ny)
                            distance = current_node.distance_to(neighbor_node)
                            neighbors.append((neighbor_node, distance))

                self.graph[current_node] = neighbors

    def _expand_obstacles(self, margin_cells: int) -> np.ndarray:
        """扩展障碍物网格以考虑安全裕度"""
        from scipy.ndimage import binary_dilation

        if margin_cells > 0:
            structure = np.ones((2 * margin_cells + 1, 2 * margin_cells + 1), dtype=bool)
            return binary_dilation(self.obstacle_grid, structure=structure)
        return self.obstacle_grid.copy()

    # ---------------- 起/终点贴近（默认以“膨胀网格”为口径） ----------------
    def find_nearest_node(self, x: float, y: float, use_expanded: bool = True) -> Node:
        """
        找到最接近给定坐标的可行节点（格心）
        # CHG: 支持以“膨胀网格”作为可行性口径，确保与建图一致
        """
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)

        occ = self.expanded_obstacles if (use_expanded and self.expanded_obstacles is not None) else self.obstacle_grid

        # 搜索最近的非障碍物节点（按环形扩张）
        search_radius = 0
        max_r = max(self.grid_width, self.grid_height)
        while search_radius <= max_r:
            for di in range(-search_radius, search_radius + 1):
                for dj in range(-search_radius, search_radius + 1):
                    ni, nj = grid_y + di, grid_x + dj
                    if 0 <= ni < self.grid_height and 0 <= nj < self.grid_width:
                        if not occ[ni, nj]:
                            cx, cy = self._cell_center(ni, nj)  # CHG: 返回格心
                            return Node(cx, cy)
            search_radius += 1

        raise ValueError("无法找到可行的起点或终点")

    # ---------------- 视线检测 + 路径拉直（string-pulling） ----------------
    def _occupied(self, x: float, y: float) -> bool:
        """
        查询世界坐标(x,y)是否在膨胀后的障碍内  # ADD
        NOTE: 用于视线检测；若还未build_graph，退化到 obstacle_grid。
        """
        i = int(y / self.resolution)
        j = int(x / self.resolution)
        if not (0 <= i < self.grid_height and 0 <= j < self.grid_width):
            return True
        occ = self.expanded_obstacles if self.expanded_obstacles is not None else self.obstacle_grid
        return bool(occ[i, j])

    def _line_of_sight(self, p: Tuple[float, float], q: Tuple[float, float]) -> bool:
        """
        检查 p->q 直线是否完全在“膨胀后的自由空间”内  # ADD
        """
        px, py = p
        qx, qy = q
        dx = qx - px
        dy = qy - py
        L = np.hypot(dx, dy)
        if L < 1e-9:
            return True
        step = 0.5 * self.resolution  # 采样步长（可调）
        n = int(L / step) + 1
        for k in range(n + 1):
            t = k / max(1, n)
            x = px + t * dx
            y = py + t * dy
            if self._occupied(x, y):
                return False
        return True

    def _shortcut_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        对折线做“锚点推进式”拉直  # ADD
        在保证整段可视线直连的前提下，最大化删除中间点
        """
        if len(path) <= 2:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            # 找到最远可直连的 j
            while j > i + 1 and not self._line_of_sight(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    # ---------------- Dijkstra 主流程 ----------------
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        使用Dijkstra算法规划路径

        Args:
            start: 起点坐标 (x, y)
            goal: 终点坐标 (x, y)

        Returns:
            路径点列表 [(x1, y1), (x2, y2), ...]
        """
        # CHG: 先确保已构建图（含expanded网格）
        if self.expanded_obstacles is None:
            self.build_graph(safety_margin=0.0)

        # CHG: 以“膨胀网格”作为可行性口径寻找最近可行格心
        start_node = self.find_nearest_node(start[0], start[1], use_expanded=True)
        goal_node = self.find_nearest_node(goal[0], goal[1], use_expanded=True)

        # Dijkstra算法
        pq = [(0.0, start_node)]
        distances = {start_node: 0.0}
        previous = {start_node: None}
        visited = set()

        while pq:
            current_dist, current_node = heapq.heappop(pq)
            if current_node in visited:
                continue
            visited.add(current_node)

            if current_node == goal_node:
                break

            if current_node not in self.graph:
                continue

            for neighbor, edge_weight in self.graph[current_node]:
                if neighbor in visited:
                    continue
                new_dist = current_dist + edge_weight
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (new_dist, neighbor))

        # 重建路径
        if goal_node not in previous and goal_node != start_node:
            raise ValueError("无法找到从起点到终点的路径")

        path = []
        cur = goal_node
        while cur is not None:
            path.append((cur.x, cur.y))
            cur = previous.get(cur)
        path.reverse()

        # CHG: 拉直（保证直连段都在膨胀自由空间内）
        path = self._shortcut_path(path)

        return path

    def get_path_length(self, path: List[Tuple[float, float]]) -> float:
        """计算路径长度"""
        if len(path) < 2:
            return 0.0
        diffs = np.diff(np.array(path), axis=0)
        seg = np.hypot(diffs[:, 0], diffs[:, 1])
        return float(np.sum(seg))


if __name__ == "__main__":
    # 简单自测
    planner = DijkstraPathPlanner(100, 100, resolution=0.5)

    # 添加障碍物
    planner.add_obstacle({"type": "rectangle", "vertices": [[20, 20], [20, 40], [40, 40], [40, 20]]})
    planner.add_obstacle({"type": "circle", "center": [60, 60], "radius": 10})

    # 构建图（含安全裕度）
    planner.build_graph(safety_margin=2.0)

    # 规划路径
    try:
        path = planner.plan((5, 5), (95, 95))
        print(f"找到路径，包含 {len(path)} 个点")
        print(f"路径长度: {planner.get_path_length(path):.2f}")
        print(f"前5个点: {path[:5]}")
    except ValueError as e:
        print(f"路径规划失败: {e}")
