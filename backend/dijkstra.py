"""
Dijkstra算法实现
用于在加权图中寻找最短路径
"""

import heapq
import numpy as np
from typing import List, Tuple, Dict, Set
from dataclasses import dataclass


@dataclass
class Node:
    """图节点"""
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
    
    def distance_to(self, other: 'Node') -> float:
        """计算到另一个节点的欧几里得距离"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


class DijkstraPathPlanner:
    """Dijkstra路径规划器"""
    
    def __init__(self, environment_width: float, environment_height: float, 
                 resolution: float = 1.0):
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
        
        # 图结构：节点 -> [(邻居节点, 边权重)]
        self.graph: Dict[Node, List[Tuple[Node, float]]] = {}
    
    def add_obstacle(self, obstacle: Dict):
        """
        添加障碍物到环境中
        
        Args:
            obstacle: 障碍物字典，包含type和相关参数
        """
        if obstacle['type'] == 'rectangle':
            self._add_rectangle_obstacle(obstacle['vertices'])
        elif obstacle['type'] == 'circle':
            self._add_circle_obstacle(obstacle['center'], obstacle['radius'])
        elif obstacle['type'] == 'polygon':
            self._add_polygon_obstacle(obstacle['vertices'])
    
    def _add_rectangle_obstacle(self, vertices: List[List[float]]):
        """添加矩形障碍物"""
        # 获取边界框
        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        # 标记网格
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                x = j * self.resolution
                y = i * self.resolution
                if min_x <= x <= max_x and min_y <= y <= max_y:
                    self.obstacle_grid[i, j] = True
    
    def _add_circle_obstacle(self, center: List[float], radius: float):
        """添加圆形障碍物"""
        cx, cy = center
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                x = j * self.resolution
                y = i * self.resolution
                if np.sqrt((x - cx)**2 + (y - cy)**2) <= radius:
                    self.obstacle_grid[i, j] = True
    
    def _add_polygon_obstacle(self, vertices: List[List[float]]):
        """添加多边形障碍物（简化版：使用边界框）"""
        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                x = j * self.resolution
                y = i * self.resolution
                if self._point_in_polygon(x, y, vertices):
                    self.obstacle_grid[i, j] = True
    
    def _point_in_polygon(self, x: float, y: float, vertices: List[List[float]]) -> bool:
        """射线法判断点是否在多边形内"""
        n = len(vertices)
        inside = False
        
        j = n - 1
        for i in range(n):
            xi, yi = vertices[i]
            xj, yj = vertices[j]
            
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        
        return inside
    
    def build_graph(self, safety_margin: float = 2.0):
        """
        构建加权图
        
        Args:
            safety_margin: 安全裕度（距离障碍物的最小距离）
        """
        self.graph.clear()
        
        # 扩展障碍物（考虑安全裕度）
        margin_cells = int(safety_margin / self.resolution)
        expanded_obstacles = self._expand_obstacles(margin_cells)
        
        # 8连通性：上下左右+对角线
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 上下左右
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # 对角线
        ]
        
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                if expanded_obstacles[i, j]:
                    continue
                
                current_node = Node(j * self.resolution, i * self.resolution)
                neighbors = []
                
                for di, dj in directions:
                    ni, nj = i + di, j + dj
                    
                    # 边界检查
                    if 0 <= ni < self.grid_height and 0 <= nj < self.grid_width:
                        if not expanded_obstacles[ni, nj]:
                            neighbor_node = Node(nj * self.resolution, ni * self.resolution)
                            distance = current_node.distance_to(neighbor_node)
                            neighbors.append((neighbor_node, distance))
                
                self.graph[current_node] = neighbors
    
    def _expand_obstacles(self, margin_cells: int) -> np.ndarray:
        """扩展障碍物网格以考虑安全裕度"""
        from scipy.ndimage import binary_dilation
        
        if margin_cells > 0:
            structure = np.ones((2 * margin_cells + 1, 2 * margin_cells + 1))
            return binary_dilation(self.obstacle_grid, structure=structure)
        return self.obstacle_grid.copy()
    
    def find_nearest_node(self, x: float, y: float) -> Node:
        """找到最接近给定坐标的可行节点"""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        
        # 搜索最近的非障碍物节点
        search_radius = 1
        while search_radius < max(self.grid_width, self.grid_height):
            for di in range(-search_radius, search_radius + 1):
                for dj in range(-search_radius, search_radius + 1):
                    ni, nj = grid_y + di, grid_x + dj
                    if 0 <= ni < self.grid_height and 0 <= nj < self.grid_width:
                        if not self.obstacle_grid[ni, nj]:
                            return Node(nj * self.resolution, ni * self.resolution)
            search_radius += 1
        
        raise ValueError("无法找到可行的起点或终点")
    
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        使用Dijkstra算法规划路径
        
        Args:
            start: 起点坐标 (x, y)
            goal: 终点坐标 (x, y)
        
        Returns:
            路径点列表 [(x1, y1), (x2, y2), ...]
        """
        # 找到最近的可行节点
        start_node = self.find_nearest_node(start[0], start[1])
        goal_node = self.find_nearest_node(goal[0], goal[1])
        
        # Dijkstra算法
        # 优先队列：(距离, 节点)
        pq = [(0, start_node)]
        distances = {start_node: 0}
        previous = {start_node: None}
        visited = set()
        
        while pq:
            current_dist, current_node = heapq.heappop(pq)
            
            if current_node in visited:
                continue
            
            visited.add(current_node)
            
            # 到达目标
            if current_node == goal_node:
                break
            
            # 检查邻居
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
        current = goal_node
        while current is not None:
            path.append((current.x, current.y))
            current = previous.get(current)
        
        path.reverse()
        return path
    
    def get_path_length(self, path: List[Tuple[float, float]]) -> float:
        """计算路径长度"""
        length = 0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += np.sqrt(dx**2 + dy**2)
        return length


if __name__ == "__main__":
    # 测试代码
    planner = DijkstraPathPlanner(100, 100, resolution=0.5)
    
    # 添加障碍物
    planner.add_obstacle({
        'type': 'rectangle',
        'vertices': [[20, 20], [20, 40], [40, 40], [40, 20]]
    })
    
    planner.add_obstacle({
        'type': 'circle',
        'center': [60, 60],
        'radius': 10
    })
    
    # 构建图
    planner.build_graph(safety_margin=2.0)
    
    # 规划路径
    try:
        path = planner.plan((5, 5), (95, 95))
        print(f"找到路径，包含 {len(path)} 个点")
        print(f"路径长度: {planner.get_path_length(path):.2f}")
        print(f"前5个点: {path[:5]}")
    except ValueError as e:
        print(f"路径规划失败: {e}")

