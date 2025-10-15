"""
分段三次贝塞尔曲线生成模块
确保C²连续性（二阶导数连续）
"""

import numpy as np
from typing import List, Tuple
from dataclasses import dataclass
from scipy.special import comb


@dataclass
class BezierSegment:
    """贝塞尔曲线段"""
    control_points: List[Tuple[float, float]]
    start_param: float = 0.0
    end_param: float = 1.0
    
    def evaluate(self, u: float) -> Tuple[float, float]:
        """
        计算贝塞尔曲线上的点
        
        Args:
            u: 参数，范围[0, 1]
        
        Returns:
            (x, y) 坐标
        """
        n = len(self.control_points) - 1
        x, y = 0.0, 0.0
        
        for i, (px, py) in enumerate(self.control_points):
            # 贝塞尔基函数
            basis = comb(n, i) * ((1 - u) ** (n - i)) * (u ** i)
            x += basis * px
            y += basis * py
        
        return (x, y)
    
    def evaluate_derivative(self, u: float, order: int = 1) -> Tuple[float, float]:
        """
        计算贝塞尔曲线的导数
        
        Args:
            u: 参数
            order: 导数阶数（1或2）
        
        Returns:
            导数向量 (dx, dy)
        """
        n = len(self.control_points) - 1
        
        if order == 1:
            # 一阶导数
            dx, dy = 0.0, 0.0
            for i in range(n):
                p_curr = np.array(self.control_points[i])
                p_next = np.array(self.control_points[i + 1])
                
                basis = comb(n - 1, i) * ((1 - u) ** (n - 1 - i)) * (u ** i)
                diff = p_next - p_curr
                
                dx += n * basis * diff[0]
                dy += n * basis * diff[1]
            
            return (dx, dy)
        
        elif order == 2:
            # 二阶导数
            dx, dy = 0.0, 0.0
            for i in range(n - 1):
                p_curr = np.array(self.control_points[i])
                p_next = np.array(self.control_points[i + 1])
                p_next2 = np.array(self.control_points[i + 2])
                
                basis = comb(n - 2, i) * ((1 - u) ** (n - 2 - i)) * (u ** i)
                diff = p_next2 - 2 * p_next + p_curr
                
                dx += n * (n - 1) * basis * diff[0]
                dy += n * (n - 1) * basis * diff[1]
            
            return (dx, dy)
        
        else:
            raise ValueError("仅支持1阶和2阶导数")


class PiecewiseCubicBezierCurve:
    """分段三次贝塞尔曲线"""
    
    def __init__(self, waypoints: List[Tuple[float, float]], 
                 ensure_c2_continuity: bool = True):
        """
        初始化分段贝塞尔曲线
        
        Args:
            waypoints: 路径点列表
            ensure_c2_continuity: 是否确保C²连续性
        """
        self.waypoints = waypoints
        self.ensure_c2_continuity = ensure_c2_continuity
        self.segments: List[BezierSegment] = []
        
        if len(waypoints) >= 2:
            self._generate_segments()
    
    def _generate_segments(self):
        """生成贝塞尔曲线段"""
        n = len(self.waypoints)
        
        if n < 2:
            raise ValueError("至少需要2个路径点")
        
        # 为每对相邻路径点生成一个三次贝塞尔曲线段
        for i in range(n - 1):
            p0 = np.array(self.waypoints[i])
            p3 = np.array(self.waypoints[i + 1])
            
            # 计算控制点p1和p2
            if self.ensure_c2_continuity:
                p1, p2 = self._compute_control_points_c2(i)
            else:
                # 简单方法：将控制点放在路径点之间
                direction = p3 - p0
                p1 = p0 + direction / 3
                p2 = p0 + 2 * direction / 3
            
            # 创建贝塞尔段
            control_points = [
                tuple(p0),
                tuple(p1),
                tuple(p2),
                tuple(p3)
            ]
            
            segment = BezierSegment(control_points)
            self.segments.append(segment)
    
    def _compute_control_points_c2(self, segment_index: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算确保C²连续性的控制点
        
        使用论文中的方法：
        B(1, v) = B(0, v+1) 确保C⁰连续性
        B'(1, v) = B'(0, v+1) 确保C¹连续性
        B''(1, v) = B''(0, v+1) 确保C²连续性
        
        Args:
            segment_index: 当前段索引
        
        Returns:
            (p1, p2) 控制点
        """
        i = segment_index
        n = len(self.waypoints)
        
        p0 = np.array(self.waypoints[i])
        p3 = np.array(self.waypoints[i + 1])
        
        # 计算切向量
        if i == 0:
            # 第一段：使用前向差分
            if i + 2 < n:
                tangent_start = np.array(self.waypoints[i + 1]) - p0
                tangent_end = np.array(self.waypoints[i + 2]) - np.array(self.waypoints[i + 1])
            else:
                tangent_start = p3 - p0
                tangent_end = p3 - p0
        elif i == n - 2:
            # 最后一段：使用后向差分
            tangent_start = p0 - np.array(self.waypoints[i - 1])
            tangent_end = p3 - p0
        else:
            # 中间段：使用中心差分
            tangent_start = (p3 - np.array(self.waypoints[i - 1])) / 2
            tangent_end = (np.array(self.waypoints[i + 2]) - p0) / 2
        
        # 归一化切向量
        tangent_start_norm = np.linalg.norm(tangent_start)
        tangent_end_norm = np.linalg.norm(tangent_end)
        
        if tangent_start_norm > 1e-6:
            tangent_start = tangent_start / tangent_start_norm
        if tangent_end_norm > 1e-6:
            tangent_end = tangent_end / tangent_end_norm
        
        # 计算段长度
        segment_length = np.linalg.norm(p3 - p0)
        
        # 控制点距离（使用段长度的1/3）
        control_distance = segment_length / 3
        
        # 计算控制点
        p1 = p0 + tangent_start * control_distance
        p2 = p3 - tangent_end * control_distance
        
        return p1, p2
    
    def evaluate(self, u: float) -> Tuple[float, float]:
        """
        计算整条曲线上的点
        
        Args:
            u: 参数，范围[0, len(segments)]
        
        Returns:
            (x, y) 坐标
        """
        if not self.segments:
            raise ValueError("没有可用的曲线段")
        
        # 确定在哪个段上
        segment_index = int(u)
        if segment_index >= len(self.segments):
            segment_index = len(self.segments) - 1
            local_u = 1.0
        else:
            local_u = u - segment_index
        
        return self.segments[segment_index].evaluate(local_u)
    
    def generate_trajectory(self, num_points: int = 100) -> List[Tuple[float, float]]:
        """
        生成离散的轨迹点
        
        Args:
            num_points: 总点数
        
        Returns:
            轨迹点列表
        """
        if not self.segments:
            return self.waypoints
        
        trajectory = []
        points_per_segment = max(2, num_points // len(self.segments))
        
        for segment in self.segments:
            for i in range(points_per_segment):
                u = i / (points_per_segment - 1) if points_per_segment > 1 else 0
                point = segment.evaluate(u)
                trajectory.append(point)
        
        # 确保包含最后一个点
        if trajectory[-1] != self.waypoints[-1]:
            trajectory.append(self.waypoints[-1])
        
        return trajectory
    
    def compute_curvature_at(self, segment_index: int, u: float) -> float:
        """
        计算指定位置的曲率
        
        曲率公式: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        
        Args:
            segment_index: 段索引
            u: 参数
        
        Returns:
            曲率值
        """
        if segment_index >= len(self.segments):
            return 0.0
        
        segment = self.segments[segment_index]
        
        # 一阶导数
        dx1, dy1 = segment.evaluate_derivative(u, order=1)
        # 二阶导数
        dx2, dy2 = segment.evaluate_derivative(u, order=2)
        
        # 曲率公式
        numerator = abs(dx1 * dy2 - dy1 * dx2)
        denominator = (dx1**2 + dy1**2) ** 1.5
        
        if denominator < 1e-6:
            return 0.0
        
        return numerator / denominator
    
    def compute_smoothness(self) -> float:
        """
        计算整条曲线的平滑度
        
        使用平均转角作为平滑度指标
        
        Returns:
            平滑度值（度）
        """
        if len(self.waypoints) < 3:
            return 0.0
        
        angles = []
        
        for i in range(len(self.waypoints) - 2):
            p1 = np.array(self.waypoints[i])
            p2 = np.array(self.waypoints[i + 1])
            p3 = np.array(self.waypoints[i + 2])
            
            v1 = p2 - p1
            v2 = p3 - p2
            
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm < 1e-6 or v2_norm < 1e-6:
                continue
            
            cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            
            # 转角 = 180 - 夹角
            angle = 180.0 - np.degrees(np.arccos(cos_angle))
            angles.append(abs(angle))
        
        return np.mean(angles) if angles else 0.0
    
    def get_total_length(self) -> float:
        """
        计算曲线总长度（近似）
        
        Returns:
            曲线长度
        """
        trajectory = self.generate_trajectory(num_points=200)
        
        length = 0.0
        for i in range(len(trajectory) - 1):
            dx = trajectory[i+1][0] - trajectory[i][0]
            dy = trajectory[i+1][1] - trajectory[i][1]
            length += np.sqrt(dx**2 + dy**2)
        
        return length


if __name__ == "__main__":
    # 测试代码
    waypoints = [
        (0, 0),
        (20, 10),
        (40, 30),
        (60, 35),
        (80, 50),
        (100, 60)
    ]
    
    print("路径点数:", len(waypoints))
    
    # 创建贝塞尔曲线
    bezier = PiecewiseCubicBezierCurve(waypoints, ensure_c2_continuity=True)
    
    print("曲线段数:", len(bezier.segments))
    
    # 生成轨迹
    trajectory = bezier.generate_trajectory(num_points=100)
    print("轨迹点数:", len(trajectory))
    
    # 计算平滑度
    smoothness = bezier.compute_smoothness()
    print(f"平滑度: {smoothness:.2f}°")
    
    # 计算长度
    length = bezier.get_total_length()
    print(f"曲线长度: {length:.2f}")
    
    # 计算一些点的曲率
    print("\n曲率采样:")
    for i in range(len(bezier.segments)):
        for u in [0.0, 0.5, 1.0]:
            curvature = bezier.compute_curvature_at(i, u)
            print(f"  段{i}, u={u}: κ={curvature:.6f}")

