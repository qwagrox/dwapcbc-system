"""
路径优化模块
包含曲率分析、角度偏差计算和冗余点移除
"""

import numpy as np
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class PathMetrics:
    """路径度量指标"""
    curvature: List[float]
    angles: List[float]
    segment_lengths: List[float]


class PathOptimizer:
    """路径优化器"""
    
    def __init__(self, curvature_threshold: float = 0.1, 
                 angle_threshold: float = 175.0):
        """
        初始化路径优化器
        
        Args:
            curvature_threshold: 曲率阈值
            angle_threshold: 角度阈值（度），接近180度的点被认为是冗余的
        """
        self.curvature_threshold = curvature_threshold
        self.angle_threshold = angle_threshold
    
    def compute_curvature(self, path: List[Tuple[float, float]]) -> List[float]:
        """
        计算路径每个点的曲率
        
        公式: κ = ||v1 × v2|| / ||v1||³
        其中 v1 是一阶导数，v2 是二阶导数
        
        Args:
            path: 路径点列表
        
        Returns:
            曲率列表
        """
        if len(path) < 3:
            return [0.0] * len(path)
        
        curvatures = [0.0]  # 第一个点曲率为0
        
        for i in range(1, len(path) - 1):
            # 前一个点
            p_prev = np.array(path[i - 1])
            # 当前点
            p_curr = np.array(path[i])
            # 后一个点
            p_next = np.array(path[i + 1])
            
            # 一阶导数（速度向量）
            v1 = p_curr - p_prev
            v2 = p_next - p_curr
            
            # 计算叉积的模（2D中简化为标量）
            cross_product = v1[0] * v2[1] - v1[1] * v2[0]
            
            # 计算v1的模
            v1_norm = np.linalg.norm(v1)
            
            if v1_norm < 1e-6:
                curvatures.append(0.0)
            else:
                # 曲率公式
                curvature = abs(cross_product) / (v1_norm ** 3)
                curvatures.append(curvature)
        
        curvatures.append(0.0)  # 最后一个点曲率为0
        
        return curvatures
    
    def compute_angles(self, path: List[Tuple[float, float]]) -> List[float]:
        """
        计算路径中连续三个点之间的角度
        
        Args:
            path: 路径点列表
        
        Returns:
            角度列表（度）
        """
        if len(path) < 3:
            return []
        
        angles = []
        
        for i in range(len(path) - 2):
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            p3 = np.array(path[i + 2])
            
            # 向量
            v1 = p2 - p1
            v2 = p3 - p2
            
            # 向量的模
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm < 1e-6 or v2_norm < 1e-6:
                angles.append(180.0)
                continue
            
            # 计算夹角（使用点积）
            cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
            # 限制在[-1, 1]范围内，避免数值误差
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            
            # 转换为度
            angle = np.degrees(np.arccos(cos_angle))
            angles.append(angle)
        
        return angles
    
    def remove_redundant_points(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        使用角度偏差法移除冗余点
        
        如果三个连续点之间的角度接近180度（即几乎在一条直线上），
        则中间的点被认为是冗余的
        
        Args:
            path: 原始路径
        
        Returns:
            优化后的路径
        """
        if len(path) <= 2:
            return path
        
        # 始终保留起点
        optimized_path = [path[0]]
        
        i = 1
        while i < len(path) - 1:
            # 检查当前点是否冗余
            p_prev = np.array(optimized_path[-1])
            p_curr = np.array(path[i])
            p_next = np.array(path[i + 1])
            
            # 计算角度
            v1 = p_curr - p_prev
            v2 = p_next - p_curr
            
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm < 1e-6 or v2_norm < 1e-6:
                i += 1
                continue
            
            cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.degrees(np.arccos(cos_angle))
            
            # 如果角度小于阈值（接近180度），则跳过当前点
            if angle < self.angle_threshold:
                optimized_path.append(path[i])
            
            i += 1
        
        # 始终保留终点
        optimized_path.append(path[-1])
        
        return optimized_path
    
    def identify_critical_points(self, path: List[Tuple[float, float]], 
                                 num_points: int = None) -> List[int]:
        """
        识别关键路径点（高曲率点）
        
        Args:
            path: 路径点列表
            num_points: 期望的关键点数量（不包括起点和终点）
        
        Returns:
            关键点索引列表
        """
        if len(path) <= 2:
            return list(range(len(path)))
        
        # 计算曲率
        curvatures = self.compute_curvature(path)
        
        # 始终包含起点和终点
        critical_indices = [0, len(path) - 1]
        
        if num_points is None or num_points >= len(path) - 2:
            # 返回所有点
            return list(range(len(path)))
        
        # 找到曲率最大的num_points个点
        # 排除起点和终点
        middle_indices = list(range(1, len(path) - 1))
        middle_curvatures = [curvatures[i] for i in middle_indices]
        
        # 按曲率排序
        sorted_indices = sorted(range(len(middle_curvatures)), 
                               key=lambda i: middle_curvatures[i], 
                               reverse=True)
        
        # 选择前num_points个
        selected_middle = [middle_indices[i] for i in sorted_indices[:num_points]]
        
        # 合并并排序
        critical_indices.extend(selected_middle)
        critical_indices.sort()
        
        return critical_indices
    
    def optimize_path(self, path: List[Tuple[float, float]], 
                     target_waypoints: int = None) -> Tuple[List[Tuple[float, float]], PathMetrics]:
        """
        优化路径：移除冗余点并识别关键点
        
        Args:
            path: 原始路径
            target_waypoints: 目标路径点数量
        
        Returns:
            (优化后的路径, 路径度量指标)
        """
        # 步骤1: 移除冗余点
        simplified_path = self.remove_redundant_points(path)
        
        # 步骤2: 如果指定了目标路径点数量，进一步筛选
        if target_waypoints is not None and len(simplified_path) > target_waypoints:
            critical_indices = self.identify_critical_points(simplified_path, 
                                                            target_waypoints - 2)
            simplified_path = [simplified_path[i] for i in critical_indices]
        
        # 计算度量指标
        curvatures = self.compute_curvature(simplified_path)
        angles = self.compute_angles(simplified_path)
        
        segment_lengths = []
        for i in range(len(simplified_path) - 1):
            dx = simplified_path[i+1][0] - simplified_path[i][0]
            dy = simplified_path[i+1][1] - simplified_path[i][1]
            segment_lengths.append(np.sqrt(dx**2 + dy**2))
        
        metrics = PathMetrics(
            curvature=curvatures,
            angles=angles,
            segment_lengths=segment_lengths
        )
        
        return simplified_path, metrics
    
    def analyze_path_complexity(self, path: List[Tuple[float, float]]) -> dict:
        """
        分析路径复杂度
        
        Args:
            path: 路径点列表
        
        Returns:
            复杂度分析结果字典
        """
        curvatures = self.compute_curvature(path)
        angles = self.compute_angles(path)
        
        # 计算统计指标
        avg_curvature = np.mean(curvatures)
        max_curvature = np.max(curvatures)
        std_curvature = np.std(curvatures)
        
        avg_angle = np.mean(angles) if angles else 180.0
        min_angle = np.min(angles) if angles else 180.0
        
        # 高曲率点数量
        high_curvature_count = sum(1 for c in curvatures if c > self.curvature_threshold)
        
        # 急转弯数量（角度小于150度）
        sharp_turn_count = sum(1 for a in angles if a < 150.0)
        
        return {
            'avg_curvature': avg_curvature,
            'max_curvature': max_curvature,
            'std_curvature': std_curvature,
            'avg_angle': avg_angle,
            'min_angle': min_angle,
            'high_curvature_count': high_curvature_count,
            'sharp_turn_count': sharp_turn_count,
            'path_length': len(path)
        }


if __name__ == "__main__":
    # 测试代码
    optimizer = PathOptimizer(curvature_threshold=0.1, angle_threshold=175.0)
    
    # 创建测试路径
    test_path = [
        (0, 0), (1, 1), (2, 2), (3, 3),  # 直线段
        (4, 4), (5, 5), (6, 6),
        (7, 8), (8, 10), (9, 12),  # 曲线段
        (10, 14), (11, 15), (12, 15), (13, 15)  # 另一段
    ]
    
    print("原始路径点数:", len(test_path))
    
    # 优化路径
    optimized_path, metrics = optimizer.optimize_path(test_path, target_waypoints=8)
    
    print("优化后路径点数:", len(optimized_path))
    print("平均曲率:", np.mean(metrics.curvature))
    print("最大曲率:", np.max(metrics.curvature))
    
    # 分析复杂度
    complexity = optimizer.analyze_path_complexity(test_path)
    print("\n路径复杂度分析:")
    for key, value in complexity.items():
        print(f"  {key}: {value:.4f}" if isinstance(value, float) else f"  {key}: {value}")

