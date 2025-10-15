"""
DWAPCBC - Dynamic Waypoint Allocation Piecewise Cubic Bezier Curve

A commercial-grade path planning library for mobile robots.

Author: Based on Ahmad & Wahab (2025)
License: MIT
Version: 1.0.0
"""

__version__ = "1.0.0"
__author__ = "DWAPCBC Team"
__license__ = "MIT"

# 导入核心类和函数
from .dwapcbc import (
    DWAPCBCPlanner,
    DWAPCBCConfig,
    PlanningResult
)

from .dijkstra import (
    DijkstraPathPlanner,
    Node
)

from .path_optimizer import (
    PathOptimizer,
    PathMetrics
)

from .bezier_curve import (
    PiecewiseCubicBezierCurve
)

# 定义公开API
__all__ = [
    # 核心规划器
    'DWAPCBCPlanner',
    'DWAPCBCConfig',
    'PlanningResult',
    
    # Dijkstra模块
    'DijkstraPathPlanner',
    'Node',
    
    # 路径优化
    'PathOptimizer',
    'PathMetrics',
    
    # 贝塞尔曲线
    'PiecewiseCubicBezierCurve',
]

# 版本信息
def get_version():
    """获取库版本"""
    return __version__

def get_info():
    """获取库信息"""
    return {
        'name': 'dwapcbc',
        'version': __version__,
        'author': __author__,
        'license': __license__,
        'description': 'Dynamic Waypoint Allocation Piecewise Cubic Bezier Curve Path Planning'
    }

