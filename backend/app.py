"""
Flask API服务
提供DWAPCBC路径规划的RESTful API
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import numpy as np
from typing import Dict, Any

from dwapcbc import DWAPCBCPlanner, DWAPCBCConfig, PlanningResult


app = Flask(__name__)
CORS(app)  # 允许跨域请求

# 全局规划器实例
planner = None


@app.route('/api/health', methods=['GET'])
def health_check():
    """健康检查接口"""
    return jsonify({
        'status': 'healthy',
        'service': 'DWAPCBC Path Planning API',
        'version': '1.0.0'
    })


@app.route('/api/v1/plan', methods=['POST'])
def plan_path():
    """
    路径规划接口
    
    请求体:
    {
        "start": {"x": 5, "y": 5},
        "goal": {"x": 95, "y": 95},
        "obstacles": [
            {
                "type": "rectangle",
                "vertices": [[10, 20], [10, 40], [15, 40], [15, 20]]
            },
            {
                "type": "circle",
                "center": [50, 50],
                "radius": 10
            }
        ],
        "environment": {
            "width": 100,
            "height": 100,
            "resolution": 0.5
        },
        "parameters": {
            "w1": 0.6,
            "w2": 0.4,
            "safety_margin": 2.0,
            "curvature_threshold": 0.1,
            "max_iterations": 50
        }
    }
    """
    try:
        data = request.get_json()
        
        # 解析请求参数
        start = (data['start']['x'], data['start']['y'])
        goal = (data['goal']['x'], data['goal']['y'])
        obstacles = data.get('obstacles', [])
        
        # 环境参数
        env = data.get('environment', {})
        env_width = env.get('width', 100.0)
        env_height = env.get('height', 100.0)
        resolution = env.get('resolution', 0.5)
        
        # 算法参数
        params = data.get('parameters', {})
        
        # 创建配置
        config = DWAPCBCConfig(
            environment_width=env_width,
            environment_height=env_height,
            grid_resolution=resolution,
            safety_margin=params.get('safety_margin', 2.0),
            w1_safety=params.get('w1', 0.6),
            w2_smoothness=params.get('w2', 0.4),
            curvature_threshold=params.get('curvature_threshold', 0.1),
            max_iterations=params.get('max_iterations', 50),
            trajectory_points=params.get('trajectory_points', 100)
        )
        
        # 创建规划器
        planner = DWAPCBCPlanner(config)
        planner.set_obstacles(obstacles)
        
        # 执行规划
        result = planner.plan(start, goal)
        
        if not result.success:
            return jsonify({
                'status': 'error',
                'message': '路径规划失败，无法找到可行路径'
            }), 400
        
        # 生成SVG路径字符串（用于前端可视化）
        svg_path = generate_svg_path(result.final_trajectory)
        
        # 构建响应
        response = {
            'status': 'success',
            'path': {
                'initial_waypoints': result.initial_path[:20],  # 限制数量
                'optimized_waypoints': result.optimized_waypoints,
                'trajectory_points': result.final_trajectory
            },
            'metrics': {
                'safety_distance': round(result.safety_distance, 4),
                'smoothness': round(result.smoothness, 4),
                'path_length': round(result.path_length, 4),
                'computation_time': round(result.computation_time, 4),
                'iterations': result.iterations
            },
            'environment': {
                'complexity': round(result.environment_complexity, 4),
                'num_waypoints': result.num_waypoints,
                'num_obstacles': len(obstacles)
            },
            'visualization': {
                'svg_path': svg_path
            }
        }
        
        return jsonify(response)
    
    except KeyError as e:
        return jsonify({
            'status': 'error',
            'message': f'缺少必要参数: {str(e)}'
        }), 400
    
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': f'服务器错误: {str(e)}'
        }), 500


@app.route('/api/v1/analyze-environment', methods=['POST'])
def analyze_environment():
    """
    环境分析接口
    
    请求体:
    {
        "obstacles": [...],
        "environment": {...}
    }
    """
    try:
        data = request.get_json()
        
        obstacles = data.get('obstacles', [])
        env = data.get('environment', {})
        
        env_width = env.get('width', 100.0)
        env_height = env.get('height', 100.0)
        resolution = env.get('resolution', 0.5)
        
        # 创建配置和规划器
        config = DWAPCBCConfig(
            environment_width=env_width,
            environment_height=env_height,
            grid_resolution=resolution
        )
        
        planner = DWAPCBCPlanner(config)
        planner.set_obstacles(obstacles)
        
        # 评估环境复杂度
        complexity = planner._assess_environment_complexity()
        
        # 推荐路径点数量
        recommended_waypoints = planner._determine_waypoint_count(complexity, 50)
        
        # 计算自由空间比例
        total_cells = planner.dijkstra.grid_width * planner.dijkstra.grid_height
        obstacle_cells = np.sum(planner.dijkstra.obstacle_grid)
        free_space_ratio = 1.0 - (obstacle_cells / total_cells)
        
        # 复杂度分类
        if complexity < 0.3:
            complexity_label = "low"
        elif complexity < 0.6:
            complexity_label = "medium"
        else:
            complexity_label = "high"
        
        response = {
            'status': 'success',
            'complexity': complexity_label,
            'complexity_score': round(complexity, 4),
            'obstacle_density': round(obstacle_cells / total_cells, 4),
            'free_space_ratio': round(free_space_ratio, 4),
            'recommended_waypoints': recommended_waypoints,
            'num_obstacles': len(obstacles)
        }
        
        return jsonify(response)
    
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': f'服务器错误: {str(e)}'
        }), 500


@app.route('/api/v1/suggest-parameters', methods=['POST'])
def suggest_parameters():
    """
    参数建议接口
    
    请求体:
    {
        "scenario": "indoor_navigation",
        "robot_type": "differential_drive",
        "priority": "safety"
    }
    """
    try:
        data = request.get_json()
        
        scenario = data.get('scenario', 'general')
        robot_type = data.get('robot_type', 'general')
        priority = data.get('priority', 'balanced')
        
        # 根据场景和优先级推荐参数
        if priority == 'safety':
            w1, w2 = 0.7, 0.3
            safety_margin = 2.5
        elif priority == 'smoothness':
            w1, w2 = 0.4, 0.6
            safety_margin = 1.5
        else:  # balanced
            w1, w2 = 0.6, 0.4
            safety_margin = 2.0
        
        # 根据场景调整
        if scenario == 'indoor_navigation':
            curvature_threshold = 0.08
            recommended_waypoints = 9
        elif scenario == 'outdoor_navigation':
            curvature_threshold = 0.12
            recommended_waypoints = 7
        else:
            curvature_threshold = 0.1
            recommended_waypoints = 8
        
        response = {
            'status': 'success',
            'parameters': {
                'w1': w1,
                'w2': w2,
                'safety_margin': safety_margin,
                'curvature_threshold': curvature_threshold,
                'recommended_waypoints': recommended_waypoints
            },
            'description': f'针对{scenario}场景和{priority}优先级的推荐参数'
        }
        
        return jsonify(response)
    
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': f'服务器错误: {str(e)}'
        }), 500


def generate_svg_path(trajectory):
    """生成SVG路径字符串"""
    if not trajectory:
        return ""
    
    path_parts = [f"M {trajectory[0][0]} {trajectory[0][1]}"]
    
    for point in trajectory[1:]:
        path_parts.append(f"L {point[0]} {point[1]}")
    
    return " ".join(path_parts)


if __name__ == '__main__':
    print("=" * 60)
    print("DWAPCBC Path Planning API Server")
    print("=" * 60)
    print("Starting server on http://localhost:5000")
    print("API Documentation:")
    print("  - POST /api/v1/plan - 路径规划")
    print("  - POST /api/v1/analyze-environment - 环境分析")
    print("  - POST /api/v1/suggest-parameters - 参数建议")
    print("=" * 60)
    
    app.run(host='0.0.0.0', port=5000, debug=True)

