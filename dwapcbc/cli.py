#!/usr/bin/env python3
"""
DWAPCBC路径规划命令行工具
提供交互式和批处理两种模式
"""

import argparse
import json
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle, Polygon as MplPolygon
import numpy as np

from dwapcbc import DWAPCBCPlanner, DWAPCBCConfig
plt.rcParams['font.sans-serif'] = ['SimHei']  # 适用于 Windows
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def visualize_result(planner, result, save_path=None, show=True):
    """
    可视化路径规划结果
    
    Args:
        planner: 规划器实例
        result: 规划结果
        save_path: 保存路径（可选）
        show: 是否显示图形
    """
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # 设置坐标轴
    ax.set_xlim(0, planner.config.environment_width)
    ax.set_ylim(0, planner.config.environment_height)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('DWAPCBC路径规划结果', fontsize=16, fontweight='bold')
    
    # 绘制障碍物
    for obstacle in planner.obstacles:
        if obstacle['type'] == 'rectangle':
            vertices = obstacle['vertices']
            poly = MplPolygon(vertices, facecolor='red', alpha=0.5, 
                            edgecolor='darkred', linewidth=2, label='障碍物')
            ax.add_patch(poly)
        
        elif obstacle['type'] == 'circle':
            center = obstacle['center']
            radius = obstacle['radius']
            circle = Circle(center, radius, facecolor='red', alpha=0.5,
                          edgecolor='darkred', linewidth=2)
            ax.add_patch(circle)
        
        elif obstacle['type'] == 'polygon':
            vertices = obstacle['vertices']
            poly = MplPolygon(vertices, facecolor='red', alpha=0.5,
                            edgecolor='darkred', linewidth=2)
            ax.add_patch(poly)
    
    if result.success:
        # 绘制初始路径（虚线，完整不截断）
        if len(result.initial_path) > 1:
            pts = result.initial_path  # 不截断
            initial_x = [p[0] for p in pts]
            initial_y = [p[1] for p in pts]
            ax.plot(
                initial_x, initial_y,
                linestyle=(0, (4, 4)),   # 更清晰的虚线
                linewidth=1.5,
                color='dimgray',
                alpha=0.9,
                label='初始路径',
                zorder=6                 # 稍微抬高层级，避免被遮挡
            )

        # 绘制优化后的路径点
        if len(result.optimized_waypoints) > 0:
            wp_x = [p[0] for p in result.optimized_waypoints]
            wp_y = [p[1] for p in result.optimized_waypoints]
            ax.plot(wp_x, wp_y, 'o', color='purple', markersize=8,
                   markeredgecolor='white', markeredgewidth=2, 
                   label=f'路径点 (n={len(result.optimized_waypoints)})')
        
        # 绘制最终轨迹（平滑曲线）
        if len(result.final_trajectory) > 1:
            traj_x = [p[0] for p in result.final_trajectory]
            traj_y = [p[1] for p in result.final_trajectory]
            ax.plot(traj_x, traj_y, 'b-', linewidth=3, 
                   label='最终轨迹', alpha=0.8)
        
        # 绘制起点和终点
        start = result.initial_path[0] if result.initial_path else (0, 0)
        goal = result.initial_path[-1] if result.initial_path else (100, 100)
        
        ax.plot(start[0], start[1], 'go', markersize=15, 
               markeredgecolor='white', markeredgewidth=3, label='起点')
        ax.plot(goal[0], goal[1], 'rs', markersize=15,
               markeredgecolor='white', markeredgewidth=3, label='终点')
        
        # 添加性能指标文本框
        textstr = '\n'.join([
            f'性能指标:',
            f'  安全距离: {result.safety_distance:.2f} m',
            f'  平滑度: {result.smoothness:.2f}°',
            f'  路径长度: {result.path_length:.2f} m',
            f'  计算时间: {result.computation_time:.3f} s',
            f'  迭代次数: {result.iterations}',
            f'',
            f'环境信息:',
            f'  复杂度: {result.environment_complexity:.3f}',
            f'  路径点数: {result.num_waypoints}'
        ])
        
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.9)
        ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
               verticalalignment='top', bbox=props)
    
    ax.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ 结果已保存到: {save_path}")
    
    if show:
        plt.show()
    
    plt.close()


def load_scenario_from_json(json_path):
    """从JSON文件加载场景配置"""
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return data


def run_planning(scenario_data, config_override=None, visualize=True, save_path=None):
    """
    执行路径规划
    
    Args:
        scenario_data: 场景数据字典
        config_override: 配置覆盖
        visualize: 是否可视化
        save_path: 保存路径
    """
    # 创建配置
    config = DWAPCBCConfig()
    
    # 应用场景配置
    if 'environment' in scenario_data:
        env = scenario_data['environment']
        config.environment_width = env.get('width', 100.0)
        config.environment_height = env.get('height', 100.0)
        config.grid_resolution = env.get('resolution', 0.5)
    
    if 'parameters' in scenario_data:
        params = scenario_data['parameters']
        config.w1_safety = params.get('w1', 0.6)
        config.w2_smoothness = params.get('w2', 0.4)
        config.safety_margin = params.get('safety_margin', 2.0)
        config.curvature_threshold = params.get('curvature_threshold', 0.1)
        config.max_iterations = params.get('max_iterations', 50)
    
    # 应用命令行覆盖
    if config_override:
        for key, value in config_override.items():
            if hasattr(config, key):
                setattr(config, key, value)
    
    # 创建规划器
    planner = DWAPCBCPlanner(config)
    
    # 设置障碍物
    obstacles = scenario_data.get('obstacles', [])
    planner.set_obstacles(obstacles)
    
    # 获取起点和终点
    start = tuple(scenario_data['start'].values())
    goal = tuple(scenario_data['goal'].values())
    
    # 执行规划
    print("\n" + "="*60)
    print("开始路径规划...")
    print("="*60)
    print(f"起点: ({start[0]:.1f}, {start[1]:.1f})")
    print(f"终点: ({goal[0]:.1f}, {goal[1]:.1f})")
    print(f"障碍物数量: {len(obstacles)}")
    print(f"环境大小: {config.environment_width} x {config.environment_height}")
    print("-"*60)
    
    result = planner.plan(start, goal)
    
    # 输出结果
    print("\n" + "="*60)
    if result.success:
        print("✓ 路径规划成功!")
        print("="*60)
        print(f"\n【路径信息】")
        print(f"  初始路径点数: {len(result.initial_path)}")
        print(f"  优化后路径点数: {len(result.optimized_waypoints)}")
        print(f"  最终轨迹点数: {len(result.final_trajectory)}")
        
        print(f"\n【性能指标】")
        print(f"  安全距离: {result.safety_distance:.4f} m")
        print(f"  平滑度: {result.smoothness:.4f}°")
        print(f"  路径长度: {result.path_length:.4f} m")
        print(f"  计算时间: {result.computation_time:.4f} s")
        print(f"  迭代次数: {result.iterations}")
        
        print(f"\n【环境信息】")
        print(f"  环境复杂度: {result.environment_complexity:.4f}")
        print(f"  障碍物数量: {len(obstacles)}")
        
        print("="*60)
        
        # 可视化
        if visualize:
            visualize_result(planner, result, save_path=save_path, show=True)
        
        return result
    else:
        print("✗ 路径规划失败")
        print("="*60)
        return None


def interactive_mode():
    """交互式模式"""
    print("\n" + "="*60)
    print("DWAPCBC路径规划系统 - 交互式模式")
    print("="*60)
    
    # 输入起点和终点
    print("\n请输入起点坐标:")
    start_x = float(input("  X: "))
    start_y = float(input("  Y: "))
    
    print("\n请输入终点坐标:")
    goal_x = float(input("  X: "))
    goal_y = float(input("  Y: "))
    
    # 输入障碍物
    print("\n请输入障碍物数量:")
    num_obstacles = int(input("  数量: "))
    
    obstacles = []
    for i in range(num_obstacles):
        print(f"\n障碍物 {i+1}:")
        obs_type = input("  类型 (rectangle/circle): ").strip().lower()
        
        if obs_type == 'rectangle':
            print("  请输入4个顶点坐标 (x y):")
            vertices = []
            for j in range(4):
                coords = input(f"    顶点{j+1}: ").strip().split()
                vertices.append([float(coords[0]), float(coords[1])])
            obstacles.append({'type': 'rectangle', 'vertices': vertices})
        
        elif obs_type == 'circle':
            center_x = float(input("  圆心X: "))
            center_y = float(input("  圆心Y: "))
            radius = float(input("  半径: "))
            obstacles.append({
                'type': 'circle',
                'center': [center_x, center_y],
                'radius': radius
            })
    
    # 构建场景数据
    scenario_data = {
        'start': {'x': start_x, 'y': start_y},
        'goal': {'x': goal_x, 'y': goal_y},
        'obstacles': obstacles,
        'environment': {
            'width': 100,
            'height': 100,
            'resolution': 0.5
        }
    }
    
    # 执行规划
    run_planning(scenario_data, visualize=True)


def main():
    parser = argparse.ArgumentParser(
        description='DWAPCBC路径规划命令行工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 使用JSON配置文件
  python cli.py -f scenario.json
  
  # 使用JSON配置文件并保存结果
  python cli.py -f scenario.json -o result.png
  
  # 交互式模式
  python cli.py -i
  
  # 覆盖参数
  python cli.py -f scenario.json --w1 0.7 --w2 0.3
        """
    )
    
    parser.add_argument('-f', '--file', type=str,
                       help='场景配置JSON文件路径')
    parser.add_argument('-i', '--interactive', action='store_true',
                       help='交互式模式')
    parser.add_argument('-o', '--output', type=str,
                       help='输出图像路径')
    parser.add_argument('--no-viz', action='store_true',
                       help='不显示可视化')
    
    # 参数覆盖
    parser.add_argument('--w1', type=float, help='安全性权重')
    parser.add_argument('--w2', type=float, help='平滑性权重')
    parser.add_argument('--safety-margin', type=float, help='安全裕度')
    parser.add_argument('--curvature-threshold', type=float, help='曲率阈值')
    parser.add_argument('--max-iterations', type=int, help='最大迭代次数')
    
    args = parser.parse_args()
    
    # 构建配置覆盖
    config_override = {}
    if args.w1 is not None:
        config_override['w1_safety'] = args.w1
    if args.w2 is not None:
        config_override['w2_smoothness'] = args.w2
    if args.safety_margin is not None:
        config_override['safety_margin'] = args.safety_margin
    if args.curvature_threshold is not None:
        config_override['curvature_threshold'] = args.curvature_threshold
    if args.max_iterations is not None:
        config_override['max_iterations'] = args.max_iterations
    
    # 执行模式
    if args.interactive:
        interactive_mode()
    elif args.file:
        scenario_data = load_scenario_from_json(args.file)
        run_planning(
            scenario_data,
            config_override=config_override,
            visualize=not args.no_viz,
            save_path=args.output
        )
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == '__main__':
    main()

