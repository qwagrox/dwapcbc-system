#!/usr/bin/env python3
"""
DWAPCBC客户端应用示例
演示如何在其他应用中调用DWAPCBC库
"""

from dwapcbc import DWAPCBCPlanner, DWAPCBCConfig

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def example1_basic_usage():
    """示例1: 基本使用"""
    print("="*60)
    print("示例1: 基本使用")
    print("="*60)
    
    # 1. 创建配置
    config = DWAPCBCConfig(
        environment_width=100,
        environment_height=100,
        w1_safety=0.6,
        w2_smoothness=0.4
    )
    
    # 2. 创建规划器
    planner = DWAPCBCPlanner(config)
    
    # 3. 设置障碍物
    obstacles = [
        {'type': 'rectangle', 'vertices': [[20, 20], [20, 40], [40, 40], [40, 20]]},
        {'type': 'circle', 'center': [60, 60], 'radius': 8}
    ]
    planner.set_obstacles(obstacles)
    
    # 4. 执行规划
    result = planner.plan((5, 5), (95, 95))
    
    # 5. 处理结果
    if result.success:
        print(f"✓ 规划成功")
        print(f"  路径长度: {result.path_length:.2f} m")
        print(f"  安全距离: {result.safety_distance:.2f} m")
        print(f"  平滑度: {result.smoothness:.2f}°")
        print(f"  计算时间: {result.computation_time:.3f} s")
    else:
        print("✗ 规划失败")


def example2_robot_navigation():
    """示例2: 机器人导航应用"""
    print("\n" + "="*60)
    print("示例2: 机器人导航应用")
    print("="*60)
    
    class RobotNavigator:
        """机器人导航器"""
        
        def __init__(self, robot_width=0.5):
            self.robot_width = robot_width
            
            # 创建规划器（安全优先）
            config = DWAPCBCConfig(
                w1_safety=0.7,
                w2_smoothness=0.3,
                safety_margin=robot_width * 2
            )
            self.planner = DWAPCBCPlanner(config)
        
        def update_map(self, obstacles):
            """更新地图"""
            self.planner.set_obstacles(obstacles)
            print(f"✓ 地图更新: {len(obstacles)} 个障碍物")
        
        def navigate_to(self, current_pos, target_pos):
            """导航到目标位置"""
            print(f"导航: {current_pos} → {target_pos}")
            
            result = self.planner.plan(current_pos, target_pos)
            
            if result.success:
                print(f"✓ 路径规划成功")
                print(f"  预计距离: {result.path_length:.2f} m")
                print(f"  预计时间: {result.path_length / 1.0:.1f} s (假设速度1m/s)")
                return result.final_trajectory
            else:
                print("✗ 无法找到路径")
                return None
    
    # 使用机器人导航器
    robot = RobotNavigator(robot_width=0.5)
    
    # 更新地图
    obstacles = [
        {'type': 'rectangle', 'vertices': [[30, 30], [30, 50], [35, 50], [35, 30]]},
        {'type': 'circle', 'center': [70, 70], 'radius': 5}
    ]
    robot.update_map(obstacles)
    
    # 导航
    path = robot.navigate_to((10, 10), (90, 90))
    
    if path:
        print(f"✓ 路径包含 {len(path)} 个点")


def example3_multi_task_planning():
    """示例3: 多任务规划"""
    print("\n" + "="*60)
    print("示例3: 多任务规划")
    print("="*60)
    
    # 创建规划器
    planner = DWAPCBCPlanner(DWAPCBCConfig())
    
    # 设置环境
    obstacles = [
        {'type': 'circle', 'center': [50, 50], 'radius': 10}
    ]
    planner.set_obstacles(obstacles)
    
    # 多个任务
    tasks = [
        ("任务1", (10, 10), (90, 90)),
        ("任务2", (10, 90), (90, 10)),
        ("任务3", (50, 10), (50, 90))
    ]
    
    results = []
    for name, start, goal in tasks:
        result = planner.plan(start, goal)
        results.append((name, result))
        
        if result.success:
            print(f"✓ {name}: {result.path_length:.2f} m, {result.computation_time:.3f} s")
        else:
            print(f"✗ {name}: 失败")
    
    # 统计
    success_count = sum(1 for _, r in results if r.success)
    print(f"\n总计: {success_count}/{len(tasks)} 成功")


def example4_parameter_comparison():
    """示例4: 参数对比"""
    print("\n" + "="*60)
    print("示例4: 参数对比")
    print("="*60)
    
    # 不同配置
    configs = {
        '安全优先': DWAPCBCConfig(w1_safety=0.7, w2_smoothness=0.3, safety_margin=2.5),
        '平滑优先': DWAPCBCConfig(w1_safety=0.4, w2_smoothness=0.6, safety_margin=1.5),
        '平衡模式': DWAPCBCConfig(w1_safety=0.6, w2_smoothness=0.4, safety_margin=2.0)
    }
    
    obstacles = [
        {'type': 'rectangle', 'vertices': [[40, 40], [40, 60], [60, 60], [60, 40]]}
    ]
    
    print(f"{'模式':<10} {'安全距离':<12} {'平滑度':<12} {'路径长度':<12}")
    print("-" * 50)
    
    for name, config in configs.items():
        planner = DWAPCBCPlanner(config)
        planner.set_obstacles(obstacles)
        result = planner.plan((10, 50), (90, 50))
        
        if result.success:
            print(f"{name:<10} {result.safety_distance:<12.2f} {result.smoothness:<12.2f} {result.path_length:<12.2f}")


def example5_visualization():
    """示例5: 可视化"""
    print("\n" + "="*60)
    print("示例5: 路径可视化")
    print("="*60)
    
    if not HAS_MATPLOTLIB:
        print("⚠ matplotlib未安装，跳过可视化示例")
        return
    
    # 创建规划器
    planner = DWAPCBCPlanner(DWAPCBCConfig())
    
    # 设置障碍物
    obstacles = [
        {'type': 'rectangle', 'vertices': [[20, 20], [20, 40], [40, 40], [40, 20]]},
        {'type': 'circle', 'center': [60, 60], 'radius': 8},
        {'type': 'rectangle', 'vertices': [[70, 10], [70, 30], [85, 30], [85, 10]]}
    ]
    planner.set_obstacles(obstacles)
    
    # 规划
    result = planner.plan((5, 5), (95, 95))
    
    if result.success:
        # 创建图形
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # 绘制障碍物
        for obs in obstacles:
            if obs['type'] == 'rectangle':
                from matplotlib.patches import Polygon
                poly = Polygon(obs['vertices'], facecolor='red', alpha=0.5)
                ax.add_patch(poly)
            elif obs['type'] == 'circle':
                from matplotlib.patches import Circle
                circle = Circle(obs['center'], obs['radius'], facecolor='red', alpha=0.5)
                ax.add_patch(circle)
        
        # 绘制路径
        x = [p[0] for p in result.final_trajectory]
        y = [p[1] for p in result.final_trajectory]
        ax.plot(x, y, 'b-', linewidth=2, label='规划路径')
        
        # 绘制起点和终点
        ax.plot(5, 5, 'go', markersize=12, label='起点')
        ax.plot(95, 95, 'rs', markersize=12, label='终点')
        
        # 设置
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('DWAPCBC路径规划结果')
        
        # 保存
        output_path = 'client_example_path.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ 可视化已保存: {output_path}")
        plt.close()


def example6_real_time_update():
    """示例6: 实时更新场景"""
    print("\n" + "="*60)
    print("示例6: 实时路径更新")
    print("="*60)
    
    planner = DWAPCBCPlanner(DWAPCBCConfig())
    
    # 初始障碍物
    obstacles = [
        {'type': 'circle', 'center': [30, 50], 'radius': 5}
    ]
    planner.set_obstacles(obstacles)
    
    # 初始规划
    current_pos = (10, 50)
    goal = (90, 50)
    
    result = planner.plan(current_pos, goal)
    print(f"初始规划: {result.path_length:.2f} m")
    
    # 模拟检测到新障碍物
    print("\n检测到新障碍物...")
    obstacles.append({'type': 'circle', 'center': [50, 50], 'radius': 8})
    planner.set_obstacles(obstacles)
    
    # 重新规划
    result = planner.plan(current_pos, goal)
    if result.success:
        print(f"重新规划: {result.path_length:.2f} m")
    else:
        print("重新规划失败，需要寻找替代路径")


def main():
    """运行所有示例"""
    print("\n" + "="*60)
    print("DWAPCBC客户端应用示例")
    print("="*60 + "\n")
    
    # 运行示例
    example1_basic_usage()
    example2_robot_navigation()
    example3_multi_task_planning()
    example4_parameter_comparison()
    example5_visualization()
    example6_real_time_update()
    
    print("\n" + "="*60)
    print("所有示例运行完成")
    print("="*60)


if __name__ == '__main__':
    main()

