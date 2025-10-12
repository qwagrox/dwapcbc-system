#!/usr/bin/env python3
"""
批量测试脚本
对多个场景进行路径规划并生成对比报告
"""

import os
import json
import glob
import time
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd

from dwapcbc import DWAPCBCPlanner, DWAPCBCConfig


def run_batch_tests(scenario_dir='scenarios', output_dir='batch_results'):
    """
    批量运行测试场景
    
    Args:
        scenario_dir: 场景文件目录
        output_dir: 输出目录
    """
    # 创建输出目录
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    # 获取所有场景文件
    scenario_files = glob.glob(os.path.join(scenario_dir, '*.json'))
    
    if not scenario_files:
        print(f"未找到场景文件在目录: {scenario_dir}")
        return
    
    print("="*80)
    print(f"批量测试 - 共 {len(scenario_files)} 个场景")
    print("="*80)
    
    results = []
    
    for i, scenario_file in enumerate(scenario_files, 1):
        print(f"\n[{i}/{len(scenario_files)}] 测试场景: {os.path.basename(scenario_file)}")
        print("-"*80)
        
        try:
            # 加载场景
            with open(scenario_file, 'r', encoding='utf-8') as f:
                scenario_data = json.load(f)
            
            # 创建配置
            config = DWAPCBCConfig()
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
            
            # 创建规划器
            planner = DWAPCBCPlanner(config)
            obstacles = scenario_data.get('obstacles', [])
            planner.set_obstacles(obstacles)
            
            # 执行规划
            start = tuple(scenario_data['start'].values())
            goal = tuple(scenario_data['goal'].values())
            
            result = planner.plan(start, goal)
            
            # 记录结果
            scenario_name = scenario_data.get('name', os.path.basename(scenario_file))
            
            if result.success:
                print(f"✓ 成功")
                print(f"  安全距离: {result.safety_distance:.2f} m")
                print(f"  平滑度: {result.smoothness:.2f}°")
                print(f"  路径长度: {result.path_length:.2f} m")
                print(f"  计算时间: {result.computation_time:.3f} s")
                
                results.append({
                    'scenario': scenario_name,
                    'file': os.path.basename(scenario_file),
                    'success': True,
                    'safety_distance': result.safety_distance,
                    'smoothness': result.smoothness,
                    'path_length': result.path_length,
                    'computation_time': result.computation_time,
                    'iterations': result.iterations,
                    'num_waypoints': result.num_waypoints,
                    'environment_complexity': result.environment_complexity,
                    'num_obstacles': len(obstacles)
                })
            else:
                print(f"✗ 失败")
                results.append({
                    'scenario': scenario_name,
                    'file': os.path.basename(scenario_file),
                    'success': False,
                    'safety_distance': 0,
                    'smoothness': 0,
                    'path_length': 0,
                    'computation_time': result.computation_time,
                    'iterations': 0,
                    'num_waypoints': 0,
                    'environment_complexity': 0,
                    'num_obstacles': len(obstacles)
                })
        
        except Exception as e:
            print(f"✗ 错误: {str(e)}")
            results.append({
                'scenario': os.path.basename(scenario_file),
                'file': os.path.basename(scenario_file),
                'success': False,
                'error': str(e)
            })
    
    # 生成报告
    print("\n" + "="*80)
    print("生成测试报告...")
    print("="*80)
    
    generate_report(results, output_dir)
    
    print(f"\n✓ 批量测试完成，结果保存在: {output_dir}")


def generate_report(results, output_dir):
    """生成测试报告"""
    
    # 转换为DataFrame
    df = pd.DataFrame(results)
    
    # 保存CSV
    csv_path = os.path.join(output_dir, 'test_results.csv')
    df.to_csv(csv_path, index=False, encoding='utf-8-sig')
    print(f"✓ CSV报告: {csv_path}")
    
    # 生成统计摘要
    summary_path = os.path.join(output_dir, 'summary.txt')
    with open(summary_path, 'w', encoding='utf-8') as f:
        f.write("="*80 + "\n")
        f.write("DWAPCBC批量测试报告\n")
        f.write("="*80 + "\n\n")
        
        success_count = df['success'].sum()
        total_count = len(df)
        
        f.write(f"总测试数: {total_count}\n")
        f.write(f"成功: {success_count}\n")
        f.write(f"失败: {total_count - success_count}\n")
        f.write(f"成功率: {success_count/total_count*100:.1f}%\n\n")
        
        if success_count > 0:
            successful_df = df[df['success'] == True]
            
            f.write("性能统计 (仅成功案例):\n")
            f.write("-"*80 + "\n")
            f.write(f"平均安全距离: {successful_df['safety_distance'].mean():.4f} m\n")
            f.write(f"平均平滑度: {successful_df['smoothness'].mean():.4f}°\n")
            f.write(f"平均路径长度: {successful_df['path_length'].mean():.4f} m\n")
            f.write(f"平均计算时间: {successful_df['computation_time'].mean():.4f} s\n")
            f.write(f"平均迭代次数: {successful_df['iterations'].mean():.2f}\n")
            f.write(f"平均路径点数: {successful_df['num_waypoints'].mean():.2f}\n\n")
            
            f.write("详细结果:\n")
            f.write("-"*80 + "\n")
            for _, row in successful_df.iterrows():
                f.write(f"\n场景: {row['scenario']}\n")
                f.write(f"  文件: {row['file']}\n")
                f.write(f"  安全距离: {row['safety_distance']:.4f} m\n")
                f.write(f"  平滑度: {row['smoothness']:.4f}°\n")
                f.write(f"  路径长度: {row['path_length']:.4f} m\n")
                f.write(f"  计算时间: {row['computation_time']:.4f} s\n")
                f.write(f"  环境复杂度: {row['environment_complexity']:.4f}\n")
    
    print(f"✓ 文本摘要: {summary_path}")
    
    # 生成可视化对比图
    if df['success'].sum() > 0:
        generate_comparison_charts(df[df['success'] == True], output_dir)


def generate_comparison_charts(df, output_dir):
    """生成对比图表"""
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('DWAPCBC批量测试性能对比', fontsize=16, fontweight='bold')
    
    scenarios = df['scenario'].tolist()
    
    # 安全距离对比
    ax1 = axes[0, 0]
    ax1.bar(range(len(scenarios)), df['safety_distance'], color='#3b82f6', alpha=0.7)
    ax1.set_xlabel('场景')
    ax1.set_ylabel('安全距离 (m)')
    ax1.set_title('安全距离对比')
    ax1.set_xticks(range(len(scenarios)))
    ax1.set_xticklabels(range(1, len(scenarios)+1))
    ax1.grid(axis='y', alpha=0.3)
    
    # 平滑度对比
    ax2 = axes[0, 1]
    ax2.bar(range(len(scenarios)), df['smoothness'], color='#10b981', alpha=0.7)
    ax2.set_xlabel('场景')
    ax2.set_ylabel('平滑度 (°)')
    ax2.set_title('平滑度对比 (越小越好)')
    ax2.set_xticks(range(len(scenarios)))
    ax2.set_xticklabels(range(1, len(scenarios)+1))
    ax2.grid(axis='y', alpha=0.3)
    
    # 计算时间对比
    ax3 = axes[1, 0]
    ax3.bar(range(len(scenarios)), df['computation_time'], color='#f59e0b', alpha=0.7)
    ax3.set_xlabel('场景')
    ax3.set_ylabel('计算时间 (s)')
    ax3.set_title('计算时间对比')
    ax3.set_xticks(range(len(scenarios)))
    ax3.set_xticklabels(range(1, len(scenarios)+1))
    ax3.grid(axis='y', alpha=0.3)
    
    # 路径长度对比
    ax4 = axes[1, 1]
    ax4.bar(range(len(scenarios)), df['path_length'], color='#8b5cf6', alpha=0.7)
    ax4.set_xlabel('场景')
    ax4.set_ylabel('路径长度 (m)')
    ax4.set_title('路径长度对比')
    ax4.set_xticks(range(len(scenarios)))
    ax4.set_xticklabels(range(1, len(scenarios)+1))
    ax4.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    
    chart_path = os.path.join(output_dir, 'comparison_charts.png')
    plt.savefig(chart_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"✓ 对比图表: {chart_path}")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='DWAPCBC批量测试工具')
    parser.add_argument('-d', '--dir', type=str, default='scenarios',
                       help='场景文件目录 (默认: scenarios)')
    parser.add_argument('-o', '--output', type=str, default='batch_results',
                       help='输出目录 (默认: batch_results)')
    
    args = parser.parse_args()
    
    run_batch_tests(args.dir, args.output)

