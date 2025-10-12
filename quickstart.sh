#!/bin/bash

echo "========================================"
echo "DWAPCBC路径规划系统 - 快速开始"
echo "========================================"
echo ""

# 检查Python版本
echo "[1/5] 检查Python环境..."
python3 --version
if [ $? -ne 0 ]; then
    echo "错误: 未找到Python3"
    exit 1
fi
echo "✓ Python环境正常"
echo ""

# 安装依赖
echo "[2/5] 安装依赖包..."
pip3 install -q numpy scipy matplotlib pandas networkx shapely
if [ $? -ne 0 ]; then
    echo "警告: 部分依赖安装失败，但可能已经安装"
fi
echo "✓ 依赖安装完成"
echo ""

# 进入后端目录
cd backend

# 运行测试
echo "[3/5] 运行核心算法测试..."
python3 dwapcbc.py
if [ $? -ne 0 ]; then
    echo "错误: 核心算法测试失败"
    exit 1
fi
echo ""

# 运行示例场景
echo "[4/5] 运行示例场景..."
python3 cli.py -f scenarios/example1.json --no-viz
if [ $? -ne 0 ]; then
    echo "错误: 场景规划失败"
    exit 1
fi
echo ""

# 批量测试
echo "[5/5] 运行批量测试..."
python3 batch_test.py -d scenarios -o batch_results
if [ $? -ne 0 ]; then
    echo "错误: 批量测试失败"
    exit 1
fi
echo ""

echo "========================================"
echo "✓ 快速开始完成!"
echo "========================================"
echo ""
echo "接下来你可以:"
echo "  1. 查看批量测试结果: cat batch_results/summary.txt"
echo "  2. 运行其他场景: python3 cli.py -f scenarios/example2.json --no-viz"
echo "  3. 创建自定义场景: 参考 scenarios/ 目录下的示例"
echo "  4. 查看完整文档: cat ../README.md"
echo ""

