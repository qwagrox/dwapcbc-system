# DWAPCBC安装指南

## 📦 安装方式

### 方式1: 从PyPI安装（推荐）

```bash
pip install dwapcbc
```

### 方式2: 从GitHub安装

```bash
pip install git+https://github.com/qwagrox/dwapcbc-system.git
```

### 方式3: 从源码安装

```bash
# 克隆仓库
git clone https://github.com/qwagrox/dwapcbc-system.git
cd dwapcbc-system

# 安装
pip install -e .
```

### 方式4: 开发模式安装

```bash
# 克隆仓库
git clone https://github.com/qwagrox/dwapcbc-system.git
cd dwapcbc-system

# 安装开发依赖
pip install -e ".[dev]"
```

---

## 🔧 依赖要求

### 必需依赖

- Python >= 3.8
- numpy >= 1.20.0
- scipy >= 1.7.0
- matplotlib >= 3.4.0
- pandas >= 1.3.0
- networkx >= 2.6.0
- shapely >= 1.8.0

### 可选依赖

**API服务**（如需使用Flask API）：
```bash
pip install dwapcbc[api]
```

**开发工具**（如需参与开发）：
```bash
pip install dwapcbc[dev]
```

**完整安装**（包含所有可选依赖）：
```bash
pip install dwapcbc[all]
```

---

## ✅ 验证安装

### 方法1: Python导入测试

```python
import dwapcbc

# 查看版本
print(dwapcbc.get_version())

# 查看库信息
print(dwapcbc.get_info())
```

### 方法2: 命令行测试

```bash
# 查看帮助
dwapcbc-plan --help

# 运行示例
dwapcbc-plan -f scenarios/example1.json --no-viz
```

### 方法3: 完整功能测试

```python
from dwapcbc import DWAPCBCPlanner, DWAPCBCConfig

# 创建规划器
config = DWAPCBCConfig()
planner = DWAPCBCPlanner(config)

# 设置障碍物
obstacles = [
    {'type': 'circle', 'center': [50, 50], 'radius': 10}
]
planner.set_obstacles(obstacles)

# 执行规划
result = planner.plan((10, 10), (90, 90))

if result.success:
    print("✓ 安装成功！")
    print(f"路径长度: {result.path_length:.2f} m")
else:
    print("✗ 安装可能有问题")
```

---

## 🐛 常见问题

### Q1: 安装时提示"No module named 'setuptools'"

**解决方案**：
```bash
pip install --upgrade setuptools wheel
```

### Q2: NumPy/SciPy安装失败

**解决方案**：
```bash
# 对于Ubuntu/Debian
sudo apt-get install python3-dev

# 对于macOS
brew install python@3.11

# 然后重新安装
pip install numpy scipy
```

### Q3: Shapely安装失败

**解决方案**：
```bash
# 对于Ubuntu/Debian
sudo apt-get install libgeos-dev

# 对于macOS
brew install geos

# 然后重新安装
pip install shapely
```

### Q4: 导入时提示"ModuleNotFoundError"

**解决方案**：
```bash
# 确认安装路径
pip show dwapcbc

# 重新安装
pip uninstall dwapcbc
pip install dwapcbc
```

---

## 🔄 升级

```bash
# 从PyPI升级
pip install --upgrade dwapcbc

# 从GitHub升级
pip install --upgrade git+https://github.com/qwagrox/dwapcbc-system.git
```

---

## 🗑️ 卸载

```bash
pip uninstall dwapcbc
```

---

## 📚 下一步

安装成功后，请查看：

1. **快速开始**：[README.md](README.md)
2. **详细教程**：[USAGE_GUIDE.md](USAGE_GUIDE.md)
3. **API文档**：[API_REFERENCE.md](API_REFERENCE.md)
4. **示例代码**：[examples/](examples/)

---

## 💬 获取帮助

如果遇到安装问题，请：

1. 查看[常见问题](#常见问题)
2. 提交[Issue](https://github.com/qwagrox/dwapcbc-system/issues)
3. 查看[文档](https://github.com/qwagrox/dwapcbc-system)

---

**版本**：1.0.0  
**更新日期**：2025-10

