# AssetX USD 支持配置指南

## 🚀 快速开始

### 1. 创建Conda虚拟环境

```bash
# 创建新的conda环境
conda create -n assetx-usd python=3.10
conda activate assetx-usd

# 或者使用现有环境
conda activate your-existing-env
```

### 2. 安装USD库

```bash
# 方法1: 通过conda-forge安装 (推荐)
conda install -c conda-forge pxr

# 方法2: 通过pip安装
pip install pxr

# 方法3: 如果上述方法不行，使用USD官方版本
pip install usd-core
```

### 3. 安装AssetX及USD支持

```bash
# 安装AssetX基础版本
pip install -e .

# 安装USD支持
pip install -e ".[usd]"

# 或者安装完整版本
pip install -e ".[all]"
```

## 🔧 验证安装

```python
# 测试USD库
try:
    from pxr import Usd, UsdGeom, Sdf
    print("✅ USD库安装成功!")
except ImportError as e:
    print(f"❌ USD库安装失败: {e}")

# 测试AssetX USD支持
from assetx import Asset
asset = Asset("path/to/robot.usd")
print(f"支持的格式: {asset.supported_formats}")
```

## 📁 USD文件读取示例

```python
from assetx import Asset
from pathlib import Path

# 加载USD机器人文件
usd_file = Path("robot.usd")
asset = Asset(usd_file)

# 获取场景信息
stage = asset.get_default_stage()
print(f"场景中的Prim数量: {len(stage.get_all_prims())}")

# 遍历所有Prim
for prim in stage.traverse():
    print(f"Prim: {prim.get_path()}, 类型: {prim.get_type_name()}")
```

## 🐛 常见问题

### 问题1: ImportError: No module named 'pxr'
```bash
# 解决方案1: 重新安装
conda install -c conda-forge pxr

# 解决方案2: 使用pip
pip install usd-core
```

### 问题2: DLL加载错误 (Windows)
```bash
# 安装Visual C++ Redistributable
# 或使用conda版本
conda install -c conda-forge pxr
```

### 问题3: 版本兼容性
```bash
# 检查版本
python -c "from pxr import Usd; print(Usd.GetVersion())"

# 使用兼容版本
conda install -c conda-forge "pxr>=22.11"
```

## 📊 推荐环境配置

```yaml
# environment.yml
name: assetx-usd
channels:
  - conda-forge
  - defaults
dependencies:
  - python=3.10
  - pxr>=22.11
  - numpy>=1.20
  - pip
  - pip:
    - click>=8.0.0
    - pyyaml>=6.0
    - trimesh>=3.15.0
```

## 🎯 下一步

1. 创建虚拟环境
2. 安装USD库
3. 测试USD文件读取
4. 实现完整的USD加载逻辑
