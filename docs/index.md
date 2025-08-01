---
layout: default
title: AssetX - Multi-format Robot Simulation Asset Bridge
---

# 🧩 AssetX

## Multi-format Robot Simulation Asset Bridge Tool

AssetX 致力于解决机器人仿真生态中不同物理引擎之间的资产格式转换、统一管理、参数校验与格式生成问题。

### 🛠️ 核心功能

- **格式转换**: 支持 URDF ↔ MJCF ↔ USD ↔ Genesis JSON 的自动转换
- **物理验证**: 检查不同格式间惯性、质量、碰撞体等参数一致性
- **网格处理**: 自动缩放/重定位/简化 mesh，统一单位，生成 collision shape
- **元数据管理**: 管理 meta.yaml，记录资产语义类别、原始格式、适配历史
- **可视化预览**: 使用 trimesh 或 open3d 可视化单个资产

### 🚀 快速开始

```bash
# 安装 AssetX
pip install assetx

# 转换 URDF 到 MJCF
assetx convert --source robot.urdf --to mjcf --output robot.xml

# 验证物理参数一致性
assetx validate --ref robot.urdf --target robot.xml

# 预览资产
assetx preview --path robot.urdf
```

### 📚 文档导航

<div class="grid">
  <div class="card">
    <h3><a href="docs/installation">安装指南</a></h3>
    <p>在 Windows、Linux、macOS 上安装 AssetX</p>
  </div>
  
  <div class="card">
    <h3><a href="docs/quickstart">快速开始</a></h3>
    <p>15分钟学会 AssetX 核心功能</p>
  </div>
  
  <div class="card">
    <h3><a href="docs/api/">API 参考</a></h3>
    <p>完整的 Python API 文档</p>
  </div>
  
  <div class="card">
    <h3><a href="docs/overview">项目概览</a></h3>
    <p>了解 AssetX 的设计理念和架构</p>
  </div>
</div>

### 🎯 支持的格式

| 源格式 | 目标格式 | 状态 | 应用场景 |
|--------|----------|------|----------|
| URDF   | MJCF     | ✅ 完成 | ROS → MuJoCo |
| MJCF   | URDF     | 🔄 开发中 | MuJoCo → ROS |
| URDF   | USD      | 🔄 开发中 | ROS → Isaac Sim |
| USD    | Genesis  | 🔄 开发中 | Isaac Sim → Genesis |

### 🌟 主要特性

#### 统一资产表示
```python
from assetx import Asset

asset = Asset("robot.urdf")
asset.load()
print(f"发现 {len(asset.links)} 个链接和 {len(asset.joints)} 个关节")
```

#### 智能物理验证
```python
from assetx import PhysicsValidator

validator = PhysicsValidator()
result = validator.validate_asset(asset)
if result.is_valid:
    print("✅ 物理参数验证通过")
```

#### 高效网格处理
```python
from assetx import MeshProcessor

processor = MeshProcessor()
simplified = processor.simplify_mesh("mesh.obj", target_faces=1000)
collision = processor.generate_collision_mesh("visual.obj")
```

### 🏗️ 项目架构

```
AssetX 生态系统
├── 核心引擎
│   ├── 资产解析器 - 统一格式读取
│   ├── 转换引擎 - 跨格式转换
│   └── 验证器 - 物理参数检查
├── 处理工具
│   ├── 网格处理器 - 3D几何优化
│   ├── 元数据管理 - 版本追踪
│   └── 可视化器 - 预览和验证
└── 用户界面
    ├── Python API - 编程接口
    ├── CLI 工具 - 命令行操作
    └── Web GUI - 图形界面 (计划中)
```

### 🎓 学习路径

1. **入门** (15分钟)
   - [安装 AssetX](docs/installation)
   - [完成快速教程](docs/quickstart)
   - 转换你的第一个资产

2. **进阶** (1小时)
   - [学习 API 使用](docs/api/)
   - [掌握物理验证](docs/api/validator)
   - [优化网格资产](docs/api/mesh-processor)

3. **专家** (2+小时)
   - [自定义工作流](docs/advanced/workflows)
   - [开发扩展功能](docs/development)
   - [研究实际案例](docs/case-studies/)

### 📊 使用统计

- **🔁 支持格式**: 4+ 种主流仿真格式
- **🧮 验证规则**: 10+ 种物理参数检查
- **🧰 网格操作**: 6+ 种处理算法
- **🖥️ CLI 命令**: 6 个主要功能命令
- **🌐 平台支持**: Windows/Linux/macOS 全平台

### 🤝 开源社区

AssetX 是完全开源的项目，欢迎参与：

- **🐛 报告问题**: [GitHub Issues](https://github.com/jandan138/AssetX/issues)
- **💡 功能建议**: [GitHub Discussions](https://github.com/jandan138/AssetX/discussions)  
- **🔧 贡献代码**: [Contributing Guide](https://github.com/jandan138/AssetX/blob/main/CONTRIBUTING.md)
- **📚 改进文档**: [Documentation](https://github.com/jandan138/AssetX/tree/main/docs)

### 📈 项目状态

**当前版本**: v0.1.0 (初始发布)

**开发路线**:
- ✅ v0.1.0: 核心架构 + URDF↔MJCF 转换
- 🔄 v0.2.0: 完整 USD 支持 + Web GUI
- 📋 v1.0.0: 全格式覆盖 + 云端集成

---

<div style="text-align: center; margin: 40px 0;">
  <a href="docs/installation" class="btn btn-primary">开始使用 AssetX</a>
  <a href="https://github.com/jandan138/AssetX" class="btn btn-secondary">查看源码</a>
</div>

<style>
.grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 20px;
  margin: 30px 0;
}

.card {
  border: 1px solid #e1e4e8;
  border-radius: 6px;
  padding: 20px;
  background: #f8f9fa;
}

.card h3 {
  margin-top: 0;
  color: #0366d6;
}

.card h3 a {
  text-decoration: none;
  color: inherit;
}

.card h3 a:hover {
  text-decoration: underline;
}

.btn {
  display: inline-block;
  padding: 10px 20px;
  margin: 0 10px;
  border-radius: 4px;
  text-decoration: none;
  font-weight: bold;
}

.btn-primary {
  background-color: #28a745;
  color: white;
}

.btn-secondary {
  background-color: #6c757d;
  color: white;
}

.btn:hover {
  text-decoration: none;
  opacity: 0.9;
}
</style>
