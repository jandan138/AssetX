# AssetX 项目概览

## 🎯 项目背景

AssetX 是一个专为机器人仿真生态设计的资产桥梁工具。在机器人研发中，我们经常需要在不同的仿真平台之间切换：

- **Isaac Sim** (NVIDIA) - 使用USD格式
- **MuJoCo** - 使用MJCF格式  
- **ROS/Gazebo** - 使用URDF格式
- **Genesis** - 使用自定义JSON格式
- **SAPIEN** - 支持多种格式

每个平台都有自己的资产格式，导致同一个机器人模型需要维护多个版本，这带来了：

❌ **格式割裂** - 同一资产需要多个版本  
❌ **参数漂移** - 转换过程中物理参数可能不一致  
❌ **重复工作** - 每次适配新平台都要重新建模  
❌ **质量控制困难** - 难以验证不同版本的一致性  

## 🚀 AssetX 解决方案

AssetX 提供了一套完整的工具链：

### 1. 统一资产表示
```python
from assetx import Asset

# 加载任意格式的资产
asset = Asset("robot.urdf")  # 或 .xml, .usd, .json
asset.load()

# 统一的数据结构
print(f"Links: {len(asset.links)}")
print(f"Joints: {len(asset.joints)}")
print(f"Physics properties: {len(asset.physics_properties)}")
```

### 2. 格式转换
```bash
# URDF → MJCF
assetx convert --source robot.urdf --to mjcf --output robot.xml

# USD → Genesis JSON
assetx convert --source robot.usd --to genesis --output robot.json
```

### 3. 物理验证
```bash
# 验证单个资产的物理参数
assetx validate --ref robot.urdf

# 比较两个格式的一致性
assetx validate --ref robot.urdf --target robot.xml
```

### 4. 网格处理
```bash
# 简化网格以提高性能
assetx mesh --path mesh.obj --operation simplify --target-faces 1000

# 生成碰撞体
assetx mesh --path visual.obj --operation collision --collision-method convex_hull
```

### 5. 元数据管理
```bash
# 注册新资产
assetx register --name panda_arm --path panda.urdf --category robot_arm

# 查看资产库
assetx meta --category robot_arm

# 导出报告
assetx meta --export report.yaml
```

## 🏗️ 架构设计

```
AssetX 架构
├── 🔁 core/              # 核心转换引擎
│   ├── asset.py          # 统一资产表示
│   ├── converter.py      # 格式转换器
│   └── validator.py      # 物理验证器
├── 🧰 mesh/              # 网格处理
│   └── processor.py      # 网格操作工具
├── 🧾 meta/              # 元数据管理
│   └── manager.py        # 资产元信息管理
├── 🔍 viewer/            # 可视化预览
│   └── preview.py        # 3D预览工具
└── 🖥️ cli.py             # 命令行界面
```

## 📊 支持的格式

| 源格式 | 目标格式 | 状态 | 用途 |
|--------|----------|------|------|
| URDF   | MJCF     | ✅   | ROS → MuJoCo |
| URDF   | USD      | 🔄   | ROS → Isaac Sim |
| MJCF   | URDF     | 🔄   | MuJoCo → ROS |
| USD    | Genesis  | 🔄   | Isaac Sim → Genesis |
| USD    | MJCF     | 📋   | Isaac Sim → MuJoCo |

图例：✅ 已实现 | 🔄 开发中 | 📋 计划中

## 🎯 使用场景

### 场景1：多平台仿真验证
```bash
# 原始URDF模型
assetx convert --source panda.urdf --to mjcf
assetx validate --ref panda.urdf --target panda.xml
# 确保MuJoCo仿真行为一致
```

### 场景2：资产库管理
```bash
# 构建组织级资产库
assetx register --name franka_panda --path panda.urdf --category robot_arm
assetx meta --export asset_report.yaml
# 团队共享标准化资产
```

### 场景3：性能优化
```bash
# 为实时仿真优化网格
assetx mesh --path high_res.obj --operation simplify --target-faces 500
assetx mesh --path visual.obj --operation collision
# 平衡视觉质量与计算性能
```

## 🔮 发展路线

### v0.1.0 (当前)
- ✅ 基础架构和CLI
- ✅ URDF ↔ MJCF 转换
- ✅ 物理参数验证
- ✅ 元数据管理

### v0.2.0 (下一版本)
- 🔄 完整的USD支持
- 🔄 Web GUI界面
- 🔄 资产质量评分系统
- 🔄 批量处理工具

### v1.0.0 (长期目标)
- 📋 完整格式覆盖
- 📋 云端资产库
- 📋 AI辅助转换优化
- 📋 VS Code扩展

## 🤝 贡献指南

AssetX是开源项目，欢迎贡献：

- 🐛 **报告Bug** - 在Issues中详细描述问题
- 💡 **功能建议** - 提出新的转换需求
- 🔧 **代码贡献** - 实现新的格式支持
- 📚 **文档改进** - 完善使用说明

## 📚 快速开始

1. **安装AssetX**
```bash
pip install assetx
# 或安装完整功能
pip install 'assetx[all]'
```

2. **转换你的第一个资产**
```bash
assetx convert --source robot.urdf --to mjcf
```

3. **验证转换质量**
```bash
assetx validate --ref robot.urdf --target robot.xml
```

4. **查看帮助**
```bash
assetx --help
```

---

**下一步**: 查看 [安装指南](installation.md) 开始使用 AssetX！
