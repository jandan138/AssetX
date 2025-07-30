# 🧩 AssetX – 多格式机器人仿真资产桥梁工具

AssetX 是一个轻量级、可扩展的资产管理工具，致力于解决机器人仿真生态中不同物理引擎（如 Isaac Sim、MuJoCo、MJX、SAPIEN、Genesis、Newton Warp）之间的资产格式转换、统一管理、参数校验与格式生成问题。

🛠️ 支持 URDF ↔ MJCF ↔ USD ↔ Genesis JSON 的自动转换与可视化检查  
🌐 构建多物理后端共享资产库，加速多环境部署、仿真训练与 sim2real 工程

## 🚀 核心功能

| 模块 | 功能描述 |
|------|----------|
| 🔁 FormatConverter | 支持 URDF ↔ MJCF ↔ USD，USD → Genesis JSON 等格式转换 |
| 🧮 PhysicsValidator | 检查不同格式间惯性、质量、碰撞体等参数一致性，避免行为漂移 |
| 🧰 MeshProcessor | 自动缩放/重定位/简化 mesh，统一单位（m/kg），生成 collision shape 等 |
| 🧩 AssetRouter | 自动根据目标后端选择最优资产版本，简化多引擎部署 |
| 🔍 Previewer | 使用 trimesh 或 open3d 可视化单个资产，检查是否正确加载 |
| 🧾 MetaManager | 管理 meta.yaml，记录资产语义类别、原始格式、适配历史等元信息 |

## 📦 安装方式

```bash
git clone https://github.com/your-name/assetx.git
cd assetx
pip install -e .
```

依赖库建议：
```bash
pip install urdfpy mujoco pyusd trimesh open3d pyyaml click
```

## 🧪 使用示例

```bash
# 1. 将 URDF 转为 MJCF
assetx convert --source panda.urdf --to mjcf --output ./converted/panda.xml

# 2. 从 USD 生成 Genesis 格式
assetx convert --source panda.usd --to genesis --output ./converted/panda.json

# 3. 可视化一个 URDF 模型
assetx preview --path panda.urdf

# 4. 检查 URDF 与 MJCF 的质量参数是否一致
assetx validate --ref panda.urdf --target panda.xml
```

## ✅ 未来计划

- [ ] 增加 USD ↔ MJCF 直接映射（通过中间抽象层）
- [ ] 支持 OMPL / MoveIt 系列描述的自定义标签转换
- [ ] 构建 Web GUI / VS Code 插件做资产浏览与校验
- [ ] 收录常见仿真平台资产（YCB 物体、Panda、Fetch 等）作为示例库

## 🤝 欢迎协作

本项目处于 alpha 阶段，欢迎开 issue、提 PR、一起定义标准。  
📧 如果你来自具身智能、仿真平台或机器人软件开发背景，欢迎一起构建跨引擎资产生态。
