# AssetX 安装指南

## 📋 系统要求

- **Python**: 3.8 或更高版本
- **操作系统**: Windows 10+, Ubuntu 18.04+, macOS 10.15+
- **内存**: 建议 4GB+ RAM
- **存储**: 500MB+ 可用空间

## 🚀 快速安装

### 基础安装（推荐）

```bash
pip install assetx
```

这将安装核心功能，支持：
- ✅ 基础格式转换
- ✅ 元数据管理
- ✅ CLI工具

### 完整安装

```bash
pip install 'assetx[all]'
```

包含所有可选功能：
- ✅ 3D网格处理 (trimesh)
- ✅ 高级可视化 (open3d)  
- ✅ URDF支持 (urdfpy)
- ✅ MuJoCo集成 (mujoco)

### 按需安装

根据你的具体需求选择：

```bash
# 仅网格处理
pip install 'assetx[mesh]'

# 仅可视化功能
pip install 'assetx[viewer]'

# URDF格式支持
pip install 'assetx[urdf]'

# MuJoCo集成
pip install 'assetx[mujoco]'

# 高级可视化
pip install 'assetx[viewer-open3d]'
```

## 🔧 开发安装

如果你想参与开发或使用最新功能：

```bash
# 克隆仓库
git clone https://github.com/jandan138/AssetX.git
cd AssetX

# 创建虚拟环境（推荐）
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# 开发模式安装
pip install -e .

# 安装开发依赖
pip install -e ".[dev,all]"
```

## ✅ 验证安装

运行安装验证脚本：

```bash
cd AssetX
python test_install.py
```

预期输出：
```
🧩 AssetX Installation Test
==================================================
🧪 Testing basic imports...
   ✅ AssetX version: 0.1.0
   ✅ Core asset classes imported
   ✅ Format converter imported
   ✅ Physics validator imported
   ✅ Meta manager imported

🔍 Checking optional dependencies...
   ✅ trimesh available
   ⚠️ open3d not available (install with: pip install open3d)
   ⚠️ urdfpy not available (install with: pip install urdfpy)

⚙️ Testing core functionality...
   ✅ Converter supports: ['urdf', 'mjcf', 'usd', 'genesis']
   ✅ Physics validator initialized
   ✅ Meta template created with schema: 1.0
   ✅ Mesh processor supports: ['.obj', '.stl', '.ply', '.dae', '.glb', '.gltf']

🎉 All tests passed! AssetX is ready to use.
```

## 🖥️ 命令行测试

验证CLI工具：

```bash
# 查看版本
assetx --version

# 查看帮助
assetx --help

# 测试子命令
assetx convert --help
assetx validate --help
assetx mesh --help
```

## 🐛 常见问题

### 问题1：pip install 网络错误

**解决方案**：使用国内镜像

```bash
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple assetx
```

### 问题2：open3d 安装失败

**原因**：open3d对某些系统版本要求较高

**解决方案**：
1. 先安装基础版本：`pip install assetx`
2. 单独安装open3d：`pip install open3d`
3. 或使用trimesh：`pip install 'assetx[mesh]'`

### 问题3：Windows下权限错误

**解决方案**：以管理员身份运行PowerShell或使用虚拟环境

```bash
# 创建虚拟环境避免权限问题
python -m venv assetx_env
assetx_env\Scripts\activate
pip install assetx
```

### 问题4：Python版本不兼容

**检查Python版本**：
```bash
python --version
```

**升级Python**：
- Windows: 从 [python.org](https://www.python.org/downloads/) 下载
- Ubuntu: `sudo apt update && sudo apt install python3.8`
- macOS: `brew install python@3.8`

### 问题5：依赖冲突

**解决方案**：使用干净的虚拟环境

```bash
# 删除旧环境
rm -rf venv

# 创建新环境
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install --upgrade pip
pip install assetx
```

## 🔄 升级AssetX

```bash
# 升级到最新版本
pip install --upgrade assetx

# 升级特定功能
pip install --upgrade 'assetx[all]'
```

## 🗑️ 卸载

```bash
pip uninstall assetx

# 如果使用开发模式安装
pip uninstall assetx -y
rm -rf AssetX/  # 删除源码目录
```

## 🌐 网络环境配置

### 企业防火墙

如果在企业网络环境中：

```bash
# 配置代理
pip install --proxy http://proxy.company.com:8080 assetx

# 使用可信主机
pip install --trusted-host pypi.org --trusted-host files.pythonhosted.org assetx
```

### 离线安装

```bash
# 在有网络的机器上下载
pip download assetx -d assetx_packages/

# 在离线机器上安装
pip install --no-index --find-links assetx_packages/ assetx
```

## 📱 IDE 集成

### VS Code

安装Python扩展后，AssetX CLI会自动可用：

```json
// .vscode/settings.json
{
    "python.defaultInterpreterPath": "./venv/bin/python",
    "python.terminal.activateEnvironment": true
}
```

### PyCharm

1. 配置项目解释器为AssetX虚拟环境
2. 在Terminal中可直接使用 `assetx` 命令

## 🏁 下一步

安装完成后，查看：

- [快速开始教程](quickstart.md) - 学习基本用法
- [API参考](api/README.md) - 详细的API文档  
- [示例集合](examples/README.md) - 实际使用案例
- [故障排除](troubleshooting.md) - 解决常见问题

---

**有问题？** 查看 [FAQ](faq.md) 或在 [GitHub Issues](https://github.com/jandan138/AssetX/issues) 中报告问题。
