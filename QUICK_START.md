# 🚀 AssetX USD环境快速启动脚本

为了方便开发，提供了多种方式快速进入AssetX USD开发环境。

## 📁 脚本文件

### 1. Windows批处理文件 (推荐)
- **文件**: `activate_assetx.bat`
- **使用**: 双击运行或命令行执行
- **特点**: 最简单，直接双击即可

### 2. PowerShell脚本 (高级)
- **文件**: `activate_assetx.ps1`
- **使用**: 右键"使用PowerShell运行"
- **特点**: 更好的状态检查和显示

### 3. Python脚本 (跨平台)
- **文件**: `scripts/activate_assetx.py`
- **使用**: `python scripts/activate_assetx.py`
- **特点**: 支持Windows/Linux/macOS

## 🎯 使用方法

### 方法1: 最简单 (推荐)
1. 在AssetX项目根目录双击 `activate_assetx.bat`
2. 等待环境激活
3. 开始开发！

### 方法2: PowerShell
```powershell
# 在项目根目录
.\activate_assetx.ps1
```

### 方法3: Python
```powershell
python scripts/activate_assetx.py
```

## ✅ 脚本功能

- **🔍 环境检查**: 验证目录、conda、环境是否正确
- **🔧 自动激活**: 激活assetx-usd conda环境
- **📊 状态显示**: 检查Python、USD、AssetX状态
- **💡 使用提示**: 显示常用命令
- **🚀 保持环境**: 启动新shell并保持在环境中

## 🛠️ 故障排除

### 如果环境不存在
```powershell
conda env create -f environment.yml
```

### 如果提示PowerShell执行策略错误
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### 如果conda不可用
运行修复脚本:
```powershell
python scripts/fix_conda_path.py
```

## 🎉 成功启动后

你会看到类似的输出:
```
🎉 AssetX USD开发环境已就绪!

📊 环境状态:
   Python: Python 3.10.x
   USD: ✅ USD版本: (25, 5, 1)
   AssetX: ✅ 可用

🎯 常用命令:
   测试USD功能:  python examples/simple_usd_test.py
   运行测试:     pytest tests/
   查看帮助:     python -m assetx.cli --help
   退出环境:     conda deactivate
```

现在你可以开始使用AssetX处理USD机器人文件了！🤖
