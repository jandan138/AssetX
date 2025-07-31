#!/usr/bin/env python3
"""
AssetX USD环境快速启动脚本 (Python版本)
支持跨平台使用
"""

import os
import sys
import subprocess
import platform
from pathlib import Path

def check_environment():
    """检查当前目录和环境"""
    print("🔍 环境检查...")
    
    # 检查是否在正确目录
    if not Path("assetx").exists():
        print("❌ 错误: 请在AssetX项目根目录运行此脚本")
        print(f"   当前目录: {os.getcwd()}")
        print("   应该包含: assetx 文件夹")
        return False
    
    # 检查conda是否可用
    try:
        result = subprocess.run(["conda", "--version"], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ Conda可用: {result.stdout.strip()}")
            return True
        else:
            print("❌ Conda不可用")
            return False
    except FileNotFoundError:
        print("❌ Conda未安装或未在PATH中")
        return False

def activate_environment():
    """激活assetx-usd环境"""
    print("\n🔧 激活 assetx-usd 环境...")
    
    # 检查环境是否存在
    try:
        result = subprocess.run(
            ["conda", "env", "list"], 
            capture_output=True, 
            text=True
        )
        if "assetx-usd" not in result.stdout:
            print("❌ assetx-usd 环境不存在")
            print("💡 请先运行以下命令创建环境:")
            print("   conda env create -f environment.yml")
            return False
    except Exception as e:
        print(f"❌ 检查环境失败: {e}")
        return False
    
    print("✅ 找到 assetx-usd 环境")
    return True

def check_packages():
    """检查包安装状态"""
    print("\n📊 检查包状态...")
    
    # 构建激活环境的命令
    if platform.system() == "Windows":
        activate_cmd = f"conda activate assetx-usd && "
    else:
        activate_cmd = f"source activate assetx-usd && "
    
    # 检查Python
    try:
        result = subprocess.run(
            activate_cmd + "python --version",
            shell=True,
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            print(f"   Python: {result.stdout.strip()}")
        else:
            print("   Python: ❌ 无法检测")
    except Exception:
        print("   Python: ❌ 检查失败")
    
    # 检查USD
    try:
        result = subprocess.run(
            activate_cmd + 'python -c "from pxr import Usd; print(f\\"USD版本: {Usd.GetVersion()}\\")"',
            shell=True,
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            print(f"   USD: ✅ {result.stdout.strip()}")
        else:
            print("   USD: ❌ 未安装或有问题")
    except Exception:
        print("   USD: ❌ 检查失败")
    
    # 检查AssetX
    try:
        result = subprocess.run(
            activate_cmd + 'python -c "from assetx import Asset; print(\\"AssetX可用\\")"',
            shell=True,
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            print("   AssetX: ✅ 可用")
        else:
            print("   AssetX: ❌ 未安装")
    except Exception:
        print("   AssetX: ❌ 检查失败")

def show_usage_info():
    """显示使用信息"""
    print("\n🎯 常用命令:")
    print("   测试USD功能:  python examples/simple_usd_test.py")
    print("   运行测试:     pytest tests/")
    print("   查看帮助:     python -m assetx.cli --help")
    print("   退出环境:     conda deactivate")

def launch_environment():
    """启动环境"""
    print("\n🚀 启动AssetX USD开发环境...")
    
    system = platform.system()
    
    if system == "Windows":
        # Windows使用PowerShell
        activation_script = """
        conda activate assetx-usd
        Write-Host "🎉 AssetX USD开发环境已就绪!" -ForegroundColor Green
        Write-Host "💡 使用 'conda deactivate' 退出环境" -ForegroundColor Yellow
        """
        subprocess.run([
            "powershell", "-NoExit", "-Command", activation_script
        ])
    else:
        # Linux/macOS使用bash
        activation_script = f"""
        source $(conda info --base)/etc/profile.d/conda.sh
        conda activate assetx-usd
        echo "🎉 AssetX USD开发环境已就绪!"
        echo "💡 使用 'conda deactivate' 退出环境"
        exec bash
        """
        subprocess.run([
            "bash", "-c", activation_script
        ])

def main():
    """主函数"""
    print("🚀 AssetX USD开发环境快速启动")
    print("=" * 50)
    
    # 环境检查
    if not check_environment():
        input("\n按回车键退出...")
        return False
    
    # 激活环境
    if not activate_environment():
        input("\n按回车键退出...")
        return False
    
    # 检查包状态
    check_packages()
    
    # 显示使用信息
    show_usage_info()
    
    # 启动环境
    print("\n" + "=" * 50)
    launch_environment()
    
    return True

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⛔ 用户取消")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        input("按回车键退出...")
