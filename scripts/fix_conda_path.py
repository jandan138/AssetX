#!/usr/bin/env python3
"""
Windows Conda环境变量修复脚本
"""

import os
import sys
import winreg
from pathlib import Path

def find_conda_installation():
    """查找Conda安装路径"""
    print("🔍 查找Conda安装...")
    
    # 常见的Conda安装路径
    possible_paths = [
        Path.home() / "miniconda3",
        Path.home() / "anaconda3", 
        Path("C:/ProgramData/Miniconda3"),
        Path("C:/ProgramData/Anaconda3"),
        Path("C:/tools/miniconda3"),
        Path("C:/tools/anaconda3"),
    ]
    
    for path in possible_paths:
        conda_exe = path / "Scripts" / "conda.exe"
        if conda_exe.exists():
            print(f"✅ 找到Conda: {path}")
            return path
    
    print("❌ 未找到Conda安装")
    return None

def get_current_user_path():
    """获取当前用户的PATH环境变量"""
    try:
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, "Environment") as key:
            path_value, _ = winreg.QueryValueEx(key, "PATH")
            return path_value
    except FileNotFoundError:
        return ""

def set_user_path(new_path):
    """设置用户PATH环境变量"""
    try:
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, "Environment", 0, winreg.KEY_SET_VALUE) as key:
            winreg.SetValueEx(key, "PATH", 0, winreg.REG_EXPAND_SZ, new_path)
        return True
    except Exception as e:
        print(f"❌ 设置环境变量失败: {e}")
        return False

def add_conda_to_path(conda_path):
    """添加Conda路径到环境变量"""
    print("🔧 修复环境变量...")
    
    # 需要添加的路径
    conda_paths = [
        str(conda_path),
        str(conda_path / "Scripts"),
        str(conda_path / "condabin"),
        str(conda_path / "Library" / "bin"),
    ]
    
    # 获取当前PATH
    current_path = get_current_user_path()
    current_paths = current_path.split(";") if current_path else []
    
    # 去除空项和重复项
    current_paths = [p.strip() for p in current_paths if p.strip()]
    existing_paths = set(current_paths)
    
    # 添加新路径（如果不存在）
    new_paths_added = []
    for path in conda_paths:
        if path not in existing_paths:
            current_paths.insert(0, path)  # 添加到开头
            new_paths_added.append(path)
            print(f"  ➕ 添加: {path}")
        else:
            print(f"  ✅ 已存在: {path}")
    
    if new_paths_added:
        # 构建新的PATH
        new_path = ";".join(current_paths)
        
        # 设置环境变量
        if set_user_path(new_path):
            print(f"✅ 成功添加 {len(new_paths_added)} 个路径到环境变量")
            return True
        else:
            return False
    else:
        print("✅ 所有Conda路径都已在环境变量中")
        return True

def test_conda_after_fix():
    """测试修复后的Conda"""
    print("\n🧪 测试Conda...")
    
    # 更新当前进程的环境变量
    current_path = get_current_user_path()
    os.environ["PATH"] = current_path + ";" + os.environ.get("PATH", "")
    
    # 尝试运行conda
    import subprocess
    try:
        result = subprocess.run(["conda", "--version"], capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✅ Conda测试成功: {result.stdout.strip()}")
            return True
        else:
            print(f"❌ Conda测试失败: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("⏰ Conda测试超时")
        return False
    except FileNotFoundError:
        print("⚠️  需要重启PowerShell才能生效")
        return False

def main():
    """主函数"""
    print("🚀 Windows Conda环境变量修复")
    print("=" * 50)
    
    # 检查操作系统
    if os.name != 'nt':
        print("❌ 此脚本仅适用于Windows系统")
        return False
    
    # 查找Conda安装
    conda_path = find_conda_installation()
    if not conda_path:
        print("\n💡 解决方案:")
        print("1. 安装Miniconda: https://docs.conda.io/en/latest/miniconda.html")
        print("2. 或者使用当前Python环境 (已经有USD支持)")
        return False
    
    # 添加到环境变量
    if add_conda_to_path(conda_path):
        print("\n✅ 环境变量修复完成!")
        
        # 测试
        if test_conda_after_fix():
            print("\n🎉 Conda现在可以直接使用了!")
        else:
            print("\n📝 请按以下步骤完成:")
            print("1. 关闭当前PowerShell窗口")
            print("2. 重新打开PowerShell")
            print("3. 测试: conda --version")
            print("4. 创建环境: conda create -n assetx-usd python=3.10")
        
        return True
    else:
        print("\n❌ 环境变量修复失败")
        print("\n💡 手动解决方案:")
        print(f"1. 打开系统环境变量设置")
        print(f"2. 在用户PATH中添加:")
        for path in [conda_path, conda_path / "Scripts", conda_path / "condabin"]:
            print(f"   {path}")
        return False

if __name__ == "__main__":
    try:
        success = main()
        input("\n按回车键退出...")
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⛔ 用户取消操作")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        input("\n按回车键退出...")
        sys.exit(1)
