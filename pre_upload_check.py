#!/usr/bin/env python3
"""
AssetX 项目完整性检查和准备上传验证
"""

import os
import subprocess
from pathlib import Path

def check_project_structure():
    """检查项目结构完整性"""
    print("🔍 Checking project structure...")
    
    required_files = [
        "README.md",
        "LICENSE", 
        "pyproject.toml",
        ".gitignore",
        "CHANGELOG.md",
        "CONTRIBUTING.md",
        "assetx/__init__.py",
        "assetx/cli.py",
        "assetx/core/asset.py",
        "assetx/core/converter.py", 
        "assetx/core/validator.py",
        "assetx/mesh/processor.py",
        "assetx/meta/manager.py",
        "assetx/viewer/preview.py",
        "test_install.py",
        "tests/test_basic.py",
        "docs/meta_schema.md",
        "docs/development.md",
        "examples/basic_usage.py",
        "examples/meta.yaml",
        ".github/workflows/ci.yml",
        ".github/ISSUE_TEMPLATE/bug_report.md",
        ".github/ISSUE_TEMPLATE/feature_request.md"
    ]
    
    missing_files = []
    for file_path in required_files:
        if not Path(file_path).exists():
            missing_files.append(file_path)
    
    if missing_files:
        print(f"   ❌ Missing files: {missing_files}")
        return False
    else:
        print(f"   ✅ All {len(required_files)} required files present")
        return True

def check_git_status():
    """检查Git状态"""
    print("\n📝 Checking Git status...")
    
    try:
        # 检查是否有未提交的更改
        result = subprocess.run(['git', 'status', '--porcelain'], 
                              capture_output=True, text=True, check=True)
        
        if result.stdout.strip():
            print("   ⚠️ Uncommitted changes found:")
            print(result.stdout)
            return False
        else:
            print("   ✅ Working directory clean")
            
        # 检查提交历史
        result = subprocess.run(['git', 'log', '--oneline', '-1'], 
                              capture_output=True, text=True, check=True)
        print(f"   ✅ Latest commit: {result.stdout.strip()}")
        
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"   ❌ Git error: {e}")
        return False

def check_installation():
    """检查安装状态"""
    print("\n🔧 Checking installation...")
    
    try:
        import assetx
        print(f"   ✅ AssetX imported successfully, version: {assetx.__version__}")
        
        # 测试CLI
        result = subprocess.run(['python', '-m', 'assetx.cli', '--help'], 
                              capture_output=True, text=True, check=True)
        print("   ✅ CLI accessible")
        
        return True
        
    except Exception as e:
        print(f"   ❌ Installation check failed: {e}")
        return False

def check_dependencies():
    """检查依赖情况"""
    print("\n📦 Checking dependencies...")
    
    core_deps = ['click', 'pyyaml', 'numpy']
    optional_deps = ['trimesh', 'open3d', 'urdfpy', 'mujoco']
    
    for dep in core_deps:
        try:
            __import__(dep)
            print(f"   ✅ {dep} (core)")
        except ImportError:
            print(f"   ❌ {dep} (core) - REQUIRED")
            return False
    
    for dep in optional_deps:
        try:
            __import__(dep)
            print(f"   ✅ {dep} (optional)")
        except ImportError:
            print(f"   ⚠️ {dep} (optional) - not installed")
    
    return True

def generate_upload_summary():
    """生成上传摘要"""
    print("\n📊 Project Summary:")
    print("=" * 50)
    
    # 文件统计
    py_files = list(Path('.').rglob('*.py'))
    md_files = list(Path('.').rglob('*.md'))
    
    print(f"📁 Python files: {len(py_files)}")
    print(f"📄 Documentation files: {len(md_files)}")
    
    # 代码行数统计
    total_lines = 0
    for py_file in py_files:
        if '__pycache__' not in str(py_file):
            try:
                with open(py_file, 'r', encoding='utf-8') as f:
                    total_lines += len(f.readlines())
            except:
                pass
    
    print(f"📏 Total lines of Python code: ~{total_lines}")
    
    # 功能特性
    print(f"\n🚀 Key Features:")
    print(f"   • Multi-format asset support (URDF, MJCF, USD, Genesis)")
    print(f"   • Physics parameter validation")
    print(f"   • Mesh processing pipeline")
    print(f"   • Metadata management system")
    print(f"   • CLI interface with {len(['convert', 'validate', 'preview', 'mesh', 'meta', 'register'])} commands")
    print(f"   • Cross-platform compatibility")
    print(f"   • Modular dependency system")

def main():
    """主检查函数"""
    print("🧩 AssetX Project Upload Readiness Check")
    print("=" * 50)
    
    checks = [
        check_project_structure(),
        check_git_status(), 
        check_installation(),
        check_dependencies()
    ]
    
    if all(checks):
        print("\n🎉 Project is ready for GitHub upload!")
        generate_upload_summary()
        print("\n📋 Next steps:")
        print("1. Create repository on GitHub")
        print("2. Add remote: git remote add origin https://github.com/YOUR_USERNAME/AssetX.git")
        print("3. Push: git branch -M main && git push -u origin main")
        print("4. Check GITHUB_SETUP.md for detailed instructions")
    else:
        print("\n❌ Project has issues that need to be resolved before upload.")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
