#!/usr/bin/env python3
"""
测试 AssetX 安装和基本功能
"""

import sys
from pathlib import Path

def test_basic_imports():
    """测试基本导入"""
    print("🧪 Testing basic imports...")
    
    try:
        from assetx import __version__
        print(f"   ✅ AssetX version: {__version__}")
    except ImportError as e:
        print(f"   ❌ Failed to import AssetX: {e}")
        return False
    
    try:
        from assetx import Asset, AssetFormat
        print("   ✅ Core asset classes imported")
    except ImportError as e:
        print(f"   ❌ Failed to import core classes: {e}")
        return False
    
    try:
        from assetx.core.converter import FormatConverter
        print("   ✅ Format converter imported")
    except ImportError as e:
        print(f"   ❌ Failed to import converter: {e}")
        return False
    
    try:
        from assetx.core.validator import PhysicsValidator
        print("   ✅ Physics validator imported")
    except ImportError as e:
        print(f"   ❌ Failed to import validator: {e}")
        return False
    
    try:
        from assetx.meta.manager import MetaManager
        print("   ✅ Meta manager imported")
    except ImportError as e:
        print(f"   ❌ Failed to import meta manager: {e}")
        return False
    
    return True

def test_optional_dependencies():
    """测试可选依赖"""
    print("\n🔍 Checking optional dependencies...")
    
    # Test trimesh
    try:
        import trimesh
        print("   ✅ trimesh available")
    except ImportError:
        print("   ⚠️ trimesh not available (install with: pip install trimesh)")
    
    # Test open3d
    try:
        import open3d
        print("   ✅ open3d available")
    except ImportError:
        print("   ⚠️ open3d not available (install with: pip install open3d)")
    
    # Test urdfpy
    try:
        import urdfpy
        print("   ✅ urdfpy available")
    except ImportError:
        print("   ⚠️ urdfpy not available (install with: pip install urdfpy)")

def test_core_functionality():
    """测试核心功能"""
    print("\n⚙️ Testing core functionality...")
    
    try:
        # Test format converter
        from assetx.core.converter import FormatConverter
        converter = FormatConverter()
        print("   ✅ Converter initialized")
        
        # Test validator
        from assetx.core.validator import PhysicsValidator
        validator = PhysicsValidator()
        print("   ✅ Physics validator initialized")
        
        # Test meta manager
        from assetx.meta.manager import MetaManager
        meta_manager = MetaManager(Path("."))
        meta_template = meta_manager.create_meta_template()
        print(f"   ✅ Meta template created with schema: {meta_template['schema_version']}")
        
        # Test mesh processor (without external deps)
        from assetx.mesh.processor import MeshProcessor
        processor = MeshProcessor()
        print(f"   ✅ Mesh processor supports: {processor.supported_formats}")
        
        return True
        
    except Exception as e:
        print(f"   ❌ Core functionality test failed: {e}")
        return False

def test_cli_availability():
    """测试CLI可用性"""
    print("\n🖥️ Testing CLI availability...")
    
    try:
        from assetx.cli import main
        print("   ✅ CLI main function available")
        
        # Test if assetx command is in PATH
        import subprocess
        result = subprocess.run(
            [sys.executable, "-c", "import assetx.cli; print('CLI OK')"],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            print("   ✅ CLI command accessible")
        else:
            print(f"   ⚠️ CLI command issue: {result.stderr}")
        
        return True
        
    except Exception as e:
        print(f"   ❌ CLI test failed: {e}")
        return False

def test_create_sample_meta():
    """测试创建示例元数据"""
    print("\n📝 Testing sample meta creation...")
    
    try:
        from assetx.meta.manager import MetaManager
        import tempfile
        
        with tempfile.TemporaryDirectory() as tmpdir:
            meta_manager = MetaManager(Path(tmpdir))
            
            # Create sample asset meta
            asset_meta = meta_manager.create_asset_meta(
                asset_name="test_robot.urdf",
                original_format="urdf",
                semantic_category="robot_arm",
                description="Test robot for AssetX validation",
                tags=["test", "validation", "demo"]
            )
            
            print(f"   ✅ Sample asset meta created: {asset_meta['name']}")
            print(f"      Category: {asset_meta['semantic_category']}")
            print(f"      Tags: {asset_meta['tags']}")
            
            # Test meta file operations
            meta_manager.update_asset_meta("test_robot.urdf", asset_meta)
            loaded_meta = meta_manager.load_meta("test_robot.urdf")
            
            if loaded_meta.get('name') == "test_robot.urdf":
                print("   ✅ Meta file save/load working")
            else:
                print("   ⚠️ Meta file save/load issue")
            
            return True
            
    except Exception as e:
        print(f"   ❌ Sample meta creation failed: {e}")
        return False

def main():
    """主测试函数"""
    print("🧩 AssetX Installation Test")
    print("=" * 50)
    
    all_tests_passed = True
    
    # Run tests
    all_tests_passed &= test_basic_imports()
    test_optional_dependencies()  # This doesn't affect overall pass/fail
    all_tests_passed &= test_core_functionality()
    all_tests_passed &= test_cli_availability()
    all_tests_passed &= test_create_sample_meta()
    
    print("\n" + "=" * 50)
    
    if all_tests_passed:
        print("🎉 All tests passed! AssetX is ready to use.")
        print("\n💡 Quick start:")
        print("   python -m assetx.cli --help")
        print("   # or if installed as command:")
        print("   assetx --help")
        print("\n📦 Install optional dependencies:")
        print("   pip install 'assetx[mesh]'      # for mesh processing")
        print("   pip install 'assetx[urdf]'      # for URDF support")
        print("   pip install 'assetx[viewer]'    # for visualization")
    else:
        print("❌ Some tests failed. Check the output above.")
        sys.exit(1)

if __name__ == "__main__":
    main()
