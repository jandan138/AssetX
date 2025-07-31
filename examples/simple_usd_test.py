#!/usr/bin/env python3
"""
简单的USD加载测试
"""

from assetx import Asset
from pathlib import Path
import tempfile

def simple_usd_test():
    """简单USD测试"""
    print("🧪 AssetX USD简单测试")
    print("=" * 40)
    
    # 创建有效的USD文件
    usd_content = '''#usda 1.0

def "robot" (
    kind = "component"
)
{
    def "base_link" {
        custom float physics:mass = 1.0
    }
    
    def "arm_link" {
        custom float physics:mass = 0.5
    }
    
    def "joint1" {
        custom string joint:type = "revolute"
    }
}
'''
    
    # 写入临时文件
    with tempfile.NamedTemporaryFile(mode='w', suffix='.usd', delete=False, encoding='utf-8') as f:
        f.write(usd_content)
        usd_path = Path(f.name)
    
    try:
        print(f"📂 创建USD文件: {usd_path.name}")
        
        # 创建Asset
        asset = Asset(usd_path)
        print(f"✅ Asset创建成功: {asset.format}")
        
        # 加载Asset
        asset.load()
        print(f"✅ Asset加载成功")
        
        # 获取基本信息
        prim_count = len(asset.get_all_prims())
        print(f"✅ 找到 {prim_count} 个Prim")
        
        # 显示资产信息
        info = asset.get_asset_info()
        print(f"📊 资产信息:")
        for key, value in info.items():
            print(f"   {key}: {value}")
            
        print(f"\n🎉 USD加载测试成功!")
        return True
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # 清理
        if usd_path.exists():
            usd_path.unlink()

def check_environment():
    """检查环境"""
    print("🔍 环境检查")
    print("=" * 20)
    
    # 检查USD
    try:
        from pxr import Usd
        print(f"✅ USD库: {Usd.GetVersion()}")
    except ImportError:
        print("❌ USD库未安装")
        return False
    
    # 检查AssetX
    try:
        from assetx import Asset
        print("✅ AssetX可用")
    except ImportError:
        print("❌ AssetX不可用")
        return False
    
    return True

if __name__ == "__main__":
    if check_environment():
        simple_usd_test()
    else:
        print("❌ 环境检查失败")
