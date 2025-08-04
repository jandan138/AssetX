#!/usr/bin/env python3
"""
简化的USD内容测试 - 专注于核心功能，减少干扰输出

此测试专门验证USD文件的基本加载和内容获取，
忽略外部引用警告，聚焦于我们能控制的核心功能。
"""

import os
import sys
import logging
from pathlib import Path

# 添加项目根目录到Python路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from assetx.core.asset import Asset
from assetx.core.modules.queries import AssetQuery

# 设置日志级别，减少USD的警告输出
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def test_usd_basic_loading():
    """测试USD文件基本加载"""
    print("=== 测试1: USD文件基本加载 ===")
    
    franka_usd_path = r"c:\Users\24494\Downloads\franka.usda"
    
    if not os.path.exists(franka_usd_path):
        print(f"❌ USD文件不存在: {franka_usd_path}")
        return False
    
    try:
        # 创建和加载Asset
        asset = Asset(franka_usd_path)
        asset.load()
        
        print(f"✅ 成功加载USD文件")
        print(f"   文件路径: {franka_usd_path}")
        print(f"   加载状态: {asset.is_loaded}")
        print(f"   资产格式: {asset.format}")
        
        # 获取Stage
        stage = asset.get_stage()
        if stage:
            print(f"   Stage对象: 已创建")
            
            # 获取默认Prim
            default_prim = asset.get_default_prim()
            if default_prim:
                print(f"   默认Prim: {default_prim.name}")
            
        return True
        
    except Exception as e:
        print(f"❌ 加载失败: {e}")
        return False


def test_usd_content_query():
    """测试USD内容查询"""
    print("\n=== 测试2: USD内容查询 ===")
    
    franka_usd_path = r"c:\Users\24494\Downloads\franka.usda"
    
    try:
        asset = Asset(franka_usd_path)
        asset.load()
        query = AssetQuery(asset)
        
        # 获取链接
        links = query.get_links()
        print(f"✅ 找到链接数量: {len(links)}")
        
        if links:
            print("   链接列表:")
            for i, link in enumerate(links[:5], 1):  # 只显示前5个
                print(f"     {i}. {link.name} (路径: {link.path})")
            if len(links) > 5:
                print(f"     ... 还有 {len(links) - 5} 个链接")
        
        # 获取关节
        joints = query.get_joints()
        print(f"✅ 找到关节数量: {len(joints)}")
        
        if joints:
            print("   关节列表:")
            for i, joint in enumerate(joints[:5], 1):  # 只显示前5个
                print(f"     {i}. {joint.name} (路径: {joint.path})")
            if len(joints) > 5:
                print(f"     ... 还有 {len(joints) - 5} 个关节")
        
        # 验证Franka特定结构
        link_names = [link.name for link in links]
        joint_names = [joint.name for joint in joints]
        
        franka_links = [name for name in link_names if 'panda_link' in name]
        franka_joints = [name for name in joint_names if 'panda_joint' in name]
        
        print(f"✅ Franka链接: {len(franka_links)} 个")
        print(f"   {franka_links}")
        print(f"✅ Franka关节: {len(franka_joints)} 个")
        print(f"   {franka_joints}")
        
        return len(links) > 0 and len(joints) > 0
        
    except Exception as e:
        print(f"❌ 查询失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_usd_link_properties():
    """测试USD链接属性提取"""
    print("\n=== 测试3: USD链接属性 ===")
    
    franka_usd_path = r"c:\Users\24494\Downloads\franka.usda"
    
    try:
        asset = Asset(franka_usd_path)
        asset.load()
        query = AssetQuery(asset)
        
        links = query.get_links()
        if not links:
            print("❌ 没有找到链接")
            return False
        
        # 检查第一个链接的属性
        first_link = links[0]
        print(f"✅ 分析链接: {first_link.name}")
        
        # 使用正确的方法获取属性
        try:
            properties = first_link.get_properties()
            if properties:
                print(f"   属性数量: {len(properties)}")
                
                # 查找关键属性
                key_attrs = ['xformOp:translate', 'xformOp:orient', 'physics:mass']
                found_attrs = []
                
                property_names = first_link.get_property_names()
                print(f"   属性名称: {property_names[:5]}...")  # 显示前5个
                
                for attr_name in key_attrs:
                    if first_link.has_property(attr_name):
                        found_attrs.append(attr_name)
                        prop = first_link.get_property(attr_name)
                        if prop and hasattr(prop, 'get'):
                            value = prop.get()
                            print(f"   {attr_name}: {value}")
                        else:
                            print(f"   {attr_name}: (无法获取值)")
                
                print(f"✅ 找到关键属性: {len(found_attrs)} 个")
                return len(properties) > 0
            else:
                print("   没有找到任何属性")
                return False
        except Exception as e:
            print(f"   属性访问异常: {e}")
            # 尝试备用方法
            try:
                property_names = first_link.get_property_names()
                print(f"   备用方法 - 属性名称数量: {len(property_names)}")
                return len(property_names) > 0
            except Exception as e2:
                print(f"   备用方法也失败: {e2}")
                return False
            
    except Exception as e:
        print(f"❌ 属性提取失败: {e}")
        return False


def test_usd_hierarchy():
    """测试USD层次结构"""
    print("\n=== 测试4: USD层次结构 ===")
    
    franka_usd_path = r"c:\Users\24494\Downloads\franka.usda"
    
    try:
        asset = Asset(franka_usd_path)
        asset.load()
        query = AssetQuery(asset)
        
        all_prims = query.get_all_prims()
        print(f"✅ 总原语数量: {len(all_prims)}")
        
        # 按类型分类
        prim_types = {}
        for prim in all_prims:
            prim_type = getattr(prim, 'type_name', '未知')
            if prim_type not in prim_types:
                prim_types[prim_type] = []
            prim_types[prim_type].append(prim.name)
        
        print("   原语类型分布:")
        for prim_type, names in prim_types.items():
            print(f"     {prim_type}: {len(names)} 个")
            if len(names) <= 3:
                print(f"       {names}")
            else:
                print(f"       {names[:3]} ... (还有{len(names)-3}个)")
        
        return len(all_prims) > 0
        
    except Exception as e:
        print(f"❌ 层次结构分析失败: {e}")
        return False


def main():
    """运行所有测试"""
    print("🚀 AssetX USD文件内容测试")
    print("=" * 50)
    
    # 抑制USD的外部引用警告
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
    
    tests = [
        test_usd_basic_loading,
        test_usd_content_query,
        test_usd_link_properties,
        test_usd_hierarchy
    ]
    
    passed = 0
    total = len(tests)
    
    for test_func in tests:
        try:
            if test_func():
                passed += 1
        except Exception as e:
            print(f"❌ 测试 {test_func.__name__} 异常: {e}")
    
    print("\n" + "=" * 50)
    print(f"📊 测试结果: {passed}/{total} 通过")
    
    if passed == total:
        print("🎉 所有测试通过！USD文件加载和内容获取功能正常")
        return True
    else:
        print(f"⚠️  {total - passed} 个测试失败")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
