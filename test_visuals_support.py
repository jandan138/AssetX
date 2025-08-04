#!/usr/bin/env python3
"""
测试USD Visuals几何提取功能

验证新增的visuals子节点支持是否正常工作。
"""

import sys
from pathlib import Path

# 添加项目根目录到Python路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from assetx.core.asset import Asset
from assetx.core.modules.queries import AssetQuery
from assetx.core.geometry.robot_visualizer import RobotVisualizer
import logging

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_visuals_extraction():
    """测试visuals几何提取"""
    print("=== 测试USD Visuals几何提取 ===")
    
    franka_usd_path = r"c:\Users\24494\Downloads\franka.usda"
    
    try:
        # 创建Asset并加载
        asset = Asset(franka_usd_path)
        asset.load()
        
        # 创建机器人可视化器
        visualizer = RobotVisualizer(asset)
        
        # 获取链接
        query = AssetQuery(asset)
        links = query.get_links()
        
        print(f"找到 {len(links)} 个链接")
        
        # 测试每个链接的几何提取
        geometry_results = {}
        
        for link in links[:3]:  # 只测试前3个链接
            print(f"\n--- 测试链接: {link.name} ---")
            
            # 测试新的几何提取方法
            geometry_info = visualizer._extract_geometry_info(link)
            
            if geometry_info:
                geometry_results[link.name] = geometry_info
                print(f"✅ 几何类型: {geometry_info['type']}")
                print(f"   数据源: {geometry_info.get('source', '未知')}")
                
                if geometry_info['type'] == 'mesh':
                    if 'reference_path' in geometry_info:
                        print(f"   外部引用: {geometry_info['reference_path']}")
                    if 'mesh_file' in geometry_info:
                        print(f"   网格文件: {geometry_info['mesh_file']}")
                
                elif geometry_info['type'] == 'box':
                    print(f"   尺寸: {geometry_info.get('size', 'N/A')}")
                
                elif geometry_info['type'] == 'cylinder':
                    print(f"   半径: {geometry_info.get('radius', 'N/A')}")
                    print(f"   高度: {geometry_info.get('height', 'N/A')}")
                
                elif geometry_info['type'] == 'sphere':
                    print(f"   半径: {geometry_info.get('radius', 'N/A')}")
                
                # 显示材质信息
                if 'material' in geometry_info:
                    material = geometry_info['material']
                    print(f"   材质颜色: {material.get('color', 'N/A')}")
                    print(f"   粗糙度: {material.get('roughness', 'N/A')}")
            else:
                print("❌ 未找到几何信息")
                
                # 尝试显示链接的子节点信息
                if hasattr(link, 'get_children'):
                    children = link.get_children()
                    if children:
                        print(f"   子节点: {[child.name for child in children]}")
                    else:
                        print("   没有子节点")
        
        # 统计结果
        print(f"\n=== 几何提取统计 ===")
        print(f"成功提取: {len(geometry_results)} / {min(3, len(links))}")
        
        geometry_types = {}
        for info in geometry_results.values():
            geom_type = info['type']
            if geom_type not in geometry_types:
                geometry_types[geom_type] = 0
            geometry_types[geom_type] += 1
        
        print("几何类型分布:")
        for geom_type, count in geometry_types.items():
            print(f"  {geom_type}: {count} 个")
        
        return len(geometry_results) > 0
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_full_robot_visualization():
    """测试完整机器人可视化"""
    print("\n=== 测试完整机器人可视化 ===")
    
    franka_usd_path = r"c:\Users\24494\Downloads\franka.usda"
    
    try:
        asset = Asset(franka_usd_path)
        asset.load()
        
        visualizer = RobotVisualizer(asset)
        
        # 尝试渲染完整机器人
        success = visualizer.render_complete_robot(
            show_joints=True,
            show_frames=True,
            auto_scale=True
        )
        
        if success:
            print("✅ 机器人可视化成功")
            print("   检查是否显示了matplotlib窗口")
        else:
            print("❌ 机器人可视化失败")
        
        return success
        
    except Exception as e:
        print(f"❌ 可视化测试失败: {e}")
        return False


def main():
    """运行所有测试"""
    print("🚀 USD Visuals几何提取测试")
    print("=" * 50)
    
    tests = [
        ("几何提取测试", test_visuals_extraction),
        ("完整可视化测试", test_full_robot_visualization)
    ]
    
    passed = 0
    
    for test_name, test_func in tests:
        print(f"\n🧪 {test_name}")
        try:
            if test_func():
                passed += 1
                print(f"✅ {test_name} 通过")
            else:
                print(f"❌ {test_name} 失败")
        except Exception as e:
            print(f"❌ {test_name} 异常: {e}")
    
    print("\n" + "=" * 50)
    print(f"📊 测试结果: {passed}/{len(tests)} 通过")
    
    if passed == len(tests):
        print("🎉 所有测试通过！")
        return True
    else:
        print(f"⚠️  {len(tests) - passed} 个测试失败")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
