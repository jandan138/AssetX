#!/usr/bin/env python3
"""
AssetX USD → URDF 转换功能演示

展示如何使用 AssetX 将 USD 机器人文件转换为 URDF 格式。
"""

import sys
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent))

from assetx.core.converter import FormatConverter
from assetx.core.asset import Asset

def demo_usd_to_urdf_conversion():
    """演示 USD → URDF 转换功能"""
    
    print("🎯 AssetX USD → URDF 转换演示")
    print("=" * 60)
    
    # 示例文件路径
    usd_file = r"c:\Users\24494\Downloads\franka.usda" 
    urdf_file = "converted_robot.urdf"
    
    print(f"📁 输入文件: {usd_file}")
    print(f"📁 输出文件: {urdf_file}")
    
    if not Path(usd_file).exists():
        print(f"❌ USD 文件不存在: {usd_file}")
        print(f"💡 请将您的 USD 文件路径更新到脚本中")
        return False
    
    try:
        # 创建转换器
        converter = FormatConverter()
        
        print(f"\n🔄 开始转换...")
        
        # 执行转换
        output_path = converter.convert(
            source_path=usd_file,
            target_format="urdf",
            output_path=urdf_file
        )
        
        print(f"✅ 转换完成！")
        print(f"📄 URDF 文件已生成: {output_path}")
        
        # 验证文件
        if Path(output_path).exists():
            file_size = Path(output_path).stat().st_size
            print(f"📏 文件大小: {file_size:,} bytes")
            
            # 简单分析生成的 URDF
            import xml.etree.ElementTree as ET
            tree = ET.parse(output_path)
            root = tree.getroot()
            
            robot_name = root.get('name', 'Unknown')
            links = root.findall('link')
            joints = root.findall('joint')
            
            print(f"\n📊 URDF 文件分析:")
            print(f"   🤖 机器人名称: {robot_name}")
            print(f"   🔗 链接数量: {len(links)}")
            print(f"   🔧 关节数量: {len(joints)}")
            
            if links:
                print(f"   📝 链接列表: {[link.get('name') for link in links[:5]]}")
            if joints:
                print(f"   📝 关节列表: {[joint.get('name') for joint in joints[:5]]}")
                
            print(f"\n🎉 转换成功！URDF 文件可以用于:")
            print(f"   • ROS/ROS2 机器人仿真")
            print(f"   • MoveIt! 运动规划")
            print(f"   • Gazebo 物理仿真")
            print(f"   • 其他 URDF 兼容工具")
            
        return True
        
    except Exception as e:
        print(f"❌ 转换失败: {e}")
        return False

def show_supported_conversions():
    """显示支持的转换格式"""
    print(f"\n🔧 AssetX 支持的格式转换:")
    print("=" * 60)
    
    converter = FormatConverter()
    
    conversions = [
        ("URDF", "MJCF", "✅ 已实现"),
        ("USD", "URDF", "✅ 已实现"),
        ("MJCF", "URDF", "🚧 开发中"),
        ("URDF", "USD", "🚧 开发中"),
        ("USD", "Genesis", "🚧 开发中"),
    ]
    
    print(f"{'源格式':<10} → {'目标格式':<10} | {'状态'}")
    print("-" * 35)
    
    for source, target, status in conversions:
        print(f"{source:<10} → {target:<10} | {status}")

if __name__ == "__main__":
    # 演示转换功能
    success = demo_usd_to_urdf_conversion()
    
    # 显示支持的转换
    show_supported_conversions()
    
    if success:
        print(f"\n🎊 演示完成！AssetX 现在支持 USD → URDF 转换功能。")
    else:
        print(f"\n💡 请确保有可用的 USD 文件进行测试。")
