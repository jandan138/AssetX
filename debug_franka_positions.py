#!/usr/bin/env python3
"""
调试Franka USD位置提取
"""

import logging
import sys
import os

# 设置日志
logging.basicConfig(level=logging.DEBUG, format='%(levelname)s:%(name)s:%(message)s')

# 添加AssetX到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

def debug_franka_positions():
    """调试Franka链接位置提取"""
    try:
        from assetx.core.asset import Asset
        
        print("=== 调试Franka USD位置提取 ===")
        
        # 加载USD文件
        usd_file = r"c:\Users\24494\Downloads\franka.usda"
        asset = Asset(usd_file)
        asset.load()
        
        # 获取链接
        links = asset.query.get_links()
        print(f"找到 {len(links)} 个链接")
        
        # 调试每个链接的属性和位置
        for i, link in enumerate(links[:5]):  # 只调试前5个
            print(f"\n--- 链接 {i+1}: {link.name} ---")
            print(f"路径: {link.path}")
            print(f"类型: {link.type_name}")
            
            # 显示所有属性
            if hasattr(link, 'get_properties'):
                props = link.get_properties()
                print(f"属性数量: {len(props)}")
                for prop in props:
                    try:
                        value = prop.get()
                        print(f"  {prop.name}: {value} (类型: {type(value)})")
                    except:
                        print(f"  {prop.name}: <无法获取值>")
            
            # 尝试提取位置
            from assetx.core.geometry.robot_visualizer import RobotVisualizer
            visualizer = RobotVisualizer(asset)
            position = visualizer._extract_real_position(link)
            print(f"提取的位置: {position}")
        
        return True
        
    except Exception as e:
        print(f"❌ 调试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    debug_franka_positions()
