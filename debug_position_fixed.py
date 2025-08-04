#!/usr/bin/env python3
"""
调试Franka机器人位置提取
"""

import logging
from assetx.core.asset import Asset

# 设置日志级别为INFO以查看详细调试信息
logging.basicConfig(level=logging.INFO, format='%(levelname)s:%(name)s:%(message)s')

def debug_franka_positions():
    """调试Franka机器人位置信息"""
    print("🔍 开始调试Franka机器人位置信息...")
    
    # 加载Franka USD文件
    asset = Asset("franka.usda")
    
    # 获取机器人可视化器，传入asset参数
    from assetx.core.geometry.robot_visualizer import RobotVisualizer
    visualizer = RobotVisualizer(asset)
    
    # 获取所有链接 - 使用正确的属性名
    links = asset.query.get_links()
    print(f"\n📋 找到 {len(links)} 个链接:")
    
    for link in links:
        print(f"\n🔗 链接: {link.name}")
        
        # 检查原始属性
        if hasattr(link, 'properties'):
            if 'xformOp:translate' in link.properties:
                translate_attr = link.properties['xformOp:translate']
                print(f"   USD原始 xformOp:translate: {translate_attr}")
            else:
                print(f"   ❌ 没有 xformOp:translate 属性")
                print(f"   可用属性: {list(link.properties.keys())}")
        
        # 使用可视化器提取位置
        extracted_pos = visualizer._extract_real_position(link)
        print(f"   🎯 提取的位置: {extracted_pos}")

if __name__ == "__main__":
    debug_franka_positions()
