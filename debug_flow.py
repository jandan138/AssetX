#!/usr/bin/env python3
"""
测试位置计算流程的详细调试
"""

import logging
from assetx.core.asset import Asset

# 设置最详细的日志级别
logging.basicConfig(level=logging.DEBUG, format='%(levelname)s:%(name)s:%(message)s')

def debug_position_flow():
    """调试位置计算的完整流程"""
    print("🔍 调试位置计算完整流程...")
    
    # 加载Franka USD文件
    asset = Asset("franka.usda")
    
    # 获取机器人可视化器
    from assetx.core.geometry.robot_visualizer import RobotVisualizer
    visualizer = RobotVisualizer(asset)
    
    # 获取链接和关节
    links = asset.query.get_links()
    joints = asset.query.get_joints()
    
    print(f"\n📋 找到 {len(links)} 个链接和 {len(joints)} 个关节")
    
    # 直接调用位置计算方法
    print("\n🎯 调用 _calculate_real_link_positions...")
    positions = visualizer._calculate_real_link_positions(links, joints)
    
    print(f"\n📍 最终计算的位置:")
    for name, pos in positions.items():
        print(f"   {name}: {pos}")

if __name__ == "__main__":
    debug_position_flow()
