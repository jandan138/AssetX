#!/usr/bin/env python3
"""
测试修复后的机器人可视化
"""

import logging
from assetx.core.asset import Asset

# 设置日志级别为INFO
logging.basicConfig(level=logging.INFO, format='%(levelname)s:%(name)s:%(message)s')

def test_fixed_visualization():
    """测试修复后的机器人可视化"""
    print("🔍 测试修复后的Franka机器人可视化...")
    
    # 加载Franka USD文件
    asset = Asset("franka.usda")
    
    # 获取机器人可视化器
    from assetx.core.geometry.robot_visualizer import RobotVisualizer
    visualizer = RobotVisualizer(asset)
    
    # 可视化机器人 - 使用正确的方法名
    print("\n🎨 开始可视化...")
    visualizer.render_complete_robot()
    print("✅ 可视化完成")

if __name__ == "__main__":
    test_fixed_visualization()
