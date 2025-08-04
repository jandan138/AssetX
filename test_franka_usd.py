#!/usr/bin/env python3
"""
测试加载和显示Franka USD文件
"""

import logging
import sys
import os

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(levelname)s:%(name)s:%(message)s')

# 添加AssetX到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

def test_franka_usd():
    """测试加载和显示Franka USD文件"""
    try:
        from assetx.core.asset import Asset
        from assetx.core.geometry.robot_visualizer import RobotVisualizer
        
        print("=== 测试Franka USD加载和显示 ===")
        
        # 指定USD文件路径
        usd_file = r"c:\Users\24494\Downloads\franka.usda"
        
        if not os.path.exists(usd_file):
            print(f"❌ USD文件不存在: {usd_file}")
            return False
            
        print(f"1. 加载USD文件: {usd_file}")
        
        # 创建Asset对象并加载USD文件
        asset = Asset(usd_file)
        asset.load()
        print(f"   ✓ USD文件加载成功: {asset}")
        
        print("2. 分析机器人结构...")
        
        # 获取所有Prim来调试
        all_prims = asset.query.get_all_prims()
        print(f"   - 总Prim数: {len(all_prims)}")
        
        # 显示所有Prim的类型信息
        type_count = {}
        print("   - 前20个Prim信息:")
        for i, prim in enumerate(all_prims[:20]):
            print(f"     {i+1}. {prim.name} (类型: {prim.type_name}, 路径: {prim.path})")
            type_count[prim.type_name] = type_count.get(prim.type_name, 0) + 1
            
        print(f"   - Prim类型统计: {type_count}")
        
        # 获取链接和关节信息
        links = asset.query.get_links()
        joints = asset.query.get_joints()
        
        print(f"   - 链接数: {len(links)}")
        print(f"   - 关节数: {len(joints)}")
        
        # 尝试查找所有Xform类型的Prim（USD中通常链接是Xform类型）
        xforms = asset.query.find_prims_by_type("Xform")
        print(f"   - Xform数: {len(xforms)}")
        if xforms:
            print("   - Xform列表:")
            for i, xform in enumerate(xforms[:10]):  # 只显示前10个
                print(f"     {i+1}. {xform.name}")
                
        # 查找所有关节类型的Prim
        physics_joints = asset.query.find_prims_by_type("PhysicsRevoluteJoint")
        physics_prismatic = asset.query.find_prims_by_type("PhysicsPrismaticJoint")
        physics_fixed = asset.query.find_prims_by_type("PhysicsFixedJoint")
        
        print(f"   - PhysicsRevoluteJoint数: {len(physics_joints)}")
        print(f"   - PhysicsPrismaticJoint数: {len(physics_prismatic)}")
        print(f"   - PhysicsFixedJoint数: {len(physics_fixed)}")
        
        all_physics_joints = physics_joints + physics_prismatic + physics_fixed
        
        if links or xforms:
            print("3. 创建机器人可视化器...")
            visualizer = RobotVisualizer(asset)
            
            print("4. 渲染机器人...")
            # 如果没有找到标准的链接，尝试使用Xform作为链接
            test_links = links if links else xforms
            test_joints = joints if joints else all_physics_joints
            
            print(f"   使用 {len(test_links)} 个链接和 {len(test_joints)} 个关节进行渲染")
            
            success = visualizer.render_complete_robot(
                show_joints=True,
                show_frames=True,
                auto_scale=True
            )
            
            if success:
                print("   🎉 Franka机器人渲染成功！")
                print("   📊 显示内容：")
                print("     - 各种颜色的机器人链接")
                print("     - 黑色虚线关节连接")
                print("     - RGB坐标系框架")
                print("     - 3D立体几何形状")
            else:
                print("   ❌ 机器人渲染失败")
        else:
            print("   ❌ 没有找到链接或Xform，无法渲染机器人")
            success = False
            
        return success
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    test_franka_usd()
