#!/usr/bin/env python3
"""
测试完整机器人渲染
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

def test_complete_robot():
    try:
        print("=== 完整机器人渲染测试 ===")
        
        from assetx.core import Asset
        
        # 创建机器人URDF
        robot_urdf = Path('complete_robot.urdf')
        robot_urdf.write_text('''<?xml version="1.0"?>
<robot name="complete_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
  
  <link name="arm1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <link name="arm2">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>
  
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="base_to_arm1" type="revolute">
    <parent link="base_link"/>
    <child link="arm1"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <joint name="arm1_to_arm2" type="revolute">
    <parent link="arm1"/>
    <child link="arm2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="arm2_to_end" type="fixed">
    <parent link="arm2"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>''')
        
        print("1. 加载机器人...")
        robot = Asset(robot_urdf)
        robot.load()
        print(f"   ✓ 机器人已加载: {robot}")
        
        print("2. 分析机器人结构...")
        links = robot.query.get_links()
        joints = robot.query.get_joints()
        print(f"   - 链接数: {len(links)}")
        print(f"   - 关节数: {len(joints)}")
        
        print("3. 渲染完整机器人...")
        print("   正在生成完整机器人可视化...")
        
        # 渲染完整机器人
        result = robot.geometry.render_complete_robot(show_joints=True, show_frames=True)
        
        if result:
            print("   🎉 完整机器人渲染成功！")
            print("   📊 包含内容：")
            print("     - 蓝色基座 (base_link)")
            print("     - 红色手臂1 (arm1)")  
            print("     - 绿色手臂2 (arm2)")
            print("     - 黄色末端执行器 (end_effector)")
            print("     - 黑色虚线关节连接")
            print("     - RGB坐标系框架")
        else:
            print("   ❌ 完整机器人渲染失败")
        
        robot_urdf.unlink()
        print("4. 清理完成")
        
        return result
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        if 'robot_urdf' in locals() and robot_urdf.exists():
            robot_urdf.unlink()
        return False

if __name__ == "__main__":
    success = test_complete_robot()
    print(f"\n=== 完整机器人测试{'成功' if success else '失败'} ===")
    
    if success:
        print("\n🚀 现在你可以使用以下方式渲染完整机器人：")
        print("robot.geometry.render_complete_robot()")
        print("robot.geometry.render_complete_robot(show_joints=True, show_frames=True)")
