#!/usr/bin/env python3
"""
简单测试USD风格Asset接口
"""

from assetx.core.asset import Asset
from pathlib import Path
import tempfile

def simple_test():
    # 创建临时URDF文件用于测试
    urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" iyy="1.0" izz="1.0" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
  </link>
  <link name="arm_link">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
  </link>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>"""
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(urdf_content)
        urdf_path = Path(f.name)
    
    try:
        print("=== USD风格Asset测试 ===")
        
        # 1. 创建Asset
        asset = Asset(urdf_path)
        print(f"✓ 创建Asset: {asset}")
        
        # 2. 获取Stage (USD核心概念)
        stage = asset.get_stage()
        print(f"✓ 获取Stage: {stage}")
        
        # 3. 获取所有Prim
        all_prims = asset.get_all_prims()
        print(f"✓ 总共有 {len(all_prims)} 个Prim:")
        for prim in all_prims:
            print(f"  - {prim.path} (类型: {prim.type_name or 'None'})")
        
        # 4. 通过路径查找Prim
        base_link = asset.get_prim_at_path("/test_robot/base_link")
        if base_link:
            print(f"✓ 找到base_link: {base_link.path}")
            
            # 5. 获取属性
            props = base_link.get_properties()
            print(f"✓ base_link有 {len(props)} 个属性:")
            for prop in props:
                print(f"  - {prop.name}: {prop.get()} (类型: {prop.type_name})")
        
        # 6. 查找所有Link类型的Prim
        links = asset.find_prims_by_type("Link")
        joints = asset.find_prims_by_type("Joint")
        print(f"✓ 找到 {len(links)} 个Link, {len(joints)} 个Joint")
        
        print("\n=== 测试成功! USD风格接口工作正常 ===")
        
    finally:
        # 清理临时文件
        try:
            urdf_path.unlink()
        except:
            pass

if __name__ == "__main__":
    simple_test()
