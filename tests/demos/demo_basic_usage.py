#!/usr/bin/env python3
"""基本用法演示

展示AssetX库的基本使用方法。
"""

import tempfile
from pathlib import Path

# 导入AssetX模块
from assetx import Asset, AssetFormat

def create_sample_urdf():
    """创建示例URDF文件"""
    urdf_content = """<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <link name="link1">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>
  
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>"""
    
    # 创建临时文件
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    temp_file.write(urdf_content)
    temp_file.close()
    return temp_file.name

def main():
    """主演示函数"""
    print("=== AssetX 基本用法演示 ===\n")
    
    # 1. 创建示例文件
    print("1. 创建示例URDF文件...")
    urdf_path = create_sample_urdf()
    print(f"   ✓ 创建文件: {urdf_path}")
    
    # 2. 创建Asset对象
    print("\n2. 创建Asset对象...")
    asset = Asset(urdf_path)
    print(f"   ✓ Asset创建成功")
    print(f"   ✓ 文件路径: {asset.asset_path}")
    print(f"   ✓ 格式: {asset.format}")
    print(f"   ✓ 是否已加载: {asset.is_loaded}")
    
    # 3. 加载资产
    print("\n3. 加载资产...")
    asset.load()
    print(f"   ✓ 加载完成")
    print(f"   ✓ 是否已加载: {asset.is_loaded}")
    
    # 4. 获取Stage信息
    print("\n4. Stage信息...")
    stage = asset.get_stage()
    print(f"   ✓ Stage 标识符: {stage._identifier}")
    
    # 5. 获取默认Prim
    print("\n5. 默认Prim...")
    default_prim = asset.get_default_prim()
    if default_prim:
        print(f"   ✓ 默认Prim: {default_prim.path}")
        print(f"   ✓ 类型名: {default_prim.type_name}")
    
    # 6. 遍历所有Prim
    print("\n6. 遍历所有Prim...")
    root_prims = asset.get_root_prims()
    print(f"   ✓ 根Prim数量: {len(root_prims)}")
    
    def print_prim_tree(prim, indent=0):
        """递归打印Prim树"""
        prefix = "  " * indent + "- "
        print(f"{prefix}{prim.path.get_name()} ({prim.type_name})")
        for child in prim.get_children():
            print_prim_tree(child, indent + 1)
    
    for prim in root_prims:
        print_prim_tree(prim)
    
    # 7. 查询特定类型的Prim
    print("\n7. 查询特定类型...")
    links = asset.find_prims_by_type("Link")
    joints = asset.find_prims_by_type("Joint")
    print(f"   ✓ Link数量: {len(links)}")
    print(f"   ✓ Joint数量: {len(joints)}")
    
    # 8. 属性操作示例
    print("\n8. 属性操作...")
    if links:
        link = links[0]
        properties = link.get_properties()
        print(f"   ✓ {link.path.get_name()} 的属性数量: {len(properties)}")
        for prop in properties[:3]:  # 只显示前3个
            value = prop.get()
            print(f"     - {prop.name}: {value} (类型: {prop.type_name})")
    
    # 9. 清理
    print("\n9. 清理...")
    import os
    os.unlink(urdf_path)
    print("   ✓ 临时文件已删除")
    
    print("\n=== 演示完成! ===")

if __name__ == "__main__":
    main()
