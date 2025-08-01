#!/usr/bin/env python3
"""
展示USD风格Asset的所有核心功能
"""

import sys
import os
from pathlib import Path
import tempfile

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from assetx.core.asset import Asset
from assetx.core.sdf_path import SdfPath

def demo_usd_features():
    print("=== USD风格Asset系统完整功能演示 ===")
    
    # 创建URDF测试文件
    urdf_content = """<?xml version="1.0"?>
<robot name="demo_robot">
  <link name="base_link">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.8" iyy="0.8" izz="0.6" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </visual>
  </link>
  
  <link name="shoulder_link">
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.4" iyy="0.4" izz="0.3" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
  </link>
  
  <link name="arm_link">
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.2" iyy="0.2" izz="0.15" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
  </link>
  
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2"/>
  </joint>
  
  <joint name="arm_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="arm_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="1"/>
  </joint>
</robot>"""
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(urdf_content)
        urdf_path = Path(f.name)
    
    try:
        # 1. 基本Asset操作
        print("\n1. 基本Asset操作:")
        asset = Asset(urdf_path)
        print(f"   ✓ 创建Asset: {asset}")
        print(f"   ✓ 格式检测: {asset.format}")
        print(f"   ✓ 是否已加载: {asset.is_loaded}")
        
        # 2. Stage操作
        print("\n2. Stage操作:")
        stage = asset.get_stage()
        print(f"   ✓ 获取Stage: {stage.identifier}")
        
        default_prim = stage.get_default_prim()
        if default_prim:
            print(f"   ✓ 默认Prim: {default_prim.path}")
        
        # 3. 层次结构遍历
        print("\n3. 层次结构遍历:")
        root_prims = asset.query.get_root_prims()
        print(f"   ✓ 根Prim数量: {len(root_prims)}")
        
        def print_hierarchy(prim, indent=0):
            spaces = "  " * indent
            type_info = f" ({prim.type_name})" if prim.type_name else ""
            print(f"   {spaces}- {prim.name}{type_info}")
            for child in prim.get_children():
                print_hierarchy(child, indent + 1)
        
        for root in root_prims:
            print_hierarchy(root)
        
        # 4. SdfPath操作演示
        print("\n4. SdfPath操作演示:")
        demo_path = SdfPath("/demo_robot/base_link")
        print(f"   ✓ 路径字符串: {demo_path}")
        print(f"   ✓ 父路径: {demo_path.get_parent_path()}")
        print(f"   ✓ 节点名: {demo_path.get_name()}")
        print(f"   ✓ 路径开始于根: {demo_path.path_string.startswith('/')}")
        
        child_path = demo_path.append_child("child_element")
        print(f"   ✓ 追加子路径: {child_path}")
        
        # 5. 属性操作
        print("\n5. 属性操作演示:")
        base_link = asset.get_prim_at_path("/demo_robot/base_link")
        if base_link:
            print(f"   ✓ 找到base_link: {base_link.path}")
            
            # 获取所有属性
            all_props = base_link.get_properties()
            print(f"   ✓ 属性总数: {len(all_props)}")
            
            # 分别获取Attribute和Relationship
            attributes = base_link.get_attributes()
            relationships = base_link.get_relationships()
            print(f"   ✓ Attribute数量: {len(attributes)}")
            print(f"   ✓ Relationship数量: {len(relationships)}")
            
            # 显示属性详情
            for attr in attributes:
                print(f"     - {attr.name}: {attr.get()} (类型: {attr.type_name})")
            
            for rel in relationships:
                targets = rel.get_targets()
                print(f"     - {rel.name}: {len(targets)} 个目标")
        
        # 6. 类型查询
        print("\n6. 类型查询:")
        links = asset.query.find_prims_by_type("Link")
        joints = asset.query.find_prims_by_type("Joint")
        robots = asset.query.find_prims_by_type("Robot")
        
        print(f"   ✓ Robot数量: {len(robots)}")
        print(f"   ✓ Link数量: {len(links)}")
        print(f"   ✓ Joint数量: {len(joints)}")
        
        # 7. Schema应用演示
        print("\n7. Schema应用演示:")
        if base_link:
            schemas = base_link.get_applied_schemas()
            print(f"   ✓ base_link应用的Schema: {schemas}")
            
            # 检查是否应用了特定API
            has_physics = base_link.has_api_schema("PhysicsAPI")
            has_link = base_link.has_api_schema("LinkAPI")
            print(f"   ✓ 是否有PhysicsAPI: {has_physics}")
            print(f"   ✓ 是否有LinkAPI: {has_link}")
        
        # 8. 元数据操作
        print("\n8. 元数据操作:")
        if base_link:
            # 设置元数据
            base_link.set_metadata("description", "Base link of the robot")
            base_link.set_metadata("version", "1.0")
            
            # 获取元数据
            desc = base_link.get_metadata("description")
            version = base_link.get_metadata("version")
            print(f"   ✓ 描述: {desc}")
            print(f"   ✓ 版本: {version}")
        
        # 9. 遍历操作
        print("\n9. 遍历操作:")
        all_prims = asset.query.traverse()
        physics_prims = asset.query.traverse(lambda p: p.has_api_schema("PhysicsAPI"))
        
        print(f"   ✓ 总Prim数量: {len(all_prims)}")
        print(f"   ✓ 具有PhysicsAPI的Prim: {len(physics_prims)}")
        
        # 10. 总结
        print("\n10. 系统总结:")
        print(f"   ✓ 成功创建了完整的USD风格Asset管理系统")
        print(f"   ✓ 支持USD核心概念: Stage, Prim, Property, SdfPath")
        print(f"   ✓ 支持层次结构、属性管理、Schema应用")
        print(f"   ✓ 兼容URDF格式并可扩展到其他格式")
        print(f"   ✓ 提供了丰富的查询和遍历接口")
        
        print("\n=== USD风格Asset系统演示完成! ===")
        
    finally:
        # 清理临时文件
        try:
            urdf_path.unlink()
        except:
            pass

if __name__ == "__main__":
    demo_usd_features()
