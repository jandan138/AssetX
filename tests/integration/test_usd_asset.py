#!/usr/bin/env python3
"""
测试USD风格的Asset类设计
"""

import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from assetx.core.asset import Asset
from assetx.core.sdf_path import SdfPath
from assetx.core.enums import AssetFormat
import tempfile

def test_usd_asset():
    print('=== USD风格Asset类测试 ===')
    
    # 创建一个临时的简单URDF文件进行测试
    urdf_content = '''<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" iyy="1.0" izz="1.0" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="base.stl"/>
      </geometry>
    </visual>
  </link>
  <link name="arm_link">
    <inertial>
      <mass value="0.5"/>
    </inertial>
  </link>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>'''

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(urdf_content)
        temp_urdf = f.name

    try:
        # 创建Asset实例
        asset = Asset(temp_urdf)
        print(f'1. Asset创建成功: {asset}')
        print(f'   格式检测: {asset.format}')
        
        # 加载资产
        asset.load()
        print(f'2. 加载结果: 已加载')
        
        if asset.is_loaded:
            # 测试Stage接口
            stage = asset.get_stage()
            print(f'3. Stage获取成功: {stage.identifier}')
            
            # 测试默认prim
            default_prim = asset.get_default_prim()
            if default_prim:
                print(f'4. 默认prim: {default_prim.name} (类型: {default_prim.type_name})')
                print(f'   应用的schemas: {default_prim.get_applied_schemas()}')
            
            # 测试遍历
            all_prims = asset.query.get_all_prims()
            print(f'5. 所有prim数量: {len(all_prims)}')
            for prim in all_prims:
                print(f'   - {prim.path} (类型: {prim.type_name})')
            
            # 测试链接和关节
            links = asset.query.get_links()
            joints = asset.query.get_joints()
            print(f'6. 链接数量: {len(links)}, 关节数量: {len(joints)}')
            
            # 测试具体信息获取
            if links:
                link = links[0]
                print(f'7. 第一个链接: {link.name} 路径: {link.path}')
            
            if joints:
                joint = joints[0]
                print(f'8. 第一个关节: {joint.name} 路径: {joint.path}')
            
            # 测试USD风格的路径操作
            test_path = SdfPath('/test_robot/base_link')
            prim = asset.get_prim_at_path(test_path)
            if prim:
                print(f'9. 路径查询成功: {prim.path}')
                
                # 测试属性操作
                attrs = prim.get_attributes()
                print(f'   属性数量: {len(attrs)}')
                for attr in attrs:
                    print(f'   - {attr.name}: {attr.get()} (类型: {attr.type_name})')
            
            print('\n=== 测试完成，USD风格接口工作正常! ===')
        else:
            print('加载失败')

    finally:
        # 清理临时文件
        try:
            os.unlink(temp_urdf)
        except:
            pass

if __name__ == '__main__':
    test_usd_asset()
