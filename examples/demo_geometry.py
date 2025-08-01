#!/usr/bin/env python3
"""
演示AssetX的USD风格几何体处理功能

展示如何使用新的geometry模块进行：
- 创建几何体Prim
- 加载网格文件
- 处理和优化网格
- 渲染可视化
"""

from pathlib import Path
import tempfile

from assetx.core import Asset
from assetx.core.primitives import SdfPath


def demo_geometry_processing():
    """演示几何体处理功能"""
    print("=== AssetX USD风格几何体处理演示 ===\n")
    
    # 1. 创建测试URDF文件
    urdf_content = """<?xml version="1.0"?>
<robot name="geometry_demo_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
  </link>
  
  <link name="arm_link">
    <visual>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm_visual.dae"/>
      </geometry>
    </visual>
  </link>
</robot>"""

    temp_file = Path(tempfile.mktemp(suffix='.urdf'))
    temp_file.write_text(urdf_content)
    
    try:
        # 2. 创建Asset并加载
        print("1. 创建Asset:")
        asset = Asset(temp_file)
        asset.load()
        print(f"   ✓ Asset加载成功: {asset.asset_path.name}")
        print(f"   ✓ Geometry模块: {type(asset.geometry).__name__}")
        print(f"   ✓ 支持状态: Mesh={asset.geometry.has_mesh_support()}, Render={asset.geometry.has_render_support()}")
        
        # 3. 创建USD风格的几何体Prim
        print("\n2. 创建USD风格几何体:")
        
        # 创建根变换节点
        robot_xform = asset.geometry.create_xform_prim("/geometry_demo_robot", "robot_root")
        print(f"   ✓ 机器人根变换节点: {robot_xform}")
        
        # 创建base_link的几何体
        base_visual = asset.geometry.create_mesh_prim("/geometry_demo_robot/base_link/visual", "base_visual")
        base_visual.set_box_size([0.2, 0.2, 0.1])
        print(f"   ✓ Base link visual: {base_visual}")
        print(f"     - 几何类型: {base_visual.get_geometry_type()}")
        
        base_collision = asset.geometry.create_mesh_prim("/geometry_demo_robot/base_link/collision", "base_collision")
        base_collision.set_box_size([0.2, 0.2, 0.1])
        print(f"   ✓ Base link collision: {base_collision}")
        
        # 创建arm_link的几何体（mesh文件）
        arm_visual = asset.geometry.create_mesh_prim("/geometry_demo_robot/arm_link/visual", "arm_visual")
        # 模拟mesh文件路径
        mesh_path = Path("arm_visual.dae")
        arm_visual.set_mesh_file(mesh_path)
        print(f"   ✓ Arm link visual: {arm_visual}")
        print(f"     - Mesh文件: {arm_visual.get_mesh_file()}")
        
        # 4. 设置变换
        print("\n3. 设置几何体变换:")
        
        # 设置机器人根变换
        robot_xform.set_translation([0.0, 0.0, 0.5])
        robot_xform.set_rotation([0.0, 0.0, 45.0])  # 45度旋转
        print(f"   ✓ 机器人变换: {robot_xform}")
        
        # 创建arm变换节点
        arm_xform = asset.geometry.create_xform_prim("/geometry_demo_robot/arm_link", "arm_transform")
        arm_xform.set_translation([0.0, 0.0, 0.2])
        arm_xform.set_rotation([90.0, 0.0, 0.0])
        print(f"   ✓ 手臂变换: {arm_xform}")
        
        # 5. 几何体信息统计
        print("\n4. 几何体信息统计:")
        
        # 验证几何体
        validation_result = asset.geometry.validate_geometry()
        print(f"   ✓ 总网格数量: {validation_result['total_meshes']}")
        print(f"   ✓ 总变换数量: {validation_result['total_xforms']}")
        print(f"   ✓ 有效网格: {len(validation_result['valid_meshes'])}")
        print(f"   ✓ 无效网格: {len(validation_result['invalid_meshes'])}")
        
        # 获取详细信息
        for prim in asset.stage.traverse():
            if hasattr(prim, 'type_name') and prim.type_name in ['Mesh', 'Xform']:
                info = asset.geometry.get_geometry_info(prim)
                print(f"   ✓ {info['type']} @ {info['path']}")
                if 'geometry_type' in info:
                    print(f"     - 几何类型: {info['geometry_type']}")
                if 'translation' in info:
                    print(f"     - 位置: {info['translation']}")
                    
        # 6. 支持的格式
        print("\n5. 支持的文件格式:")
        supported_formats = asset.geometry.get_supported_formats()
        if supported_formats:
            print(f"   ✓ 支持格式: {', '.join(supported_formats)}")
        else:
            print("   ⚠️ 未检测到MeshProcessor，无格式支持信息")
            
        # 7. 演示API使用
        print("\n6. USD风格API演示:")
        print("   # 创建网格几何体")
        print("   mesh = asset.geometry.create_mesh_prim('/robot/link/visual')")
        print("   mesh.set_box_size([0.1, 0.1, 0.1])")
        print("   mesh.set_mesh_file('model.obj')")
        print("")
        print("   # 创建变换节点")
        print("   xform = asset.geometry.create_xform_prim('/robot/link')")
        print("   xform.set_translation([1.0, 0.0, 0.0])")
        print("   xform.set_rotation([0.0, 0.0, 90.0])")
        print("")
        print("   # 渲染几何体")
        print("   asset.geometry.render_mesh(mesh)")
        print("   asset.geometry.render_link('base_link')")
        
        print("\n🎉 USD风格几何体处理演示完成!")
        print("\n总结:")
        print("✅ 实现了USD风格的几何体架构")
        print("✅ 支持AssetMesh和AssetXform对象")
        print("✅ 集成了MeshProcessor和Previewer功能") 
        print("✅ 提供了统一的几何体处理接口")
        print("✅ 兼容USD的属性和变换系统")
        
        return True
        
    except Exception as e:
        print(f"❌ 演示过程中出错: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # 清理临时文件
        if temp_file.exists():
            temp_file.unlink()


if __name__ == "__main__":
    demo_geometry_processing()
