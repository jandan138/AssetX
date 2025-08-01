#!/usr/bin/env python3
"""
测试Asset渲染功能
"""

import sys
from pathlib import Path

# 添加当前目录到路径
sys.path.insert(0, str(Path(__file__).parent))

def main():
    try:
        print("=== AssetX 渲染测试 ===")
        
        # 导入必要模块
        print("1. 导入AssetX模块...")
        from assetx.core import Asset
        print("   ✓ Asset导入成功")
        
        # 创建测试文件
        print("2. 创建测试URDF文件...")
        temp = Path('test_render.urdf')
        temp.write_text('''<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>''')
        print("   ✓ 测试文件已创建")
        
        # 创建Asset对象
        print("3. 创建Asset对象...")
        asset = Asset(temp)
        print("   ✓ Asset对象已创建")
        
        # 加载Asset
        print("4. 加载Asset...")
        asset.load()
        print("   ✓ Asset已加载")
        
        # 检查geometry模块
        print("5. 检查geometry模块...")
        print(f"   - geometry模块: {asset.geometry}")
        print(f"   - 渲染支持: {asset.geometry.has_render_support()}")
        print(f"   - 网格支持: {asset.geometry.has_mesh_support()}")
        
        # 创建测试mesh
        print("6. 创建测试网格...")
        mesh = asset.geometry.create_mesh_prim('/test/box', 'test_box')
        print(f"   ✓ 网格已创建: {mesh}")
        
        # 设置网格参数
        print("7. 设置网格参数...")
        mesh.set_box_size([2.0, 1.0, 0.5])
        print("   ✓ 参数已设置")
        
        # 尝试渲染
        print("8. 开始渲染...")
        result = asset.geometry.render_mesh(mesh, show_axes=True)
        print(f"   渲染结果: {'成功' if result else '失败'}")
        
        # 清理
        temp.unlink()
        print("9. 清理完成")
        
        return result
        
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        
        # 确保清理
        if 'temp' in locals() and temp.exists():
            temp.unlink()
        
        return False

if __name__ == "__main__":
    success = main()
    print(f"\n=== 测试{'成功' if success else '失败'} ===")
