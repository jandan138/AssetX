#!/usr/bin/env python3
"""
Example usage of AssetX library
"""

from pathlib import Path
from assetx import Asset, FormatConverter, PhysicsValidator, MeshProcessor, MetaManager, Previewer


def main():
    """演示 AssetX 的主要功能"""
    
    print("🧩 AssetX Example Usage")
    print("=" * 50)
    
    # 1. 加载资产
    print("\n1. Loading Asset...")
    # 这里假设有一个示例 URDF 文件
    # asset_path = Path("examples/panda_arm.urdf")
    # asset = Asset(asset_path)
    # asset.load()
    # print(f"   Loaded {asset.format} asset with {len(asset.links)} links")
    print("   (示例文件未提供，跳过实际加载)")
    
    # 2. 格式转换
    print("\n2. Format Conversion...")
    converter = FormatConverter()
    print(f"   Supported conversions: {converter.supported_conversions}")
    # output_path = converter.convert("panda.urdf", "mjcf")
    # print(f"   Converted to: {output_path}")
    print("   (示例：可将 URDF 转换为 MJCF、USD 等格式)")
    
    # 3. 物理验证
    print("\n3. Physics Validation...")
    validator = PhysicsValidator(tolerance=1e-6)
    print("   Validator initialized with tolerance: 1e-6")
    # result = validator.validate_asset(asset)
    # print(f"   Validation result: {result.is_valid}")
    print("   (可验证质量、惯性矩阵等物理参数)")
    
    # 4. 网格处理
    print("\n4. Mesh Processing...")
    mesh_processor = MeshProcessor()
    print(f"   Supported formats: {mesh_processor.supported_formats}")
    # simplified_mesh = mesh_processor.simplify_mesh("mesh.obj", target_faces=1000)
    # centered_mesh = mesh_processor.center_mesh("mesh.obj")
    print("   (可简化、中心化、缩放网格文件)")
    
    # 5. 元数据管理
    print("\n5. Metadata Management...")
    meta_manager = MetaManager(Path("."))
    
    # 创建示例资产元数据
    asset_meta = meta_manager.create_asset_meta(
        asset_name="example_robot.urdf",
        original_format="urdf",
        semantic_category="robot_arm",
        description="Example robot for demonstration",
        tags=["example", "demo", "7dof"]
    )
    print(f"   Created meta for: {asset_meta['name']}")
    print(f"   Category: {asset_meta['semantic_category']}")
    print(f"   Tags: {asset_meta['tags']}")
    
    # 6. 可视化预览
    print("\n6. Asset Preview...")
    try:
        previewer = Previewer(backend='trimesh')
        print("   Previewer initialized with trimesh backend")
        # previewer.preview_mesh("mesh.obj")
        print("   (可预览 URDF、网格文件等)")
    except ImportError as e:
        print(f"   预览功能需要额外依赖: {e}")
    
    print("\n" + "=" * 50)
    print("✅ AssetX 功能演示完成!")
    print("\n💡 使用提示:")
    print("   - 使用 CLI: assetx convert --source robot.urdf --to mjcf")
    print("   - 安装依赖: pip install urdfpy mujoco trimesh")
    print("   - 查看帮助: assetx --help")


if __name__ == "__main__":
    main()
