"""
Command line interface for AssetX
"""

import logging
import time
from pathlib import Path
from typing import Optional

import click

from . import __version__
from .core.asset import Asset
from .core.converter import FormatConverter
from .core.validator import PhysicsValidator
from .mesh.processor import MeshProcessor
from .meta.manager import MetaManager
from .viewer.preview import Previewer

# 设置日志
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


@click.group()
@click.version_option(version=__version__)
@click.option("--verbose", "-v", is_flag=True, help="Enable verbose logging")
def main(verbose):
    """AssetX - Multi-format robot simulation asset bridge tool"""
    if verbose:
        logging.getLogger().setLevel(logging.DEBUG)


@main.command()
@click.option(
    "--source",
    "-s",
    required=True,
    type=click.Path(exists=True),
    help="Source asset file path",
)
@click.option(
    "--to",
    "target_format",
    required=True,
    type=click.Choice(["urdf", "mjcf", "usd", "genesis"]),
    help="Target format",
)
@click.option("--output", "-o", type=click.Path(), help="Output file path (optional)")
@click.option(
    "--update-meta",
    is_flag=True,
    default=True,
    help="Update meta.yaml with conversion record",
)
def convert(source, target_format, output, update_meta):
    """Convert asset between different formats"""
    try:
        source_path = Path(source)
        converter = FormatConverter()

        start_time = time.time()
        output_path = converter.convert(source_path, target_format, output)
        conversion_time = time.time() - start_time

        click.echo(f"✅ Conversion completed successfully!")
        click.echo(f"   Source: {source_path}")
        click.echo(f"   Output: {output_path}")
        click.echo(f"   Time: {conversion_time:.2f}s")

        # 更新元数据
        if update_meta:
            meta_manager = MetaManager(source_path.parent)
            meta_manager.add_conversion_record(
                asset_name=source_path.name,
                target_format=target_format,
                output_path=str(output_path),
                conversion_time=conversion_time,
                success=True,
            )
            click.echo(f"   Meta data updated")

    except Exception as e:
        click.echo(f"❌ Conversion failed: {e}", err=True)
        if update_meta:
            try:
                meta_manager = MetaManager(Path(source).parent)
                meta_manager.add_conversion_record(
                    asset_name=Path(source).name,
                    target_format=target_format,
                    output_path="",
                    conversion_time=0,
                    success=False,
                    notes=str(e),
                )
            except:
                pass
        raise click.ClickException(str(e))


@main.command()
@click.option(
    "--path",
    "-p",
    required=True,
    type=click.Path(exists=True),
    help="Asset file path to preview",
)
@click.option(
    "--backend",
    default="trimesh",
    type=click.Choice(["trimesh", "open3d"]),
    help="Visualization backend",
)
@click.option("--axes/--no-axes", default=True, help="Show coordinate axes")
@click.option("--wireframe/--no-wireframe", default=False, help="Show wireframe")
def preview(path, backend, axes, wireframe):
    """Preview asset visualization"""
    try:
        asset_path = Path(path)
        previewer = Previewer(backend=backend)

        # 检测文件类型
        if asset_path.suffix.lower() == ".urdf":
            previewer.preview_urdf(asset_path)
        else:
            previewer.preview_mesh(asset_path, show_axes=axes, show_wireframe=wireframe)

    except Exception as e:
        click.echo(f"❌ Preview failed: {e}", err=True)
        raise click.ClickException(str(e))


@main.command()
@click.option(
    "--ref",
    required=True,
    type=click.Path(exists=True),
    help="Reference asset file path",
)
@click.option(
    "--target",
    type=click.Path(exists=True),
    help="Target asset file to compare (optional)",
)
@click.option(
    "--tolerance", default=1e-6, type=float, help="Numerical tolerance for comparisons"
)
@click.option(
    "--update-meta",
    is_flag=True,
    default=True,
    help="Update meta.yaml with validation results",
)
def validate(ref, target, tolerance, update_meta):
    """Validate asset physics parameters"""
    try:
        ref_path = Path(ref)
        validator = PhysicsValidator(tolerance=tolerance)

        # 加载参考资产
        ref_asset = Asset(ref_path)
        ref_asset.load()

        if target:
            # 比较两个资产
            target_path = Path(target)
            target_asset = Asset(target_path)
            target_asset.load()

            result = validator.compare_assets(ref_asset, target_asset)

            click.echo(f"🔍 Comparison Results:")
            click.echo(f"   Reference: {ref_path}")
            click.echo(f"   Target: {target_path}")

        else:
            # 验证单个资产
            result = validator.validate_asset(ref_asset)

            click.echo(f"🔍 Validation Results:")
            click.echo(f"   Asset: {ref_path}")

        # 显示结果
        summary = result.get_summary()

        if summary["is_valid"]:
            click.echo("   ✅ Validation passed")
        else:
            click.echo("   ❌ Validation failed")

        if summary["warning_count"] > 0:
            click.echo(f"   ⚠️ Warnings: {summary['warning_count']}")
            for warning in summary["warnings"]:
                click.echo(f"     - {warning}")

        if summary["error_count"] > 0:
            click.echo(f"   ❌ Errors: {summary['error_count']}")
            for error in summary["errors"]:
                click.echo(f"     - {error}")

        # 更新元数据
        if update_meta:
            meta_manager = MetaManager(ref_path.parent)
            validation_type = "comparison" if target else "physics"
            meta_manager.add_validation_record(
                asset_name=ref_path.name,
                validation_type=validation_type,
                result=summary,
                compared_with=Path(target).name if target else None,
            )
            click.echo(f"   Meta data updated")

    except Exception as e:
        click.echo(f"❌ Validation failed: {e}", err=True)
        raise click.ClickException(str(e))


@main.command()
@click.option(
    "--path", "-p", required=True, type=click.Path(exists=True), help="Mesh file path"
)
@click.option(
    "--operation",
    "-op",
    required=True,
    type=click.Choice(["simplify", "center", "scale", "collision", "info"]),
    help="Mesh operation to perform",
)
@click.option(
    "--output",
    "-o",
    type=click.Path(),
    help="Output file path (for operations that modify mesh)",
)
@click.option(
    "--target-faces",
    type=int,
    default=1000,
    help="Target face count for simplification",
)
@click.option(
    "--scale-factor", type=float, default=1.0, help="Scale factor for scaling operation"
)
@click.option(
    "--collision-method",
    default="convex_hull",
    type=click.Choice(["convex_hull", "bounding_box", "simplified"]),
    help="Method for collision mesh generation",
)
def mesh(path, operation, output, target_faces, scale_factor, collision_method):
    """Process mesh files"""
    try:
        mesh_path = Path(path)
        processor = MeshProcessor()

        if operation == "info":
            info = processor.get_mesh_info(mesh_path)
            click.echo(f"📊 Mesh Information:")
            click.echo(f"   Path: {info['path']}")
            click.echo(f"   Vertices: {info.get('vertices', 'N/A')}")
            click.echo(f"   Faces: {info.get('faces', 'N/A')}")
            click.echo(f"   Volume: {info.get('volume', 'N/A'):.6f}")
            click.echo(f"   Surface Area: {info.get('surface_area', 'N/A'):.6f}")
            click.echo(f"   Watertight: {info.get('is_watertight', 'N/A')}")

        elif operation == "simplify":
            output_path = processor.simplify_mesh(mesh_path, target_faces, output)
            click.echo(f"✅ Mesh simplified to {target_faces} faces")
            click.echo(f"   Output: {output_path}")

        elif operation == "center":
            output_path = processor.center_mesh(mesh_path, output)
            click.echo(f"✅ Mesh centered to origin")
            click.echo(f"   Output: {output_path}")

        elif operation == "scale":
            output_path = processor.scale_mesh(mesh_path, scale_factor, output)
            click.echo(f"✅ Mesh scaled by factor {scale_factor}")
            click.echo(f"   Output: {output_path}")

        elif operation == "collision":
            output_path = processor.generate_collision_mesh(
                mesh_path, collision_method, output
            )
            click.echo(f"✅ Collision mesh generated using {collision_method}")
            click.echo(f"   Output: {output_path}")

    except Exception as e:
        click.echo(f"❌ Mesh processing failed: {e}", err=True)
        raise click.ClickException(str(e))


@main.command()
@click.option(
    "--dir",
    "-d",
    default=".",
    type=click.Path(exists=True),
    help="Asset directory path",
)
@click.option("--category", "-c", help="Filter by semantic category")
@click.option("--tag", "-t", help="Filter by tag")
@click.option("--format", "-f", help="Filter by format")
@click.option("--export", "-e", type=click.Path(), help="Export report to file")
def meta(dir, category, tag, format, export):
    """Manage asset metadata"""
    try:
        asset_dir = Path(dir)
        meta_manager = MetaManager(asset_dir)

        if export:
            report_path = meta_manager.export_meta_report(export)
            click.echo(f"📊 Meta report exported to: {report_path}")
        else:
            assets = meta_manager.list_assets(category, tag, format)

            if not assets:
                click.echo("No assets found matching the criteria")
                return

            click.echo(f"📋 Found {len(assets)} assets:")
            click.echo()

            for asset in assets:
                click.echo(f"  • {asset['name']}")
                click.echo(f"    Category: {asset.get('category', 'N/A')}")
                click.echo(f"    Format: {asset.get('format', 'N/A')}")
                click.echo(f"    Tags: {', '.join(asset.get('tags', []))}")
                click.echo(
                    f"    Physics Validated: {'✅' if asset.get('physics_validated') else '❌'}"
                )
                if asset.get("description"):
                    click.echo(f"    Description: {asset['description']}")
                click.echo()

    except Exception as e:
        click.echo(f"❌ Meta operation failed: {e}", err=True)
        raise click.ClickException(str(e))


@main.command()
@click.option("--name", required=True, help="Asset name")
@click.option(
    "--path", required=True, type=click.Path(exists=True), help="Asset file path"
)
@click.option(
    "--category",
    required=True,
    help="Semantic category (e.g., robot_arm, mobile_robot, gripper)",
)
@click.option("--description", default="", help="Asset description")
@click.option("--tags", help="Comma-separated tags")
def register(name, path, category, description, tags):
    """Register a new asset in meta.yaml"""
    try:
        asset_path = Path(path)
        asset = Asset(asset_path)

        # 解析标签
        tag_list = [t.strip() for t in tags.split(",")] if tags else []

        # 创建资产元数据
        meta_manager = MetaManager(asset_path.parent)
        asset_meta = meta_manager.create_asset_meta(
            asset_name=name,
            original_format=asset.format,
            semantic_category=category,
            description=description,
            tags=tag_list,
        )

        # 更新元数据
        meta_manager.update_asset_meta(name, asset_meta)

        click.echo(f"✅ Asset registered successfully!")
        click.echo(f"   Name: {name}")
        click.echo(f"   Path: {asset_path}")
        click.echo(f"   Category: {category}")
        click.echo(f"   Format: {asset.format}")
        if tag_list:
            click.echo(f"   Tags: {', '.join(tag_list)}")

    except Exception as e:
        click.echo(f"❌ Registration failed: {e}", err=True)
        raise click.ClickException(str(e))


if __name__ == "__main__":
    main()
