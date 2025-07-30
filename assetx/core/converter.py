"""
Format conversion utilities
"""

import logging
from pathlib import Path
from typing import Optional, Union

from .asset import Asset

logger = logging.getLogger(__name__)


class FormatConverter:
    """资产格式转换器"""

    def __init__(self):
        self.supported_conversions = {
            "urdf": ["mjcf", "usd"],
            "mjcf": ["urdf", "usd"],
            "usd": ["urdf", "mjcf", "genesis"],
            "genesis": [],  # Genesis 主要作为目标格式
        }

    def can_convert(self, from_format: str, to_format: str) -> bool:
        """检查是否支持指定格式转换"""
        return to_format in self.supported_conversions.get(from_format, [])

    def convert(
        self,
        source_path: Union[str, Path],
        target_format: str,
        output_path: Optional[Union[str, Path]] = None,
    ) -> Path:
        """执行格式转换"""

        source_path = Path(source_path)
        if not source_path.exists():
            raise FileNotFoundError(f"Source file not found: {source_path}")

        # 加载源资产
        asset = Asset(source_path)
        asset.load()

        source_format = asset.format

        if not self.can_convert(source_format, target_format):
            raise ValueError(
                f"Conversion from {source_format} to {target_format} not supported"
            )

        # 生成输出路径
        if output_path is None:
            output_path = source_path.with_suffix(self._get_extension(target_format))
        else:
            output_path = Path(output_path)

        # 执行转换
        if source_format == "urdf" and target_format == "mjcf":
            self._urdf_to_mjcf(asset, output_path)
        elif source_format == "mjcf" and target_format == "urdf":
            self._mjcf_to_urdf(asset, output_path)
        elif source_format == "urdf" and target_format == "usd":
            self._urdf_to_usd(asset, output_path)
        elif source_format == "usd" and target_format == "genesis":
            self._usd_to_genesis(asset, output_path)
        else:
            raise NotImplementedError(
                f"Conversion {source_format} -> {target_format} not implemented yet"
            )

        logger.info(
            f"Converted {source_path} ({source_format}) to {output_path} ({target_format})"
        )
        return output_path

    def _get_extension(self, format_name: str) -> str:
        """获取格式对应的文件扩展名"""
        extensions = {
            "urdf": ".urdf",
            "mjcf": ".xml",
            "usd": ".usd",
            "genesis": ".json",
        }
        return extensions.get(format_name, ".txt")

    def _urdf_to_mjcf(self, asset: Asset, output_path: Path) -> None:
        """URDF转MJCF"""
        try:
            import xml.etree.ElementTree as ET

            # 创建MJCF根元素
            mujoco = ET.Element("mujoco", model=asset.asset_path.stem)

            # 添加编译器设置
            compiler = ET.SubElement(mujoco, "compiler", angle="radian")

            # 添加worldbody
            worldbody = ET.SubElement(mujoco, "worldbody")

            # 转换链接和关节
            for link_name, link_data in asset.links.items():
                body = ET.SubElement(worldbody, "body", name=link_name)

                # 添加几何体
                if "visual" in link_data and link_data["visual"]:
                    visual = link_data["visual"][0]  # 取第一个视觉元素
                    if hasattr(visual, "geometry") and hasattr(visual.geometry, "mesh"):
                        geom = ET.SubElement(
                            body,
                            "geom",
                            type="mesh",
                            mesh=visual.geometry.mesh.filename,
                        )

                # 添加惯性属性
                if link_name in asset.physics_properties:
                    props = asset.physics_properties[link_name]
                    if props.mass is not None:
                        inertial = ET.SubElement(body, "inertial", mass=str(props.mass))

            # 保存文件
            tree = ET.ElementTree(mujoco)
            tree.write(output_path, encoding="utf-8", xml_declaration=True)

        except Exception as e:
            raise RuntimeError(f"URDF to MJCF conversion failed: {e}")

    def _mjcf_to_urdf(self, asset: Asset, output_path: Path) -> None:
        """MJCF转URDF"""
        # TODO: 实现MJCF到URDF的转换
        raise NotImplementedError("MJCF to URDF conversion not implemented yet")

    def _urdf_to_usd(self, asset: Asset, output_path: Path) -> None:
        """URDF转USD"""
        # TODO: 实现URDF到USD的转换
        raise NotImplementedError("URDF to USD conversion not implemented yet")

    def _usd_to_genesis(self, asset: Asset, output_path: Path) -> None:
        """USD转Genesis JSON"""
        # TODO: 实现USD到Genesis的转换
        raise NotImplementedError("USD to Genesis conversion not implemented yet")
