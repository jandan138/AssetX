#!/usr/bin/env python3
"""
URDF 格式加载器
"""

import xml.etree.ElementTree as ET
from typing import TYPE_CHECKING

from .base import BaseAssetLoader

if TYPE_CHECKING:
    from ..asset import Asset


class UrdfLoader(BaseAssetLoader):
    """URDF格式加载器"""
    
    def can_load(self) -> bool:
        """检查是否为URDF文件"""
        return self.asset_path.suffix.lower() == '.urdf'
    
    def load(self, asset: "Asset") -> None:
        """加载URDF格式"""
        try:
            tree = ET.parse(self.asset_path)
            root = tree.getroot()

            # 获取机器人名称
            robot_name = root.get("name", "robot")

            # 创建机器人根Prim
            robot_prim = asset.create_robot_prim(f"/{robot_name}", robot_name)
            stage = self._get_stage(asset)
            stage.set_default_prim(robot_prim)

            # 加载链接
            for link_elem in root.findall("link"):
                self._load_link(link_elem, robot_name, asset)

            # 加载关节
            for joint_elem in root.findall("joint"):
                self._load_joint(joint_elem, robot_name, asset)

        except Exception as e:
            raise ValueError(f"Failed to load URDF: {e}")

    def _load_link(self, link_elem: ET.Element, robot_name: str, asset: "Asset") -> None:
        """加载URDF链接"""
        link_name = link_elem.get("name")
        link_path = f"/{robot_name}/{link_name}"

        # 提取质量和惯性
        mass = None
        inertia = None

        inertial_elem = link_elem.find("inertial")
        if inertial_elem is not None:
            mass_elem = inertial_elem.find("mass")
            if mass_elem is not None:
                mass = float(mass_elem.get("value", 0))

            inertia_elem = inertial_elem.find("inertia")
            if inertia_elem is not None:
                inertia = [
                    float(inertia_elem.get("ixx", 0)),
                    float(inertia_elem.get("iyy", 0)),
                    float(inertia_elem.get("izz", 0)),
                    float(inertia_elem.get("ixy", 0)),
                    float(inertia_elem.get("ixz", 0)),
                    float(inertia_elem.get("iyz", 0)),
                ]

        # 创建链接Prim
        asset.create_link_prim(link_path, mass, inertia)

    def _load_joint(self, joint_elem: ET.Element, robot_name: str, asset: "Asset") -> None:
        """加载URDF关节"""
        joint_name = joint_elem.get("name")
        joint_type = joint_elem.get("type", "fixed")
        joint_path = f"/{robot_name}/{joint_name}"

        # 获取父子链接
        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")

        parent_link = parent_elem.get("link") if parent_elem is not None else None
        child_link = child_elem.get("link") if child_elem is not None else None

        # 获取轴向
        axis = None
        axis_elem = joint_elem.find("axis")
        if axis_elem is not None:
            xyz = axis_elem.get("xyz", "0 0 1").split()
            if len(xyz) == 3:
                axis = [float(x) for x in xyz]

        # 创建关节Prim
        asset.create_joint_prim(
            joint_path,
            joint_type,
            f"/{robot_name}/{parent_link}" if parent_link else None,
            f"/{robot_name}/{child_link}" if child_link else None,
            axis,
        )
