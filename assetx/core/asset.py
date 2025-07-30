#!/usr/bin/env python3
"""
AssetX USD风格架构 - Asset主类

提供Asset的主要接口，专注于格式加载和高级操作。
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Union

from ..meta.manager import MetaManager
from .enums import AssetFormat
from .prim import AssetPrim
from .sdf_path import SdfPath
from .stage import AssetStage


class Asset:
    """USD风格的Asset类 - 资产管理的主入口

    提供统一的接口来加载、操作和查询各种格式的资产文件。
    """

    def __init__(self, asset_path: Union[str, Path]):
        """初始化Asset

        Args:
            asset_path: 资产文件路径
        """
        self.asset_path = Path(asset_path)
        self.format = self._detect_format()
        self._stage: Optional[AssetStage] = None
        self._is_loaded = False
        self.meta_manager = MetaManager(self.asset_path)

    @property
    def is_loaded(self) -> bool:
        """是否已加载"""
        return self._is_loaded

    # ============ 核心USD接口 ============

    def get_stage(self) -> AssetStage:
        """获取Stage对象

        Returns:
            AssetStage对象
        """
        if not self._is_loaded:
            self.load()
        return self._stage

    def get_default_prim(self) -> Optional[AssetPrim]:
        """获取默认Prim

        Returns:
            默认AssetPrim对象或None
        """
        stage = self.get_stage()
        return stage.get_default_prim() if stage else None

    def set_default_prim(self, prim: AssetPrim) -> bool:
        """设置默认Prim

        Args:
            prim: 要设为默认的Prim

        Returns:
            是否设置成功
        """
        stage = self.get_stage()
        return stage.set_default_prim(prim) if stage else False

    def get_prim_at_path(self, path: Union[str, SdfPath]) -> Optional[AssetPrim]:
        """根据路径获取Prim

        Args:
            path: Prim路径

        Returns:
            AssetPrim对象或None
        """
        if isinstance(path, str):
            path = SdfPath(path)

        stage = self.get_stage()
        return stage.get_prim_at_path(path) if stage else None

    def define_prim(
        self, path: Union[str, SdfPath], type_name: str = None
    ) -> AssetPrim:
        """定义Prim

        Args:
            path: Prim路径
            type_name: Prim类型名称

        Returns:
            AssetPrim对象
        """
        if isinstance(path, str):
            path = SdfPath(path)

        # 直接使用 _stage，避免循环调用
        if self._stage is None:
            raise RuntimeError("Stage is not initialized. Call load() first.")
        return self._stage.define_prim(path, type_name)

    def remove_prim(self, path: Union[str, SdfPath]) -> bool:
        """移除Prim

        Args:
            path: Prim路径

        Returns:
            是否移除成功
        """
        if isinstance(path, str):
            path = SdfPath(path)

        # 直接使用 _stage，避免循环调用
        if self._stage is None:
            return False
        return self._stage.remove_prim(path)

    # ============ 层次结构操作 ============

    def get_pseudo_root(self) -> Optional[AssetPrim]:
        """获取伪根Prim

        Returns:
            伪根AssetPrim对象或None
        """
        # 直接使用 _stage，避免循环调用
        if self._stage is None:
            return None
        return self._stage.get_pseudo_root()

    def get_root_prims(self) -> List[AssetPrim]:
        """获取所有根Prim

        Returns:
            根AssetPrim列表
        """
        pseudo_root = self.get_pseudo_root()
        return pseudo_root.get_children() if pseudo_root else []

    def get_all_prims(self) -> List[AssetPrim]:
        """获取所有Prim（除伪根外）

        Returns:
            所有AssetPrim列表
        """
        return self.traverse(lambda p: not p.path.path_string == "/")

    # ============ 机器人特定的Schema应用接口 ============

    def create_robot_prim(
        self, path: Union[str, SdfPath], name: str = None
    ) -> AssetPrim:
        """创建机器人Prim

        Args:
            path: Prim路径
            name: 机器人名称

        Returns:
            创建的AssetPrim对象
        """
        prim = self.define_prim(path, "Robot")
        prim.apply_api_schema("RobotAPI")
        if name:
            prim.set_metadata("displayName", name)
        return prim

    def create_link_prim(
        self, path: Union[str, SdfPath], mass: float = None, inertia: List[float] = None
    ) -> AssetPrim:
        """创建链接Prim

        Args:
            path: Prim路径
            mass: 质量
            inertia: 惯性矩阵

        Returns:
            创建的AssetPrim对象
        """
        prim = self.define_prim(path, "Link")
        prim.apply_api_schema("LinkAPI")
        prim.apply_api_schema("PhysicsAPI")

        if mass is not None:
            mass_attr = prim.create_attribute("physics:mass", "float")
            mass_attr.set(mass)

        if inertia is not None:
            inertia_attr = prim.create_attribute("physics:inertia", "matrix3d")
            inertia_attr.set(inertia)

        return prim

    def create_joint_prim(
        self,
        path: Union[str, SdfPath],
        joint_type: str = "revolute",
        parent_link: str = None,
        child_link: str = None,
        axis: List[float] = None,
    ) -> AssetPrim:
        """创建关节Prim

        Args:
            path: Prim路径
            joint_type: 关节类型
            parent_link: 父链接名称
            child_link: 子链接名称
            axis: 关节轴向

        Returns:
            创建的AssetPrim对象
        """
        prim = self.define_prim(path, "Joint")
        prim.apply_api_schema("JointAPI")

        # 设置关节类型
        type_attr = prim.create_attribute("joint:type", "string")
        type_attr.set(joint_type)

        # 设置父子关系
        if parent_link:
            parent_rel = prim.create_relationship("joint:parentLink")
            parent_rel.add_target(SdfPath(f"/{parent_link}"))

        if child_link:
            child_rel = prim.create_relationship("joint:childLink")
            child_rel.add_target(SdfPath(f"/{child_link}"))

        # 设置轴向
        if axis:
            axis_attr = prim.create_attribute("joint:axis", "vector3")
            axis_attr.set(axis)

        return prim

    # ============ 查询和遍历 ============

    def find_prims_by_type(self, type_name: str) -> List[AssetPrim]:
        """根据类型查找Prim

        Args:
            type_name: 类型名称

        Returns:
            匹配的AssetPrim列表
        """
        return self.traverse(lambda p: p.type_name == type_name)

    def find_prims_by_schema(self, schema_name: str) -> List[AssetPrim]:
        """根据Schema查找Prim

        Args:
            schema_name: Schema名称

        Returns:
            匹配的AssetPrim列表
        """
        return self.traverse(lambda p: p.has_api_schema(schema_name))

    def traverse(
        self, predicate: Callable[[AssetPrim], bool] = None
    ) -> List[AssetPrim]:
        """遍历所有Prim

        Args:
            predicate: 过滤条件函数

        Returns:
            符合条件的AssetPrim列表
        """
        stage = self.get_stage()
        return stage.traverse(predicate) if stage else []

    # ============ 元数据操作 ============

    def get_metadata(self, key: str) -> Any:
        """获取Asset级别元数据

        Args:
            key: 元数据键

        Returns:
            元数据值或None
        """
        stage = self.get_stage()
        return stage.get_metadata(key) if stage else None

    def set_metadata(self, key: str, value: Any) -> bool:
        """设置Asset级别元数据

        Args:
            key: 元数据键
            value: 元数据值

        Returns:
            是否设置成功
        """
        stage = self.get_stage()
        return stage.set_metadata(key, value) if stage else False

    # ============ 格式检测和加载 ============

    def _detect_format(self) -> AssetFormat:
        """根据文件扩展名检测格式

        Returns:
            检测到的AssetFormat
        """
        suffix = self.asset_path.suffix.lower()
        format_map = {
            ".urdf": AssetFormat.URDF,
            ".xml": AssetFormat.MJCF,
            ".usd": AssetFormat.USD,
            ".usda": AssetFormat.USD,
            ".usdc": AssetFormat.USD,
            ".json": AssetFormat.GENESIS_JSON,
        }
        return format_map.get(suffix, AssetFormat.UNKNOWN)

    def load(self) -> None:
        """加载资产数据"""
        if self._is_loaded:
            return

        # 创建Stage
        self._stage = AssetStage(str(self.asset_path))

        # 根据格式加载
        if self.format == AssetFormat.URDF:
            self._load_urdf()
        elif self.format == AssetFormat.MJCF:
            self._load_mjcf()
        elif self.format == AssetFormat.USD:
            self._load_usd()
        elif self.format == AssetFormat.GENESIS_JSON:
            self._load_genesis()
        else:
            raise ValueError(f"Unsupported format: {self.format}")

        self._is_loaded = True

    def _load_urdf(self) -> None:
        """加载URDF格式"""
        try:
            tree = ET.parse(self.asset_path)
            root = tree.getroot()

            # 获取机器人名称
            robot_name = root.get("name", "robot")

            # 创建机器人根Prim
            robot_prim = self.create_robot_prim(f"/{robot_name}", robot_name)
            self._stage.set_default_prim(robot_prim)

            # 加载链接
            for link_elem in root.findall("link"):
                self._load_urdf_link(link_elem, robot_name)

            # 加载关节
            for joint_elem in root.findall("joint"):
                self._load_urdf_joint(joint_elem, robot_name)

        except Exception as e:
            raise ValueError(f"Failed to load URDF: {e}")

    def _load_urdf_link(self, link_elem: ET.Element, robot_name: str) -> None:
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
        self.create_link_prim(link_path, mass, inertia)

    def _load_urdf_joint(self, joint_elem: ET.Element, robot_name: str) -> None:
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
        self.create_joint_prim(
            joint_path,
            joint_type,
            f"/{robot_name}/{parent_link}" if parent_link else None,
            f"/{robot_name}/{child_link}" if child_link else None,
            axis,
        )

    def _load_mjcf(self) -> None:
        """加载MJCF格式"""
        # TODO: 实现MJCF加载逻辑
        print(f"Loading MJCF format from {self.asset_path}")

    def _load_usd(self) -> None:
        """加载USD格式"""
        # TODO: 实现USD加载逻辑
        print(f"Loading USD format from {self.asset_path}")

    def _load_genesis(self) -> None:
        """加载Genesis JSON格式"""
        # TODO: 实现Genesis JSON加载逻辑
        print(f"Loading Genesis JSON format from {self.asset_path}")

    # ============ 高级功能 ============

    def get_asset_info(self) -> Dict[str, Any]:
        """获取资产信息摘要

        Returns:
            资产信息字典
        """
        return {
            "path": str(self.asset_path),
            "format": self.format.value,
            "loaded": self._is_loaded,
            "stage_id": self._stage.identifier if self._stage else None,
            "prim_count": len(self.get_all_prims()),
            "default_prim": (
                self.get_default_prim().name if self.get_default_prim() else None
            ),
        }

    def get_robot_name(self) -> Optional[str]:
        """获取机器人名称

        Returns:
            机器人名称或None
        """
        default_prim = self.get_default_prim()
        return default_prim.name if default_prim else None

    # ============ 特殊方法 ============

    def __repr__(self) -> str:
        format_str = (
            self.format.value if hasattr(self.format, "value") else str(self.format)
        )
        return f"Asset(path='{self.asset_path}', format='{format_str}', loaded={self._is_loaded})"


__all__ = ["Asset"]
