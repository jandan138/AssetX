#!/usr/bin/env python3
"""
AssetX USD风格架构 - Asset主类 (重构版)

提供Asset的主要接口，专注于格式加载和高级操作。
通过组合模式将功能模块化，保持主类的简洁性。
"""

from pathlib import Path
from typing import Any, Dict, List, Optional, Union

from ..meta.manager import MetaManager
from .enums import AssetFormat
from .primitives import AssetPrim, SdfPath, AssetStage

# 功能模块
from .modules import AssetQuery, AssetSchema, AssetMeta

# 几何处理模块
from .geometry import GeometryProcessor

# 加载器
from .loaders import (
    BaseAssetLoader,
    UrdfLoader,
    UsdLoader,
    MjcfLoader,
    GenesisLoader
)


class Asset:
    """USD风格的Asset类 - 资产管理的主入口

    提供统一的接口来加载、操作和查询各种格式的资产文件。
    通过组合模式集成各种功能模块，保持代码的模块化和可维护性。
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
        
        # 功能模块 - 使用组合模式
        self.query = AssetQuery(self)
        self.schema = AssetSchema(self)
        self.meta = AssetMeta(self)
        
        # 几何处理模块
        self.geometry = GeometryProcessor(self)

    @property
    def stage(self) -> Optional[AssetStage]:
        """获取Asset的Stage对象"""
        return self._stage

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

    # ============ Schema应用接口 (委托给schema模块) ============

    def create_robot_prim(
        self, path: Union[str, SdfPath], name: str = None
    ) -> AssetPrim:
        """创建机器人Prim - 委托给schema模块"""
        return self.schema.create_robot_prim(path, name)

    def create_link_prim(
        self, path: Union[str, SdfPath], mass: float = None, inertia: List[float] = None
    ) -> AssetPrim:
        """创建链接Prim - 委托给schema模块"""
        return self.schema.create_link_prim(path, mass, inertia)

    def create_joint_prim(
        self,
        path: Union[str, SdfPath],
        joint_type: str = "revolute",
        parent_link: str = None,
        child_link: str = None,
        axis: List[float] = None,
    ) -> AssetPrim:
        """创建关节Prim - 委托给schema模块"""
        return self.schema.create_joint_prim(path, joint_type, parent_link, child_link, axis)

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

    def _get_loader(self) -> BaseAssetLoader:
        """根据格式获取对应的加载器
        
        Returns:
            对应的加载器实例
        """
        loader_map = {
            AssetFormat.URDF: UrdfLoader,
            AssetFormat.MJCF: MjcfLoader,
            AssetFormat.USD: UsdLoader,
            AssetFormat.GENESIS_JSON: GenesisLoader,
        }
        
        loader_class = loader_map.get(self.format)
        if loader_class is None:
            raise ValueError(f"Unsupported format: {self.format}")
            
        return loader_class(self.asset_path)

    def load(self) -> None:
        """加载资产数据"""
        if self._is_loaded:
            return

        # 创建Stage
        self._stage = AssetStage(str(self.asset_path))

        # 获取对应的加载器并加载
        loader = self._get_loader()
        if not loader.can_load():
            raise ValueError(f"Cannot load file: {self.asset_path}")
            
        loader.load(self)
        self._is_loaded = True

    # ============ 特殊方法 ============

    def __repr__(self) -> str:
        format_str = (
            self.format.value if hasattr(self.format, "value") else str(self.format)
        )
        return f"Asset(path='{self.asset_path}', format='{format_str}', loaded={self._is_loaded})"


__all__ = ["Asset"]
