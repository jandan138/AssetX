#!/usr/bin/env python3
"""
AssetX USD风格架构 - Stage场景容器

实现USD的Stage概念，作为场景对象的主要容器。
"""

from threading import RLock
from typing import Any, Callable, Dict, List, Optional

from .prim import AssetPrim
from .sdf_path import SdfPath


class AssetStage:
    """USD风格的Stage类 - 资产的主要容器

    Stage是场景的根容器，管理所有Prim对象和场景级别的元数据。
    """

    def __init__(self, identifier: str = None):
        """初始化Stage

        Args:
            identifier: Stage标识符
        """
        self._identifier = identifier or "anonymous"
        self._root_prim: Optional[AssetPrim] = None
        self._prims: Dict[SdfPath, AssetPrim] = {}
        self._metadata: Dict[str, Any] = {}
        self._default_prim_path: Optional[SdfPath] = None
        self._lock = RLock()

        # 创建伪根
        self._pseudo_root = AssetPrim(self, SdfPath("/"))
        self._prims[SdfPath("/")] = self._pseudo_root

    @property
    def identifier(self) -> str:
        """获取Stage标识符"""
        return self._identifier

    # ============ 伪根和默认Prim管理 ============

    def get_pseudo_root(self) -> AssetPrim:
        """获取伪根prim

        Returns:
            伪根AssetPrim对象
        """
        return self._pseudo_root

    def get_default_prim(self) -> Optional[AssetPrim]:
        """获取默认prim

        Returns:
            默认AssetPrim对象或None
        """
        if self._default_prim_path:
            return self.get_prim_at_path(self._default_prim_path)
        return None

    def set_default_prim(self, prim: AssetPrim) -> bool:
        """设置默认prim

        Args:
            prim: 要设为默认的Prim

        Returns:
            是否设置成功
        """
        self._default_prim_path = prim.path
        return True

    def has_default_prim(self) -> bool:
        """是否有默认prim

        Returns:
            True如果有默认Prim
        """
        return self._default_prim_path is not None

    # ============ Prim管理 ============

    def get_prim_at_path(self, path: SdfPath) -> Optional[AssetPrim]:
        """根据路径获取prim

        Args:
            path: Prim路径

        Returns:
            AssetPrim对象或None
        """
        return self._prims.get(path)

    def define_prim(self, path: SdfPath, type_name: str = None) -> AssetPrim:
        """定义prim

        Args:
            path: Prim路径
            type_name: Prim类型名称

        Returns:
            创建或获取的AssetPrim对象
        """
        with self._lock:
            # 确保父路径存在
            parent_path = path.get_parent_path()
            if parent_path.path_string != "/" and parent_path not in self._prims:
                self.define_prim(parent_path)

            if path in self._prims:
                prim = self._prims[path]
                if type_name:
                    prim.set_type_name(type_name)
                return prim

            prim = AssetPrim(self, path)
            if type_name:
                prim.set_type_name(type_name)

            self._prims[path] = prim

            # 添加到父prim的子列表
            if parent_path in self._prims:
                parent_prim = self._prims[parent_path]
                parent_prim._children[path.get_name()] = prim

            return prim

    def override_prim(self, path: SdfPath) -> AssetPrim:
        """覆盖prim（如果不存在则创建）

        Args:
            path: Prim路径

        Returns:
            AssetPrim对象
        """
        return self.define_prim(path)

    def remove_prim(self, path: SdfPath) -> bool:
        """移除prim

        Args:
            path: Prim路径

        Returns:
            是否移除成功
        """
        with self._lock:
            if path not in self._prims:
                return False

            prim = self._prims[path]

            # 递归移除所有子prim
            for child in prim.get_children():
                self.remove_prim(child.path)

            # 从父prim中移除
            parent_path = path.get_parent_path()
            if parent_path in self._prims:
                parent_prim = self._prims[parent_path]
                if path.get_name() in parent_prim._children:
                    del parent_prim._children[path.get_name()]

            # 从stage中移除
            del self._prims[path]
            return True

    def traverse(
        self, predicate: Callable[[AssetPrim], bool] = None
    ) -> List[AssetPrim]:
        """遍历所有prim

        Args:
            predicate: 过滤条件函数

        Returns:
            符合条件的AssetPrim列表
        """
        result = []

        def _traverse(prim: AssetPrim):
            if predicate is None or predicate(prim):
                result.append(prim)
            for child in prim.get_children():
                _traverse(child)

        for child in self._pseudo_root.get_children():
            _traverse(child)

        return result

    # ============ 元数据管理 ============

    def get_metadata(self, key: str) -> Any:
        """获取stage元数据

        Args:
            key: 元数据键

        Returns:
            元数据值或None
        """
        return self._metadata.get(key)

    def set_metadata(self, key: str, value: Any) -> bool:
        """设置stage元数据

        Args:
            key: 元数据键
            value: 元数据值

        Returns:
            是否设置成功
        """
        self._metadata[key] = value
        return True

    def has_metadata(self, key: str) -> bool:
        """是否有指定元数据

        Args:
            key: 元数据键

        Returns:
            True如果存在
        """
        return key in self._metadata

    def clear_metadata(self, key: str) -> bool:
        """清除元数据

        Args:
            key: 元数据键

        Returns:
            是否清除成功
        """
        if key in self._metadata:
            del self._metadata[key]
            return True
        return False

    # ============ 静态方法 ============

    @staticmethod
    def create_new(identifier: str) -> "AssetStage":
        """创建新的stage

        Args:
            identifier: Stage标识符

        Returns:
            新的AssetStage对象
        """
        return AssetStage(identifier)

    @staticmethod
    def create_in_memory() -> "AssetStage":
        """在内存中创建stage

        Returns:
            内存中的AssetStage对象
        """
        return AssetStage()

    # ============ 特殊方法 ============

    def __repr__(self) -> str:
        return f"AssetStage(identifier='{self._identifier}', prims={len(self._prims)})"


__all__ = ["AssetStage"]
