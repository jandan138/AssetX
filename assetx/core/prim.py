#!/usr/bin/env python3
"""
AssetX USD风格架构 - Prim对象

实现USD的Prim概念，表示场景中的对象节点。
"""

import weakref
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from .property import AssetAttribute, AssetProperty, AssetRelationship
from .sdf_path import SdfPath

if TYPE_CHECKING:
    from .stage import AssetStage


class AssetPrim:
    """USD风格的Prim类

    表示场景中的一个对象节点，可以包含属性、关系和子对象。
    """

    def __init__(self, stage: "AssetStage", path: SdfPath):
        """初始化Prim

        Args:
            stage: 所属的Stage
            path: Prim路径
        """
        self._stage_ref = weakref.ref(stage)
        self._path = path
        self._type_name: Optional[str] = None
        self._active = True
        self._instanceable = False
        self._children: Dict[str, "AssetPrim"] = {}
        self._properties: Dict[str, AssetProperty] = {}
        self._metadata: Dict[str, Any] = {}
        self._applied_schemas: List[str] = []

    @property
    def stage(self) -> "AssetStage":
        """获取所属Stage"""
        stage = self._stage_ref()
        if stage is None:
            raise RuntimeError("Stage has been destroyed")
        return stage

    @property
    def path(self) -> SdfPath:
        """获取Prim路径"""
        return self._path

    @property
    def name(self) -> str:
        """获取Prim名称"""
        return self._path.get_name()

    @property
    def type_name(self) -> Optional[str]:
        """获取类型名称"""
        return self._type_name

    def set_type_name(self, type_name: str) -> bool:
        """设置类型名"""
        self._type_name = type_name
        return True

    def is_valid(self) -> bool:
        """Prim是否有效"""
        return self._stage_ref() is not None

    def is_active(self) -> bool:
        """是否激活"""
        return self._active

    def set_active(self, active: bool) -> bool:
        """设置激活状态"""
        self._active = active
        return True

    def is_instanceable(self) -> bool:
        """是否可实例化"""
        return self._instanceable

    def set_instanceable(self, instanceable: bool) -> bool:
        """设置实例化状态"""
        self._instanceable = instanceable
        return True

    # ============ 子对象管理 ============

    def get_children(self) -> List["AssetPrim"]:
        """获取所有子Prim"""
        return list(self._children.values())

    def get_child(self, name: str) -> Optional["AssetPrim"]:
        """根据名称获取子Prim"""
        return self._children.get(name)

    def get_child_names(self) -> List[str]:
        """获取所有子Prim名称"""
        return list(self._children.keys())

    def has_child(self, name: str) -> bool:
        """是否有指定名称的子Prim"""
        return name in self._children

    # ============ 属性管理 ============

    def create_attribute(self, name: str, type_name: str) -> AssetAttribute:
        """创建属性

        Args:
            name: 属性名称
            type_name: 属性类型

        Returns:
            创建的AssetAttribute对象
        """
        attr_path = self._path.append_child(name)
        attr = AssetAttribute(self.stage, attr_path, type_name)
        self._properties[name] = attr
        return attr

    def get_attribute(self, name: str) -> Optional[AssetAttribute]:
        """获取属性

        Args:
            name: 属性名称

        Returns:
            AssetAttribute对象或None
        """
        prop = self._properties.get(name)
        return prop if isinstance(prop, AssetAttribute) else None

    def has_attribute(self, name: str) -> bool:
        """是否有指定属性"""
        return isinstance(self._properties.get(name), AssetAttribute)

    def get_attributes(self) -> List[AssetAttribute]:
        """获取所有属性"""
        return [
            prop
            for prop in self._properties.values()
            if isinstance(prop, AssetAttribute)
        ]

    # ============ 关系管理 ============

    def create_relationship(self, name: str) -> AssetRelationship:
        """创建关系

        Args:
            name: 关系名称

        Returns:
            创建的AssetRelationship对象
        """
        rel_path = self._path.append_child(name)
        rel = AssetRelationship(self.stage, rel_path)
        self._properties[name] = rel
        return rel

    def get_relationship(self, name: str) -> Optional[AssetRelationship]:
        """获取关系

        Args:
            name: 关系名称

        Returns:
            AssetRelationship对象或None
        """
        prop = self._properties.get(name)
        return prop if isinstance(prop, AssetRelationship) else None

    def has_relationship(self, name: str) -> bool:
        """是否有指定关系"""
        return isinstance(self._properties.get(name), AssetRelationship)

    def get_relationships(self) -> List[AssetRelationship]:
        """获取所有关系"""
        return [
            prop
            for prop in self._properties.values()
            if isinstance(prop, AssetRelationship)
        ]

    # ============ 属性和关系的统一管理 ============

    def get_properties(self) -> List[AssetProperty]:
        """获取所有属性和关系"""
        return list(self._properties.values())

    def get_property(self, name: str) -> Optional[AssetProperty]:
        """获取属性或关系

        Args:
            name: 属性/关系名称

        Returns:
            AssetProperty对象或None
        """
        return self._properties.get(name)

    def has_property(self, name: str) -> bool:
        """是否有指定属性或关系"""
        return name in self._properties

    def get_property_names(self) -> List[str]:
        """获取所有属性和关系名称"""
        return list(self._properties.keys())

    # ============ 元数据管理 ============

    def get_metadata(self, key: str) -> Any:
        """获取元数据"""
        return self._metadata.get(key)

    def set_metadata(self, key: str, value: Any) -> bool:
        """设置元数据"""
        self._metadata[key] = value
        return True

    def has_metadata(self, key: str) -> bool:
        """是否有指定元数据"""
        return key in self._metadata

    def clear_metadata(self, key: str) -> bool:
        """清除元数据"""
        if key in self._metadata:
            del self._metadata[key]
            return True
        return False

    def get_all_metadata(self) -> Dict[str, Any]:
        """获取所有元数据"""
        return self._metadata.copy()

    # ============ Schema管理 ============

    def apply_api_schema(self, schema_name: str) -> bool:
        """应用API Schema

        Args:
            schema_name: Schema名称

        Returns:
            是否应用成功
        """
        if schema_name not in self._applied_schemas:
            self._applied_schemas.append(schema_name)
            return True
        return False

    def remove_api_schema(self, schema_name: str) -> bool:
        """移除API Schema

        Args:
            schema_name: Schema名称

        Returns:
            是否移除成功
        """
        if schema_name in self._applied_schemas:
            self._applied_schemas.remove(schema_name)
            return True
        return False

    def has_api_schema(self, schema_name: str) -> bool:
        """是否应用了指定Schema

        Args:
            schema_name: Schema名称

        Returns:
            True如果已应用
        """
        return schema_name in self._applied_schemas

    def get_applied_schemas(self) -> List[str]:
        """获取所有应用的Schema"""
        return self._applied_schemas.copy()

    # ============ 特殊方法 ============

    def __repr__(self) -> str:
        type_info = f" ({self._type_name})" if self._type_name else ""
        return f"AssetPrim('{self._path}'{type_info})"


__all__ = ["AssetPrim"]
