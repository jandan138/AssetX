#!/usr/bin/env python3
"""
AssetX USD风格架构 - 属性系统

实现USD的属性系统，包括Attribute和Relationship两种属性类型。
"""

import weakref
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from ..enums import PropertyType, VariabilityType
from .sdf_path import SdfPath

if TYPE_CHECKING:
    from .stage import AssetStage


class AssetProperty:
    """USD风格的属性基类

    所有属性的基础类，提供通用的属性操作接口。
    """

    def __init__(self, stage: "AssetStage", path: SdfPath, property_type: PropertyType):
        """初始化属性

        Args:
            stage: 所属的Stage
            path: 属性路径
            property_type: 属性类型
        """
        self._stage_ref = weakref.ref(stage)
        self._path = path
        self._property_type = property_type
        self._metadata: Dict[str, Any] = {}
        self._custom = True
        self._variability = VariabilityType.VARYING

    @property
    def stage(self) -> "AssetStage":
        """获取所属Stage"""
        stage = self._stage_ref()
        if stage is None:
            raise RuntimeError("Stage has been destroyed")
        return stage

    @property
    def path(self) -> SdfPath:
        """获取属性路径"""
        return self._path

    @property
    def name(self) -> str:
        """获取属性名称"""
        return self._path.get_name()

    @property
    def property_type(self) -> PropertyType:
        """获取属性类型"""
        return self._property_type

    @property
    def is_custom(self) -> bool:
        """是否为自定义属性"""
        return self._custom

    @property
    def variability(self) -> VariabilityType:
        """获取变化性类型"""
        return self._variability

    def is_valid(self) -> bool:
        """属性是否有效"""
        return self._stage_ref() is not None

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


class AssetAttribute(AssetProperty):
    """USD风格的属性类

    表示可以存储值的属性，支持时间采样和类型定义。
    """

    def __init__(self, stage: "AssetStage", path: SdfPath, type_name: str = None):
        """初始化Attribute

        Args:
            stage: 所属的Stage
            path: 属性路径
            type_name: 属性类型名称
        """
        super().__init__(stage, path, PropertyType.ATTRIBUTE)
        self._type_name = type_name
        self._default_value: Any = None
        self._time_samples: Dict[float, Any] = {}
        self._connections: List[SdfPath] = []

    @property
    def type_name(self) -> str:
        """获取类型名称"""
        return self._type_name

    def get(self, time: Optional[float] = None) -> Any:
        """获取属性值

        Args:
            time: 时间点，None表示获取默认值

        Returns:
            属性值
        """
        if time is not None and time in self._time_samples:
            return self._time_samples[time]
        return self._default_value

    def set(self, value: Any, time: Optional[float] = None) -> bool:
        """设置属性值

        Args:
            value: 要设置的值
            time: 时间点，None表示设置默认值

        Returns:
            是否设置成功
        """
        if time is not None:
            self._time_samples[time] = value
        else:
            self._default_value = value
        return True

    def get_time_samples(self) -> Dict[float, Any]:
        """获取所有时间采样

        Returns:
            时间采样字典的副本
        """
        return self._time_samples.copy()

    def has_authored_value(self) -> bool:
        """是否有作者设定的值

        Returns:
            True如果有设定的值
        """
        return self._default_value is not None or bool(self._time_samples)

    def clear_default_value(self) -> bool:
        """清除默认值"""
        self._default_value = None
        return True

    def block_connections(self) -> bool:
        """阻断连接"""
        self._connections.clear()
        return True

    def get_connections(self) -> List[SdfPath]:
        """获取连接

        Returns:
            连接路径列表
        """
        return self._connections.copy()


class AssetRelationship(AssetProperty):
    """USD风格的关系类

    表示对象之间的关系，通过目标路径来建立连接。
    """

    def __init__(self, stage: "AssetStage", path: SdfPath):
        """初始化Relationship

        Args:
            stage: 所属的Stage
            path: 关系路径
        """
        super().__init__(stage, path, PropertyType.RELATIONSHIP)
        self._targets: List[SdfPath] = []
        self._forwarded_targets: List[SdfPath] = []

    def add_target(self, target_path: SdfPath) -> bool:
        """添加目标

        Args:
            target_path: 目标路径

        Returns:
            是否添加成功
        """
        if target_path not in self._targets:
            self._targets.append(target_path)
            return True
        return False

    def remove_target(self, target_path: SdfPath) -> bool:
        """移除目标

        Args:
            target_path: 目标路径

        Returns:
            是否移除成功
        """
        if target_path in self._targets:
            self._targets.remove(target_path)
            return True
        return False

    def set_targets(self, target_paths: List[SdfPath]) -> bool:
        """设置所有目标

        Args:
            target_paths: 目标路径列表

        Returns:
            是否设置成功
        """
        self._targets = target_paths.copy()
        return True

    def get_targets(self) -> List[SdfPath]:
        """获取所有目标

        Returns:
            目标路径列表的副本
        """
        return self._targets.copy()

    def get_forwarded_targets(self) -> List[SdfPath]:
        """获取转发的目标

        Returns:
            转发目标路径列表
        """
        # 这里可以实现更复杂的目标解析逻辑
        return self._targets.copy()

    def has_authored_targets(self) -> bool:
        """是否有作者设定的目标

        Returns:
            True如果有设定的目标
        """
        return bool(self._targets)


__all__ = ["AssetProperty", "AssetAttribute", "AssetRelationship"]
