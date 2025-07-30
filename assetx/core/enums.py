#!/usr/bin/env python3
"""
AssetX USD风格架构 - 枚举类型定义

包含系统中使用的所有枚举类型，遵循USD规范。
"""

from enum import Enum


class AssetFormat(Enum):
    """支持的资产格式"""

    URDF = "urdf"
    MJCF = "mjcf"
    USD = "usd"
    GENESIS_JSON = "genesis_json"
    UNKNOWN = "unknown"


class PropertyType(Enum):
    """属性类型枚举"""

    ATTRIBUTE = "attribute"
    RELATIONSHIP = "relationship"


class VariabilityType(Enum):
    """属性变化性类型"""

    VARYING = "varying"  # 可以随时间变化
    UNIFORM = "uniform"  # 在一个时间段内保持一致
    CONFIG = "config"  # 配置时设定，运行时不变


__all__ = ["AssetFormat", "PropertyType", "VariabilityType"]
