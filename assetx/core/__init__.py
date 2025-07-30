#!/usr/bin/env python3
"""
AssetX USD风格架构 - 核心模块

提供USD风格的资产管理系统，包含完整的USD核心概念实现。

Architecture:
    - Asset: 主入口类，提供统一的资产操作接口
    - AssetStage: 场景容器，管理所有Prim对象
    - AssetPrim: 场景对象，表示场景中的节点
    - AssetProperty/AssetAttribute/AssetRelationship: 属性系统
    - SdfPath: USD风格的路径系统
    - Enums: 系统枚举类型

Usage:
    >>> from assetx.core import Asset
    >>> asset = Asset("robot.urdf")
    >>> stage = asset.get_stage()
    >>> links = asset.find_prims_by_type("Link")
"""

# 核心类导入
from .asset import Asset
from .enums import AssetFormat, PropertyType, VariabilityType
from .prim import AssetPrim
from .property import AssetAttribute, AssetProperty, AssetRelationship
from .sdf_path import SdfPath
from .stage import AssetStage

# 版本信息
__version__ = "2.0.0"
__usd_version__ = "24.05"  # 基于的USD版本

# 公开接口
__all__ = [
    # 主要类
    "Asset",
    "AssetStage",
    "AssetPrim",
    # 属性系统
    "AssetProperty",
    "AssetAttribute",
    "AssetRelationship",
    # 路径系统
    "SdfPath",
    # 枚举类型
    "AssetFormat",
    "PropertyType",
    "VariabilityType",
]


# 模块级别便利函数
def create_asset(path: str) -> Asset:
    """创建Asset的便利函数

    Args:
        path: 资产文件路径

    Returns:
        Asset对象
    """
    return Asset(path)


def create_stage(identifier: str = None) -> AssetStage:
    """创建Stage的便利函数

    Args:
        identifier: Stage标识符

    Returns:
        AssetStage对象
    """
    return AssetStage(identifier)


# 系统信息
def get_version_info() -> dict:
    """获取版本信息

    Returns:
        版本信息字典
    """
    return {
        "assetx_version": __version__,
        "usd_base_version": __usd_version__,
        "architecture": "USD-inspired",
        "python_minimum": "3.7+",
    }
