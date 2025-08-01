#!/usr/bin/env python3
"""
AssetX Core Primitives - USD风格基础组件

提供USD风格的基础类: Prim, Stage, SdfPath, Property
"""

from .prim import AssetPrim
from .stage import AssetStage  
from .sdf_path import SdfPath
from .property import AssetProperty, AssetRelationship

__all__ = [
    "AssetPrim",
    "AssetStage", 
    "SdfPath",
    "AssetProperty",
    "AssetRelationship"
]
