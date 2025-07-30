"""
AssetX - Multi-format robot simulation asset bridge tool
"""

__version__ = "0.1.0"
__author__ = "AssetX Team"
__email__ = "contact@assetx.dev"

from .core.asset import Asset
from .core.converter import FormatConverter
from .core.enums import AssetFormat, PropertyType, VariabilityType
from .core.validator import PhysicsValidator
from .mesh.processor import MeshProcessor
from .meta.manager import MetaManager
from .viewer.preview import Previewer

__all__ = [
    "Asset",
    "AssetFormat",
    "PropertyType",
    "VariabilityType",
    "FormatConverter",
    "PhysicsValidator",
    "MeshProcessor",
    "MetaManager",
    "Previewer",
]
