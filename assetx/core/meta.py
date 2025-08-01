#!/usr/bin/env python3
"""
AssetX 元数据操作功能

提供Asset级别的元数据管理功能
"""

from typing import TYPE_CHECKING, Any, Dict

if TYPE_CHECKING:
    from .asset import Asset


class AssetMeta:
    """Asset元数据操作功能的集合"""
    
    def __init__(self, asset: "Asset"):
        """初始化元数据对象
        
        Args:
            asset: 目标Asset对象
        """
        self.asset = asset
    
    def get_metadata(self, key: str) -> Any:
        """获取Asset级别元数据

        Args:
            key: 元数据键

        Returns:
            元数据值或None
        """
        stage = self.asset.get_stage()
        return stage.get_metadata(key) if stage else None

    def set_metadata(self, key: str, value: Any) -> bool:
        """设置Asset级别元数据

        Args:
            key: 元数据键
            value: 元数据值

        Returns:
            是否设置成功
        """
        stage = self.asset.get_stage()
        return stage.set_metadata(key, value) if stage else False
    
    def get_all_metadata(self) -> Dict[str, Any]:
        """获取所有元数据
        
        Returns:
            元数据字典
        """
        stage = self.asset.get_stage()
        return stage.get_all_metadata() if stage else {}
    
    def remove_metadata(self, key: str) -> bool:
        """移除元数据
        
        Args:
            key: 元数据键
            
        Returns:
            是否移除成功
        """
        stage = self.asset.get_stage()
        return stage.remove_metadata(key) if stage else False
    
    def get_asset_info(self) -> Dict[str, Any]:
        """获取资产信息摘要

        Returns:
            资产信息字典
        """
        return {
            "path": str(self.asset.asset_path),
            "format": self.asset.format.value,
            "loaded": self.asset._is_loaded,
            "stage_id": self.asset._stage.identifier if self.asset._stage else None,
            "prim_count": len(self.asset.query.get_all_prims()),
            "default_prim": (
                self.asset.get_default_prim().name if self.asset.get_default_prim() else None
            ),
        }

    def get_robot_name(self) -> str:
        """获取机器人名称

        Returns:
            机器人名称或None
        """
        default_prim = self.asset.get_default_prim()
        return default_prim.name if default_prim else None
