#!/usr/bin/env python3
"""
基础资产加载器

定义所有加载器的通用接口和行为
"""

from abc import ABC, abstractmethod
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..asset import Asset
    from ..primitives import AssetStage


class BaseAssetLoader(ABC):
    """资产加载器基类"""
    
    def __init__(self, asset_path: Path):
        """初始化加载器
        
        Args:
            asset_path: 资产文件路径
        """
        self.asset_path = asset_path
        
    @abstractmethod
    def load(self, asset: "Asset") -> None:
        """加载资产到Stage
        
        Args:
            asset: 目标Asset对象
        """
        pass
        
    @abstractmethod
    def can_load(self) -> bool:
        """检查是否能加载此格式
        
        Returns:
            是否支持加载
        """
        pass
        
    def _get_stage(self, asset: "Asset") -> "AssetStage":
        """获取Asset的Stage对象
        
        Args:
            asset: Asset对象
            
        Returns:
            AssetStage对象
        """
        if asset._stage is None:
            raise RuntimeError("Stage is not initialized")
        return asset._stage
