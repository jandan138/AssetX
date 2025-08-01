#!/usr/bin/env python3
"""
MJCF 格式加载器
"""

from typing import TYPE_CHECKING

from .base import BaseAssetLoader

if TYPE_CHECKING:
    from ..asset import Asset


class MjcfLoader(BaseAssetLoader):
    """MJCF格式加载器"""
    
    def can_load(self) -> bool:
        """检查是否为MJCF文件"""
        return self.asset_path.suffix.lower() == '.xml'
    
    def load(self, asset: "Asset") -> None:
        """加载MJCF格式"""
        # TODO: 实现MJCF加载逻辑
        print(f"Loading MJCF format from {self.asset_path}")
        
        # 创建基本的机器人结构作为占位符
        robot_name = self.asset_path.stem
        robot_prim = asset.create_robot_prim(f"/{robot_name}", robot_name)
        stage = self._get_stage(asset)
        stage.set_default_prim(robot_prim)
