#!/usr/bin/env python3
"""
GeometryProcessor - USD风格的几何体处理器 (重构版)

整合AssetX的几何体处理功能，作为统一的入口点。
通过组合模式将功能委托给专门的子处理器。
"""

from pathlib import Path
from typing import Dict, List, Optional, Union, Any
import logging

from ..primitives import AssetPrim, SdfPath
from .mesh import AssetMesh
from .xform import AssetXform
from .manager import GeometryManager
from .renderer import GeometryRenderer
from .mesh_processor import GeometryMeshProcessor

logger = logging.getLogger(__name__)


class GeometryProcessor:
    """USD风格的几何体处理器
    
    作为几何体处理的统一入口，委托具体功能给专门的子处理器：
    - GeometryManager: 几何体创建和管理
    - GeometryRenderer: 可视化和渲染
    - GeometryMeshProcessor: 网格处理
    """

    def __init__(self, asset: 'Asset'):
        """初始化GeometryProcessor
        
        Args:
            asset: 所属的Asset对象
        """
        self.asset = asset
        
        # 初始化子处理器
        self.manager = GeometryManager(asset)
        self.renderer = GeometryRenderer(asset)
        self.mesh_processor = GeometryMeshProcessor(asset)
        
    # ============ 几何体创建和管理 (委托给GeometryManager) ============
    
    def create_mesh_prim(self, path: Union[str, SdfPath], 
                        name: str = "") -> AssetMesh:
        """创建网格Prim"""
        return self.manager.create_mesh_prim(path, name)
        
    def create_xform_prim(self, path: Union[str, SdfPath], 
                         name: str = "") -> AssetXform:
        """创建变换Prim"""
        return self.manager.create_xform_prim(path, name)
        
    def load_mesh_file(self, mesh_prim: AssetMesh, 
                      file_path: Union[str, Path]) -> bool:
        """加载网格文件到AssetMesh"""
        return self.manager.load_mesh_file(mesh_prim, file_path)
        
    def get_geometry_info(self, prim: AssetPrim) -> Dict[str, Any]:
        """获取几何体信息"""
        return self.manager.get_geometry_info(prim)
        
    def validate_geometry(self) -> Dict[str, Any]:
        """验证所有几何体"""
        return self.manager.validate_geometry()
        
    def get_supported_formats(self) -> List[str]:
        """获取支持的文件格式"""
        return self.manager.get_supported_formats()
        
    # ============ 网格处理 (委托给GeometryMeshProcessor) ============
    
    def process_mesh(self, mesh_prim: AssetMesh, 
                    operation: str, **kwargs) -> bool:
        """处理网格几何体"""
        return self.mesh_processor.process_mesh(mesh_prim, operation, **kwargs)
        
    def has_mesh_support(self) -> bool:
        """检查是否有网格处理支持"""
        return self.mesh_processor.has_mesh_support()
        
    # ============ 可视化和渲染 (委托给GeometryRenderer) ============
    
    def render_mesh(self, mesh_prim: AssetMesh, 
                   show_axes: bool = True, 
                   show_wireframe: bool = False) -> bool:
        """渲染网格"""
        return self.renderer.render_mesh(mesh_prim, show_axes, show_wireframe)
        
    def render_link(self, link_name: str) -> bool:
        """渲染机器人链接的所有几何体"""
        return self.renderer.render_link(link_name)
        
    def render_complete_robot(self, show_joints: bool = True, show_frames: bool = True) -> bool:
        """渲染完整的机器人（所有部件在同一个图中）"""
        return self.renderer.render_complete_robot(show_joints, show_frames)
        
    def has_render_support(self) -> bool:
        """检查是否有渲染支持"""
        return self.renderer.has_render_support()
        
    # ============ 统一状态检查 ============
        
    def __repr__(self) -> str:
        """字符串表示"""
        mesh_support = "✓" if self.has_mesh_support() else "✗"
        render_support = "✓" if self.has_render_support() else "✗"
        
        return (f"GeometryProcessor(asset='{self.asset.asset_path.name}', "
                f"mesh_support={mesh_support}, render_support={render_support})")


__all__ = ['GeometryProcessor']
