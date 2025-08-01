#!/usr/bin/env python3
"""
几何体处理器 - 负责网格处理和高级操作
"""

from pathlib import Path
from typing import Optional
import logging

from .mesh import AssetMesh

logger = logging.getLogger(__name__)


class GeometryMeshProcessor:
    """几何体网格处理器
    
    专门负责网格的高级处理操作：
    - 网格简化
    - 网格缩放、居中
    - 单位转换
    - 网格优化
    """

    def __init__(self, asset: 'Asset'):
        """初始化处理器
        
        Args:
            asset: 所属的Asset对象
        """
        self.asset = asset
        self._mesh_processor = None
        self._check_dependencies()
        
    def _check_dependencies(self) -> None:
        """检查网格处理依赖"""
        try:
            from ...mesh.processor import MeshProcessor
            self._mesh_processor = MeshProcessor()
        except ImportError:
            logger.warning("MeshProcessor not available")
            
    def has_mesh_support(self) -> bool:
        """检查是否有网格处理支持"""
        return self._mesh_processor is not None
        
    def process_mesh(self, mesh_prim: AssetMesh, 
                    operation: str, **kwargs) -> bool:
        """处理网格几何体
        
        Args:
            mesh_prim: 要处理的AssetMesh
            operation: 处理操作（simplify, center, scale等）
            **kwargs: 操作参数
            
        Returns:
            是否处理成功
        """
        if not self._mesh_processor:
            logger.error("MeshProcessor not available")
            return False
            
        mesh_file = mesh_prim.get_mesh_file()
        if not mesh_file or not mesh_file.exists():
            logger.error("No valid mesh file found")
            return False
            
        try:
            if operation == "simplify":
                target_faces = kwargs.get('target_faces', 1000)
                output_path = self._mesh_processor.simplify_mesh(
                    mesh_file, target_faces=target_faces
                )
                mesh_prim.set_mesh_file(output_path)
                
            elif operation == "center":
                output_path = self._mesh_processor.center_mesh(mesh_file)
                mesh_prim.set_mesh_file(output_path)
                
            elif operation == "scale":
                scale_factor = kwargs.get('scale_factor', 1.0)
                output_path = self._mesh_processor.scale_mesh(
                    mesh_file, scale_factor
                )
                mesh_prim.set_mesh_file(output_path)
                
            elif operation == "normalize_units":
                source_unit = kwargs.get('source_unit', 'mm')
                target_unit = kwargs.get('target_unit', 'm')
                output_path = self._mesh_processor.normalize_units(
                    mesh_file, source_unit, target_unit
                )
                mesh_prim.set_mesh_file(output_path)
                
            else:
                logger.error(f"Unknown operation: {operation}")
                return False
                
            return True
            
        except Exception as e:
            logger.error(f"Mesh processing failed: {e}")
            return False
