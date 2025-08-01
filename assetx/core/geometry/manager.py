#!/usr/bin/env python3
"""
几何体管理器 - 负责几何体创建和基础操作
"""

from pathlib import Path
from typing import Union, Optional, Dict, Any, List
import logging

from ..primitives import AssetPrim, SdfPath
from .mesh import AssetMesh
from .xform import AssetXform

logger = logging.getLogger(__name__)


class GeometryManager:
    """几何体管理器
    
    专门负责几何体的创建和基础管理：
    - 网格Prim创建
    - 变换Prim创建
    - 文件加载
    - 基础验证
    """

    def __init__(self, asset: 'Asset'):
        """初始化管理器
        
        Args:
            asset: 所属的Asset对象
        """
        self.asset = asset
        
    def create_mesh_prim(self, path: Union[str, SdfPath], 
                        name: str = "") -> AssetMesh:
        """创建网格Prim
        
        Args:
            path: SDF路径
            name: 网格名称
            
        Returns:
            创建的AssetMesh对象
        """
        if isinstance(path, str):
            path = SdfPath(path)
            
        # 使用AssetStage的define_prim创建基础prim
        base_prim = self.asset.stage.define_prim(path, "Mesh")
        
        # 创建AssetMesh
        mesh_prim = AssetMesh(self.asset.stage, path, name)
        
        return mesh_prim
        
    def create_xform_prim(self, path: Union[str, SdfPath], 
                         name: str = "") -> AssetXform:
        """创建变换Prim
        
        Args:
            path: SDF路径
            name: 变换名称
            
        Returns:
            创建的AssetXform对象
        """
        if isinstance(path, str):
            path = SdfPath(path)
            
        # 使用AssetStage的define_prim创建基础prim
        base_prim = self.asset.stage.define_prim(path, "Xform")
        
        # 创建AssetXform
        xform_prim = AssetXform(self.asset.stage, path, name)
        
        return xform_prim
        
    def load_mesh_file(self, mesh_prim: AssetMesh, 
                      file_path: Union[str, Path]) -> bool:
        """加载网格文件到AssetMesh
        
        Args:
            mesh_prim: 目标AssetMesh
            file_path: 网格文件路径
            
        Returns:
            是否加载成功
        """
        file_path = Path(file_path)
        
        if not file_path.exists():
            logger.error(f"Mesh file not found: {file_path}")
            return False
            
        try:
            # 设置文件引用
            mesh_prim.set_mesh_file(file_path)
            
            # 如果有MeshProcessor，尝试加载网格数据
            try:
                from ...mesh.processor import MeshProcessor
                mesh_processor = MeshProcessor()
                if mesh_processor.has_trimesh:
                    import trimesh
                    mesh = trimesh.load(str(file_path))
                    
                    if hasattr(mesh, 'vertices') and hasattr(mesh, 'faces'):
                        mesh_prim.set_mesh_data(mesh.vertices, mesh.faces)
            except ImportError:
                logger.debug("MeshProcessor not available for detailed mesh loading")
                    
            return True
            
        except Exception as e:
            logger.error(f"Failed to load mesh file {file_path}: {e}")
            return False
            
    def get_geometry_info(self, prim: AssetPrim) -> Dict[str, Any]:
        """获取几何体信息
        
        Args:
            prim: AssetPrim对象
            
        Returns:
            几何体信息字典
        """
        info = {
            'type': prim.type_name,
            'path': str(prim.path),
            'name': prim.name
        }
        
        if isinstance(prim, AssetMesh):
            # 网格信息
            mesh_file = prim.get_mesh_file()
            if mesh_file:
                info['mesh_file'] = str(mesh_file)
                info['file_exists'] = mesh_file.exists()
                    
            # 几何体类型
            geom_type = prim.get_geometry_type()
            if geom_type:
                info['geometry_type'] = geom_type.value
                
            # 包围盒
            bbox = prim.get_bounding_box()
            if bbox:
                info['bounding_box'] = {
                    'min': bbox[0].tolist(),
                    'max': bbox[1].tolist()
                }
                
        elif isinstance(prim, AssetXform):
            # 变换信息
            info['translation'] = prim.get_translation()
            info['rotation'] = prim.get_rotation()
            info['scale'] = prim.get_scale()
            
        return info
        
    def validate_geometry(self) -> Dict[str, Any]:
        """验证所有几何体
        
        Returns:
            验证结果
        """
        results = {
            'valid_meshes': [],
            'invalid_meshes': [],
            'missing_files': [],
            'total_meshes': 0,
            'total_xforms': 0
        }
        
        # 遍历所有几何体Prim
        for prim in self.asset.stage.traverse():
            if isinstance(prim, AssetMesh):
                results['total_meshes'] += 1
                
                if prim.is_valid_mesh():
                    results['valid_meshes'].append(str(prim.path))
                else:
                    results['invalid_meshes'].append(str(prim.path))
                    
                # 检查文件引用
                mesh_file = prim.get_mesh_file()
                if mesh_file and not mesh_file.exists():
                    results['missing_files'].append(str(mesh_file))
                    
            elif isinstance(prim, AssetXform):
                results['total_xforms'] += 1
                
        return results
        
    def get_supported_formats(self) -> List[str]:
        """获取支持的文件格式"""
        formats = []
        
        try:
            from ...mesh.processor import MeshProcessor
            mesh_processor = MeshProcessor()
            formats.extend(mesh_processor.supported_formats)
        except ImportError:
            logger.debug("MeshProcessor not available")
            
        return list(set(formats))  # 去重
