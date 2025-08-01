#!/usr/bin/env python3
"""
MeshReference - USD风格的外部文件引用

对应USD的References系统，管理外部网格文件的引用。
"""

from pathlib import Path
from typing import Dict, List, Optional, Union

from ..primitives import AssetPrim, SdfPath


class MeshReference:
    """USD风格的网格文件引用管理
    
    对应USD的References系统，提供：
    - 外部文件引用管理
    - 相对路径解析
    - 文件格式支持检测
    - 延迟加载机制
    """

    def __init__(self, prim: AssetPrim):
        """初始化MeshReference
        
        Args:
            prim: 所属的AssetPrim
        """
        self.prim = prim
        self._references: List[Dict] = []
        self._base_path: Optional[Path] = None
        
    def add_reference(self, file_path: Union[str, Path], 
                     ref_type: str = "mesh") -> None:
        """添加外部文件引用
        
        Args:
            file_path: 外部文件路径
            ref_type: 引用类型（mesh, material, texture等）
        """
        file_path = Path(file_path)
        
        reference = {
            'path': file_path,
            'type': ref_type,
            'format': file_path.suffix.lower(),
            'resolved_path': self._resolve_path(file_path)
        }
        
        self._references.append(reference)
        
        # 如果是网格引用，设置到prim属性
        if ref_type == "mesh":
            if hasattr(self.prim, 'set_mesh_file'):
                self.prim.set_mesh_file(file_path)
        
    def _resolve_path(self, file_path: Path) -> Path:
        """解析文件路径（处理相对路径）
        
        Args:
            file_path: 原始文件路径
            
        Returns:
            解析后的绝对路径
        """
        if file_path.is_absolute():
            return file_path
            
        # 相对于base_path解析
        if self._base_path:
            return self._base_path / file_path
            
        # 相对于当前工作目录
        return Path.cwd() / file_path
        
    def set_base_path(self, base_path: Union[str, Path]) -> None:
        """设置基础路径（用于解析相对路径）
        
        Args:
            base_path: 基础路径
        """
        self._base_path = Path(base_path)
        
        # 重新解析所有引用
        for ref in self._references:
            ref['resolved_path'] = self._resolve_path(ref['path'])
            
    def get_references(self, ref_type: Optional[str] = None) -> List[Dict]:
        """获取文件引用列表
        
        Args:
            ref_type: 过滤引用类型，None表示返回所有
            
        Returns:
            引用列表
        """
        if ref_type:
            return [ref for ref in self._references if ref['type'] == ref_type]
        return self._references.copy()
        
    def get_mesh_files(self) -> List[Path]:
        """获取所有网格文件路径"""
        mesh_refs = self.get_references("mesh")
        return [ref['resolved_path'] for ref in mesh_refs]
        
    def get_material_files(self) -> List[Path]:
        """获取所有材质文件路径"""
        material_refs = self.get_references("material")
        return [ref['resolved_path'] for ref in material_refs]
        
    def validate_references(self) -> Dict[str, List[str]]:
        """验证所有引用文件是否存在
        
        Returns:
            验证结果：{'valid': [...], 'missing': [...]}
        """
        result = {'valid': [], 'missing': []}
        
        for ref in self._references:
            file_path = ref['resolved_path']
            if file_path.exists():
                result['valid'].append(str(file_path))
            else:
                result['missing'].append(str(file_path))
                
        return result
        
    def is_supported_format(self, file_path: Union[str, Path]) -> bool:
        """检查文件格式是否支持
        
        Args:
            file_path: 文件路径
            
        Returns:
            是否支持该格式
        """
        file_path = Path(file_path)
        suffix = file_path.suffix.lower()
        
        # 支持的网格格式
        mesh_formats = {'.obj', '.stl', '.ply', '.dae', '.fbx', '.glb', '.gltf'}
        
        # 支持的材质格式
        material_formats = {'.mtl', '.mat'}
        
        # 支持的纹理格式
        texture_formats = {'.png', '.jpg', '.jpeg', '.tga', '.bmp', '.tiff'}
        
        all_formats = mesh_formats | material_formats | texture_formats
        return suffix in all_formats
        
    def get_format_info(self) -> Dict[str, int]:
        """获取引用文件的格式统计
        
        Returns:
            格式统计字典
        """
        format_count = {}
        
        for ref in self._references:
            fmt = ref['format']
            format_count[fmt] = format_count.get(fmt, 0) + 1
            
        return format_count
        
    def clear_references(self, ref_type: Optional[str] = None) -> None:
        """清除文件引用
        
        Args:
            ref_type: 要清除的引用类型，None表示清除所有
        """
        if ref_type:
            self._references = [ref for ref in self._references 
                              if ref['type'] != ref_type]
        else:
            self._references.clear()
            
    def has_references(self, ref_type: Optional[str] = None) -> bool:
        """检查是否有文件引用
        
        Args:
            ref_type: 检查的引用类型，None表示检查所有
            
        Returns:
            是否有引用
        """
        if ref_type:
            return any(ref['type'] == ref_type for ref in self._references)
        return len(self._references) > 0
        
    def to_dict(self) -> Dict:
        """转换为字典表示"""
        return {
            'base_path': str(self._base_path) if self._base_path else None,
            'references': [
                {
                    'path': str(ref['path']),
                    'type': ref['type'],
                    'format': ref['format'],
                    'resolved_path': str(ref['resolved_path']),
                    'exists': ref['resolved_path'].exists()
                }
                for ref in self._references
            ]
        }
        
    def __len__(self) -> int:
        """返回引用数量"""
        return len(self._references)
        
    def __repr__(self) -> str:
        """字符串表示"""
        mesh_count = len(self.get_references("mesh"))
        material_count = len(self.get_references("material"))
        
        return (f"MeshReference(prim='{self.prim.path}', "
                f"meshes={mesh_count}, materials={material_count})")
