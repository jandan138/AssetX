#!/usr/bin/env python3
"""
AssetMesh - USD风格的网格几何体

对应USD的UsdGeomMesh，提供机器人资产中的几何体抽象。
"""

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np

from ..primitives import AssetPrim, SdfPath
from ..enums import GeometryType


class AssetMesh(AssetPrim):
    """USD风格的网格几何体类
    
    对应USD的UsdGeomMesh，提供：
    - 几何体数据管理（顶点、面、法向量）
    - 外部文件引用（.dae, .obj, .stl等）
    - 材质绑定
    - 空间变换
    """

    def __init__(self, stage: 'AssetStage', path: SdfPath, name: str = ""):
        """初始化AssetMesh
        
        Args:
            stage: 所属Stage
            path: SDF路径
            name: 网格名称（仅用于显示，实际名称来自路径）
        """
        super().__init__(stage, path)
        self.set_type_name("Mesh")
        self._mesh_data: Optional[Dict] = None
        self._file_reference: Optional[Path] = None
        
        # USD风格的几何属性
        self._setup_mesh_attributes()
    
    def _setup_mesh_attributes(self) -> None:
        """设置USD风格的mesh属性"""
        # 对应UsdGeomMesh的核心属性
        self.create_attribute("points", "float3[]")  # 顶点坐标
        self.create_attribute("faceVertexIndices", "int[]")  # 面顶点索引
        self.create_attribute("faceVertexCounts", "int[]")  # 每个面的顶点数
        self.create_attribute("normals", "float3[]")  # 法向量
        self.create_attribute("uvs", "float2[]")  # UV坐标
        
        # 外部文件引用（类似USD References）
        self.create_attribute("file:path", "string")  # 文件路径
        self.create_attribute("file:format", "token")  # 文件格式
        
        # 几何体元数据
        self.create_attribute("geom:type", "token")  # 几何类型（mesh, box, sphere等）
        self.create_attribute("geom:size", "float3")  # 基本几何体尺寸
        
        # 物理属性
        self.create_attribute("physics:mass", "float")
        self.create_attribute("physics:density", "float")
        
    def set_mesh_file(self, file_path: Union[str, Path]) -> None:
        """设置外部网格文件引用
        
        Args:
            file_path: 网格文件路径
        """
        file_path = Path(file_path)
        self._file_reference = file_path
        
        # 设置文件属性
        self.get_attribute("file:path").set(str(file_path))
        self.get_attribute("file:format").set(file_path.suffix.lower())
        
    def get_mesh_file(self) -> Optional[Path]:
        """获取网格文件路径"""
        file_attr = self.get_attribute("file:path")
        if file_attr and file_attr.get():
            return Path(file_attr.get())
        return self._file_reference
        
    def set_geometry_type(self, geom_type: GeometryType) -> None:
        """设置几何体类型
        
        Args:
            geom_type: 几何体类型（box, sphere, cylinder, mesh等）
        """
        self.get_attribute("geom:type").set(geom_type.value)
        
    def get_geometry_type(self) -> Optional[GeometryType]:
        """获取几何体类型"""
        type_attr = self.get_attribute("geom:type")
        if type_attr and type_attr.get():
            try:
                return GeometryType(type_attr.get())
            except ValueError:
                return None
        return None
        
    def set_box_size(self, size: List[float]) -> None:
        """设置box几何体尺寸"""
        if len(size) != 3:
            raise ValueError("Box size must have 3 dimensions")
        self.get_attribute("geom:size").set(size)
        self.set_geometry_type(GeometryType.BOX)
        
    def set_sphere_radius(self, radius: float) -> None:
        """设置sphere几何体半径"""
        self.get_attribute("geom:size").set([radius, radius, radius])
        self.set_geometry_type(GeometryType.SPHERE)
        
    def set_sphere_size(self, radius: float) -> None:
        """设置sphere几何体半径（别名方法）"""
        self.set_sphere_radius(radius)
        
    def set_cylinder_size(self, radius: float, height: float) -> None:
        """设置cylinder几何体尺寸"""
        self.get_attribute("geom:size").set([radius, radius, height])
        self.set_geometry_type(GeometryType.CYLINDER)
        
    def set_mesh_data(self, vertices: np.ndarray, faces: np.ndarray, 
                      normals: Optional[np.ndarray] = None) -> None:
        """设置网格数据（对应UsdGeomMesh属性）
        
        Args:
            vertices: 顶点坐标数组 (N, 3)
            faces: 面索引数组 (M, 3)
            normals: 法向量数组 (N, 3), 可选
        """
        # 设置顶点
        points_attr = self.get_attribute("points")
        points_attr.set(vertices.flatten().tolist())
        
        # 设置面索引
        face_indices = faces.flatten().tolist()
        self.get_attribute("faceVertexIndices").set(face_indices)
        
        # 设置每个面的顶点数（假设都是三角形）
        face_counts = [3] * len(faces)
        self.get_attribute("faceVertexCounts").set(face_counts)
        
        # 设置法向量
        if normals is not None:
            self.get_attribute("normals").set(normals.flatten().tolist())
            
        self._mesh_data = {
            'vertices': vertices,
            'faces': faces,
            'normals': normals
        }
        
    def get_mesh_data(self) -> Optional[Dict]:
        """获取网格数据"""
        if self._mesh_data:
            return self._mesh_data
            
        # 尝试从属性重构数据
        points_attr = self.get_attribute("points")
        faces_attr = self.get_attribute("faceVertexIndices")
        
        if points_attr.get() and faces_attr.get():
            points = np.array(points_attr.get()).reshape(-1, 3)
            faces = np.array(faces_attr.get()).reshape(-1, 3)
            
            normals_attr = self.get_attribute("normals")
            normals = None
            if normals_attr.get():
                normals = np.array(normals_attr.get()).reshape(-1, 3)
                
            return {
                'vertices': points,
                'faces': faces,
                'normals': normals
            }
        return None
        
    def get_bounding_box(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """获取包围盒（最小点，最大点）"""
        mesh_data = self.get_mesh_data()
        if mesh_data and 'vertices' in mesh_data:
            vertices = mesh_data['vertices']
            min_point = np.min(vertices, axis=0)
            max_point = np.max(vertices, axis=0)
            return min_point, max_point
        return None
        
    def compute_volume(self) -> float:
        """计算网格体积（仅支持闭合网格）"""
        mesh_data = self.get_mesh_data()
        if not mesh_data:
            return 0.0
            
        vertices = mesh_data['vertices']
        faces = mesh_data['faces']
        
        # 使用散度定理计算体积
        volume = 0.0
        for face in faces:
            v0, v1, v2 = vertices[face]
            # 三角形与原点形成的四面体体积
            volume += np.dot(v0, np.cross(v1, v2)) / 6.0
            
        return abs(volume)
        
    def is_valid_mesh(self) -> bool:
        """检查是否为有效网格"""
        # 检查文件引用
        if self.get_mesh_file() and self.get_mesh_file().exists():
            return True
            
        # 检查内嵌数据
        mesh_data = self.get_mesh_data()
        if mesh_data:
            vertices = mesh_data.get('vertices')
            faces = mesh_data.get('faces')
            
            if vertices is not None and faces is not None:
                return len(vertices) > 0 and len(faces) > 0
                
        # 检查基本几何体
        geom_type = self.get_geometry_type()
        size_attr = self.get_attribute("geom:size")
        
        if geom_type and size_attr.get():
            return True
            
        return False
        
    def __repr__(self) -> str:
        """字符串表示"""
        mesh_file = self.get_mesh_file()
        geom_type = self.get_geometry_type()
        
        info_parts = []
        if mesh_file:
            info_parts.append(f"file='{mesh_file.name}'")
        if geom_type:
            info_parts.append(f"type={geom_type.value}")
            
        info = ", ".join(info_parts) if info_parts else "empty"
        return f"AssetMesh(path='{self.path}', {info})"
