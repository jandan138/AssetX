"""
AssetX Core Geometry Module

USD风格的几何体处理系统，提供与USD UsdGeom相对应的功能：
- AssetMesh: 对应 UsdGeomMesh
- AssetXform: 对应 UsdGeomXform  
- MeshReference: 对应 USD References
- GeometryProcessor: 几何体处理和优化（重构为组合模式）
- GeometryManager: 几何体创建和管理
- GeometryRenderer: 可视化和渲染（重构为组合模式）
- GeometryMeshProcessor: 网格处理
- RenderBackend: 渲染后端管理
- MatplotlibPlotter: matplotlib绘图工具
- RobotVisualizer: 机器人可视化
"""

from .mesh import AssetMesh
from .xform import AssetXform  
from .reference import MeshReference
from .processor import GeometryProcessor
from .manager import GeometryManager
from .renderer import GeometryRenderer
from .mesh_processor import GeometryMeshProcessor
from .backend import RenderBackend
from .plotting import MatplotlibPlotter
from .robot_visualizer import RobotVisualizer

__all__ = [
    'AssetMesh',
    'AssetXform', 
    'MeshReference',
    'GeometryProcessor',
    'GeometryManager',
    'GeometryRenderer', 
    'GeometryMeshProcessor',
    'RenderBackend',
    'MatplotlibPlotter',
    'RobotVisualizer'
]
