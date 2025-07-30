"""
Core asset representation and management
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Union
import yaml

from ..meta.manager import MetaManager


@dataclass
class PhysicsProperties:
    """物理属性定义"""
    mass: Optional[float] = None
    inertia: Optional[List[float]] = None  # [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    center_of_mass: Optional[List[float]] = None  # [x, y, z]
    friction: Optional[float] = None
    restitution: Optional[float] = None


@dataclass
class GeometryInfo:
    """几何信息定义"""
    mesh_path: Optional[str] = None
    collision_mesh_path: Optional[str] = None
    scale: List[float] = None  # [x, y, z]
    origin: List[float] = None  # [x, y, z, roll, pitch, yaw]


@dataclass
class JointInfo:
    """关节信息定义"""
    name: str
    joint_type: str  # revolute, prismatic, fixed, etc.
    parent_link: str
    child_link: str
    axis: Optional[List[float]] = None  # [x, y, z]
    limits: Optional[Dict[str, float]] = None  # {lower, upper, effort, velocity}
    origin: Optional[List[float]] = None  # [x, y, z, roll, pitch, yaw]


class Asset:
    """统一资产表示类"""
    
    def __init__(self, asset_path: Union[str, Path]):
        self.asset_path = Path(asset_path)
        self.format = self._detect_format()
        self.meta_manager = MetaManager(self.asset_path.parent)
        
        # 核心数据结构
        self.links: Dict[str, Dict] = {}
        self.joints: Dict[str, JointInfo] = {}
        self.physics_properties: Dict[str, PhysicsProperties] = {}
        self.geometry_info: Dict[str, GeometryInfo] = {}
        
        # 元信息
        self.meta_data = self.meta_manager.load_meta(self.asset_path.name)
        
    def _detect_format(self) -> str:
        """根据文件扩展名检测格式"""
        suffix = self.asset_path.suffix.lower()
        format_map = {
            '.urdf': 'urdf',
            '.xml': 'mjcf',
            '.usd': 'usd',
            '.usda': 'usd',
            '.usdc': 'usd',
            '.json': 'genesis'
        }
        return format_map.get(suffix, 'unknown')
    
    def load(self) -> None:
        """加载资产数据"""
        if self.format == 'urdf':
            self._load_urdf()
        elif self.format == 'mjcf':
            self._load_mjcf()
        elif self.format == 'usd':
            self._load_usd()
        elif self.format == 'genesis':
            self._load_genesis()
        else:
            raise ValueError(f"Unsupported format: {self.format}")
    
    def _load_urdf(self) -> None:
        """加载URDF格式"""
        try:
            import urdfpy
        except ImportError:
            raise ImportError(
                "urdfpy is required for URDF support. "
                "Install with: pip install 'assetx[urdf]' or pip install urdfpy"
            )
        
        try:
            robot = urdfpy.URDF.load(str(self.asset_path))
            
            # 提取链接信息
            for link in robot.links:
                self.links[link.name] = {
                    'name': link.name,
                    'visual': link.visuals,
                    'collision': link.collisions,
                    'inertial': link.inertial
                }
                
                # 提取物理属性
                if link.inertial:
                    inertia = link.inertial.inertia
                    self.physics_properties[link.name] = PhysicsProperties(
                        mass=link.inertial.mass,
                        inertia=[inertia.ixx, inertia.iyy, inertia.izz, 
                                inertia.ixy, inertia.ixz, inertia.iyz],
                        center_of_mass=list(link.inertial.origin[:3, 3])
                    )
            
            # 提取关节信息
            for joint in robot.joints:
                self.joints[joint.name] = JointInfo(
                    name=joint.name,
                    joint_type=joint.joint_type,
                    parent_link=joint.parent,
                    child_link=joint.child,
                    axis=list(joint.axis) if joint.axis is not None else None,
                    limits={'lower': joint.limit.lower, 'upper': joint.limit.upper,
                           'effort': joint.limit.effort, 'velocity': joint.limit.velocity} 
                           if joint.limit else None,
                    origin=list(joint.origin.flatten()) if joint.origin is not None else None
                )
                
        except Exception as e:
            raise ValueError(f"Failed to load URDF: {e}")
    
    def _load_mjcf(self) -> None:
        """加载MJCF格式"""
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(self.asset_path)
            root = tree.getroot()
            
            # 基础MJCF解析实现
            # TODO: 完整的MJCF解析实现
            print(f"Loading MJCF format from {self.asset_path}")
            
        except Exception as e:
            raise ValueError(f"Failed to load MJCF: {e}")
    
    def _load_usd(self) -> None:
        """加载USD格式"""
        try:
            # USD解析实现
            # TODO: 完整的USD解析实现
            print(f"Loading USD format from {self.asset_path}")
            
        except Exception as e:
            raise ValueError(f"Failed to load USD: {e}")
    
    def _load_genesis(self) -> None:
        """加载Genesis JSON格式"""
        try:
            with open(self.asset_path, 'r', encoding='utf-8') as f:
                import json
                data = json.load(f)
                
            # Genesis JSON解析实现
            # TODO: 完整的Genesis JSON解析实现
            print(f"Loading Genesis JSON format from {self.asset_path}")
            
        except Exception as e:
            raise ValueError(f"Failed to load Genesis JSON: {e}")
    
    def get_summary(self) -> Dict:
        """获取资产摘要信息"""
        return {
            'path': str(self.asset_path),
            'format': self.format,
            'num_links': len(self.links),
            'num_joints': len(self.joints),
            'meta_data': self.meta_data,
            'physics_properties_count': len(self.physics_properties)
        }
