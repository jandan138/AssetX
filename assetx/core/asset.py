"""
Core asset representation and management (USD-inspired design)
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Union, Any, Callable
from abc import ABC, abstractmethod
from enum import Enum
import yaml
import weakref
from threading import RLock

from ..meta.manager import MetaManager


class AssetFormat(Enum):
    """支持的资产格式"""
    URDF = "urdf"
    MJCF = "mjcf"
    USD = "usd"
    GENESIS_JSON = "genesis_json"
    UNKNOWN = "unknown"


class PropertyType(Enum):
    """属性类型枚举"""
    ATTRIBUTE = "attribute"
    RELATIONSHIP = "relationship"


class VariabilityType(Enum):
    """属性变化性类型"""
    VARYING = "varying"        # 可以随时间变化
    UNIFORM = "uniform"        # 在一个时间段内保持一致
    CONFIG = "config"          # 配置时设定，运行时不变


class SdfPath:
    """USD风格的路径系统"""
    
    def __init__(self, path_string: str):
        self.path_string = path_string.strip()
        if not self.path_string.startswith('/'):
            self.path_string = '/' + self.path_string
    
    def __str__(self) -> str:
        return self.path_string
    
    def __repr__(self) -> str:
        return f"SdfPath('{self.path_string}')"
    
    def __eq__(self, other) -> bool:
        if isinstance(other, SdfPath):
            return self.path_string == other.path_string
        return False
    
    def __hash__(self) -> int:
        return hash(self.path_string)
    
    @property
    def is_root_prim_path(self) -> bool:
        """是否为根prim路径"""
        return self.path_string.count('/') == 1 and not self.path_string.endswith('.')
    
    @property
    def is_property_path(self) -> bool:
        """是否为属性路径"""
        return '.' in self.path_string
    
    def get_parent_path(self) -> 'SdfPath':
        """获取父路径"""
        if self.is_property_path:
            return SdfPath(self.path_string.split('.')[0])
        parts = self.path_string.rstrip('/').split('/')
        if len(parts) <= 2:  # ['', 'root'] or shorter
            return SdfPath('/')
        return SdfPath('/'.join(parts[:-1]))
    
    def get_name(self) -> str:
        """获取路径末尾的名称"""
        if self.is_property_path:
            return self.path_string.split('.')[-1]
        return self.path_string.split('/')[-1]
    
    def append_child(self, name: str) -> 'SdfPath':
        """添加子路径"""
        if self.path_string.endswith('/'):
            return SdfPath(f"{self.path_string}{name}")
        return SdfPath(f"{self.path_string}/{name}")
    
    def append_property(self, name: str) -> 'SdfPath':
        """添加属性路径"""
        return SdfPath(f"{self.path_string}.{name}")
    
    @staticmethod
    def absolute_root_path() -> 'SdfPath':
        """根路径"""
        return SdfPath('/')


class AssetProperty:
    """USD风格的属性基类"""
    
    def __init__(self, stage: 'AssetStage', path: SdfPath, property_type: PropertyType):
        self._stage_ref = weakref.ref(stage)
        self._path = path
        self._property_type = property_type
        self._metadata: Dict[str, Any] = {}
        self._custom = True
        self._variability = VariabilityType.VARYING
    
    @property
    def stage(self) -> 'AssetStage':
        stage = self._stage_ref()
        if stage is None:
            raise RuntimeError("Stage has been destroyed")
        return stage
    
    @property
    def path(self) -> SdfPath:
        return self._path
    
    @property
    def name(self) -> str:
        return self._path.get_name()
    
    @property
    def property_type(self) -> PropertyType:
        return self._property_type
    
    @property
    def is_custom(self) -> bool:
        return self._custom
    
    @property
    def variability(self) -> VariabilityType:
        return self._variability
    
    def is_valid(self) -> bool:
        """属性是否有效"""
        return self._stage_ref() is not None
    
    def get_metadata(self, key: str) -> Any:
        """获取元数据"""
        return self._metadata.get(key)
    
    def set_metadata(self, key: str, value: Any) -> bool:
        """设置元数据"""
        self._metadata[key] = value
        return True
    
    def has_metadata(self, key: str) -> bool:
        """是否有指定元数据"""
        return key in self._metadata
    
    def clear_metadata(self, key: str) -> bool:
        """清除元数据"""
        if key in self._metadata:
            del self._metadata[key]
            return True
        return False


class AssetAttribute(AssetProperty):
    """USD风格的属性类"""
    
    def __init__(self, stage: 'AssetStage', path: SdfPath, type_name: str = None):
        super().__init__(stage, path, PropertyType.ATTRIBUTE)
        self._type_name = type_name
        self._default_value: Any = None
        self._time_samples: Dict[float, Any] = {}
        self._connections: List[SdfPath] = []
    
    @property
    def type_name(self) -> str:
        return self._type_name
    
    def get(self, time: Optional[float] = None) -> Any:
        """获取属性值"""
        if time is not None and time in self._time_samples:
            return self._time_samples[time]
        return self._default_value
    
    def set(self, value: Any, time: Optional[float] = None) -> bool:
        """设置属性值"""
        if time is not None:
            self._time_samples[time] = value
        else:
            self._default_value = value
        return True
    
    def get_time_samples(self) -> Dict[float, Any]:
        """获取所有时间采样"""
        return self._time_samples.copy()
    
    def has_authored_value(self) -> bool:
        """是否有作者设定的值"""
        return self._default_value is not None or bool(self._time_samples)
    
    def add_connection(self, source_path: SdfPath) -> bool:
        """添加连接"""
        if source_path not in self._connections:
            self._connections.append(source_path)
            return True
        return False
    
    def remove_connection(self, source_path: SdfPath) -> bool:
        """移除连接"""
        if source_path in self._connections:
            self._connections.remove(source_path)
            return True
        return False
    
    def get_connections(self) -> List[SdfPath]:
        """获取所有连接"""
        return self._connections.copy()
    
    def has_connections(self) -> bool:
        """是否有连接"""
        return bool(self._connections)


class AssetRelationship(AssetProperty):
    """USD风格的关系类"""
    
    def __init__(self, stage: 'AssetStage', path: SdfPath):
        super().__init__(stage, path, PropertyType.RELATIONSHIP)
        self._targets: List[SdfPath] = []
        self._forwarded_targets: List[SdfPath] = []
    
    def add_target(self, target_path: SdfPath) -> bool:
        """添加目标"""
        if target_path not in self._targets:
            self._targets.append(target_path)
            return True
        return False
    
    def remove_target(self, target_path: SdfPath) -> bool:
        """移除目标"""
        if target_path in self._targets:
            self._targets.remove(target_path)
            return True
        return False
    
    def set_targets(self, target_paths: List[SdfPath]) -> bool:
        """设置所有目标"""
        self._targets = target_paths.copy()
        return True
    
    def get_targets(self) -> List[SdfPath]:
        """获取所有目标"""
        return self._targets.copy()
    
    def get_forwarded_targets(self) -> List[SdfPath]:
        """获取转发的目标"""
        # 这里可以实现更复杂的目标解析逻辑
        return self._targets.copy()
    
    def has_authored_targets(self) -> bool:
        """是否有作者设定的目标"""
        return bool(self._targets)


class AssetPrim:
    """USD风格的Prim类"""
    
    def __init__(self, stage: 'AssetStage', path: SdfPath):
        self._stage_ref = weakref.ref(stage)
        self._path = path
        self._type_name: Optional[str] = None
        self._active = True
        self._instanceable = False
        self._children: Dict[str, 'AssetPrim'] = {}
        self._properties: Dict[str, AssetProperty] = {}
        self._metadata: Dict[str, Any] = {}
        self._applied_schemas: List[str] = []
    
    @property
    def stage(self) -> 'AssetStage':
        stage = self._stage_ref()
        if stage is None:
            raise RuntimeError("Stage has been destroyed")
        return stage
    
    @property
    def path(self) -> SdfPath:
        return self._path
    
    @property
    def name(self) -> str:
        return self._path.get_name()
    
    @property
    def type_name(self) -> Optional[str]:
        return self._type_name
    
    def set_type_name(self, type_name: str) -> bool:
        """设置类型名"""
        self._type_name = type_name
        return True
    
    def is_valid(self) -> bool:
        """Prim是否有效"""
        return self._stage_ref() is not None
    
    def is_active(self) -> bool:
        """是否激活"""
        return self._active
    
    def set_active(self, active: bool) -> bool:
        """设置激活状态"""
        self._active = active
        return True
    
    def is_instanceable(self) -> bool:
        """是否可实例化"""
        return self._instanceable
    
    def set_instanceable(self, instanceable: bool) -> bool:
        """设置可实例化状态"""
        self._instanceable = instanceable
        return True
    
    # 子Prim管理
    def get_children(self) -> List['AssetPrim']:
        """获取所有子Prim"""
        return list(self._children.values())
    
    def get_child(self, name: str) -> Optional['AssetPrim']:
        """获取指定名称的子Prim"""
        return self._children.get(name)
    
    def get_child_names(self) -> List[str]:
        """获取所有子Prim名称"""
        return list(self._children.keys())
    
    def has_child(self, name: str) -> bool:
        """是否有指定子Prim"""
        return name in self._children
    
    # 属性管理
    def create_attribute(self, name: str, type_name: str, 
                        custom: bool = True, 
                        variability: VariabilityType = VariabilityType.VARYING) -> AssetAttribute:
        """创建属性"""
        attr_path = self._path.append_property(name)
        attr = AssetAttribute(self.stage, attr_path, type_name)
        attr._custom = custom
        attr._variability = variability
        self._properties[name] = attr
        return attr
    
    def get_attribute(self, name: str) -> Optional[AssetAttribute]:
        """获取属性"""
        prop = self._properties.get(name)
        if prop and isinstance(prop, AssetAttribute):
            return prop
        return None
    
    def has_attribute(self, name: str) -> bool:
        """是否有指定属性"""
        return (name in self._properties and 
                isinstance(self._properties[name], AssetAttribute))
    
    def get_attributes(self) -> List[AssetAttribute]:
        """获取所有属性"""
        return [prop for prop in self._properties.values() 
                if isinstance(prop, AssetAttribute)]
    
    # 关系管理
    def create_relationship(self, name: str, custom: bool = True) -> AssetRelationship:
        """创建关系"""
        rel_path = self._path.append_property(name)
        rel = AssetRelationship(self.stage, rel_path)
        rel._custom = custom
        self._properties[name] = rel
        return rel
    
    def get_relationship(self, name: str) -> Optional[AssetRelationship]:
        """获取关系"""
        prop = self._properties.get(name)
        if prop and isinstance(prop, AssetRelationship):
            return prop
        return None
    
    def has_relationship(self, name: str) -> bool:
        """是否有指定关系"""
        return (name in self._properties and 
                isinstance(self._properties[name], AssetRelationship))
    
    def get_relationships(self) -> List[AssetRelationship]:
        """获取所有关系"""
        return [prop for prop in self._properties.values() 
                if isinstance(prop, AssetRelationship)]
    
    def get_properties(self) -> List[AssetProperty]:
        """获取所有属性和关系"""
        return list(self._properties.values())
    
    def get_property(self, name: str) -> Optional[AssetProperty]:
        """获取属性或关系"""
        return self._properties.get(name)
    
    def has_property(self, name: str) -> bool:
        """是否有指定属性或关system"""
        return name in self._properties
    
    def get_property_names(self) -> List[str]:
        """获取所有属性和关系名称"""
        return list(self._properties.keys())
    
    # 元数据管理
    def get_metadata(self, key: str) -> Any:
        """获取元数据"""
        return self._metadata.get(key)
    
    def set_metadata(self, key: str, value: Any) -> bool:
        """设置元数据"""
        self._metadata[key] = value
        return True
    
    def has_metadata(self, key: str) -> bool:
        """是否有指定元数据"""
        return key in self._metadata
    
    def clear_metadata(self, key: str) -> bool:
        """清除元数据"""
        if key in self._metadata:
            del self._metadata[key]
            return True
        return False
    
    # Schema管理
    def apply_api_schema(self, schema_name: str) -> bool:
        """应用API Schema"""
        if schema_name not in self._applied_schemas:
            self._applied_schemas.append(schema_name)
            return True
        return False
    
    def remove_api_schema(self, schema_name: str) -> bool:
        """移除API Schema"""
        if schema_name in self._applied_schemas:
            self._applied_schemas.remove(schema_name)
            return True
        return False
    
    def has_api_schema(self, schema_name: str) -> bool:
        """是否应用了指定Schema"""
        return schema_name in self._applied_schemas
    
    def get_applied_schemas(self) -> List[str]:
        """获取所有应用的Schema"""
        return self._applied_schemas.copy()


class AssetStage:
    """USD风格的Stage类 - 资产的主要容器"""
    
    def __init__(self, identifier: str = None):
        self._identifier = identifier or "anonymous"
        self._root_prim: Optional[AssetPrim] = None
        self._prims: Dict[SdfPath, AssetPrim] = {}
        self._metadata: Dict[str, Any] = {}
        self._default_prim_path: Optional[SdfPath] = None
        self._lock = RLock()
        
        # 创建伪根
        self._pseudo_root = AssetPrim(self, SdfPath('/'))
        self._prims[SdfPath('/')] = self._pseudo_root
    
    @property
    def identifier(self) -> str:
        return self._identifier
    
    def get_pseudo_root(self) -> AssetPrim:
        """获取伪根prim"""
        return self._pseudo_root
    
    def get_default_prim(self) -> Optional[AssetPrim]:
        """获取默认prim"""
        if self._default_prim_path:
            return self.get_prim_at_path(self._default_prim_path)
        return None
    
    def set_default_prim(self, prim: AssetPrim) -> bool:
        """设置默认prim"""
        self._default_prim_path = prim.path
        return True
    
    def has_default_prim(self) -> bool:
        """是否有默认prim"""
        return self._default_prim_path is not None
    
    # Prim管理
    def get_prim_at_path(self, path: SdfPath) -> Optional[AssetPrim]:
        """根据路径获取prim"""
        return self._prims.get(path)
    
    def define_prim(self, path: SdfPath, type_name: str = None) -> AssetPrim:
        """定义prim"""
        with self._lock:
            # 确保父路径存在
            parent_path = path.get_parent_path()
            if parent_path.path_string != '/' and parent_path not in self._prims:
                self.define_prim(parent_path)
            
            if path in self._prims:
                prim = self._prims[path]
                if type_name:
                    prim.set_type_name(type_name)
                return prim
            
            prim = AssetPrim(self, path)
            if type_name:
                prim.set_type_name(type_name)
            
            self._prims[path] = prim
            
            # 添加到父prim的子列表
            if parent_path in self._prims:
                parent_prim = self._prims[parent_path]
                parent_prim._children[path.get_name()] = prim
            
            return prim
    
    def override_prim(self, path: SdfPath) -> AssetPrim:
        """覆盖prim（如果不存在则创建）"""
        return self.define_prim(path)
    
    def remove_prim(self, path: SdfPath) -> bool:
        """移除prim"""
        with self._lock:
            if path not in self._prims:
                return False
            
            prim = self._prims[path]
            
            # 递归移除所有子prim
            for child in prim.get_children():
                self.remove_prim(child.path)
            
            # 从父prim中移除
            parent_path = path.get_parent_path()
            if parent_path in self._prims:
                parent_prim = self._prims[parent_path]
                if path.get_name() in parent_prim._children:
                    del parent_prim._children[path.get_name()]
            
            # 从stage中移除
            del self._prims[path]
            return True
    
    def traverse(self, predicate: Callable[[AssetPrim], bool] = None) -> List[AssetPrim]:
        """遍历所有prim"""
        result = []
        
        def _traverse(prim: AssetPrim):
            if predicate is None or predicate(prim):
                result.append(prim)
            for child in prim.get_children():
                _traverse(child)
        
        for child in self._pseudo_root.get_children():
            _traverse(child)
        
        return result
    
    # 元数据管理
    def get_metadata(self, key: str) -> Any:
        """获取stage元数据"""
        return self._metadata.get(key)
    
    def set_metadata(self, key: str, value: Any) -> bool:
        """设置stage元数据"""
        self._metadata[key] = value
        return True
    
    def has_metadata(self, key: str) -> bool:
        """是否有指定元数据"""
        return key in self._metadata
    
    def clear_metadata(self, key: str) -> bool:
        """清除元数据"""
        if key in self._metadata:
            del self._metadata[key]
            return True
        return False
    
    # 静态方法
    @staticmethod
    def create_new(identifier: str) -> 'AssetStage':
        """创建新的stage"""
        return AssetStage(identifier)
    
    @staticmethod
    def create_in_memory() -> 'AssetStage':
        """在内存中创建stage"""
        return AssetStage()


class Asset:
    """统一资产表示类 - USD风格设计"""
    
    def __init__(self, asset_path: Union[str, Path]):
        self.asset_path = Path(asset_path)
        self.format = self._detect_format()
        self.meta_manager = MetaManager(self.asset_path.parent)
        
        # USD风格的核心组件
        self._stage: Optional[AssetStage] = None
        self._is_loaded = False
    
    # ============ USD风格的Stage管理接口 ============
    
    def get_stage(self) -> AssetStage:
        """获取资产的Stage"""
        if not self._is_loaded:
            self.load()
        return self._stage
    
    def create_stage(self) -> AssetStage:
        """创建新的Stage"""
        self._stage = AssetStage.create_new(str(self.asset_path))
        self._is_loaded = True
        return self._stage
    
    # ============ USD风格的Prim管理接口 ============
    
    def get_default_prim(self) -> Optional[AssetPrim]:
        """获取默认prim"""
        if not self._is_loaded:
            self.load()
        return self._stage.get_default_prim() if self._stage else None
    
    def set_default_prim(self, prim: AssetPrim) -> bool:
        """设置默认prim"""
        if not self._is_loaded:
            self.load()
        return self._stage.set_default_prim(prim) if self._stage else False
    
    def get_prim_at_path(self, path: Union[str, SdfPath]) -> Optional[AssetPrim]:
        """根据路径获取prim"""
        if not self._is_loaded:
            self.load()
        if isinstance(path, str):
            path = SdfPath(path)
        return self._stage.get_prim_at_path(path) if self._stage else None
    
    def define_prim(self, path: Union[str, SdfPath], type_name: str = None) -> AssetPrim:
        """定义prim"""
        if not self._is_loaded:
            self.load()
        if isinstance(path, str):
            path = SdfPath(path)
        if not self._stage:
            self.create_stage()
        return self._stage.define_prim(path, type_name)
    
    def override_prim(self, path: Union[str, SdfPath]) -> AssetPrim:
        """覆盖prim"""
        if not self._is_loaded:
            self.load()
        if isinstance(path, str):
            path = SdfPath(path)
        if not self._stage:
            self.create_stage()
        return self._stage.override_prim(path)
    
    def remove_prim(self, path: Union[str, SdfPath]) -> bool:
        """移除prim"""
        if not self._is_loaded:
            self.load()
        if isinstance(path, str):
            path = SdfPath(path)
        return self._stage.remove_prim(path) if self._stage else False
    
    def traverse(self, predicate: Callable[[AssetPrim], bool] = None) -> List[AssetPrim]:
        """遍历所有prim"""
        if not self._is_loaded:
            self.load()
        return self._stage.traverse(predicate) if self._stage else []
    
    # ============ USD风格的便利接口 ============
    
    def get_pseudo_root(self) -> Optional[AssetPrim]:
        """获取伪根prim"""
        if not self._is_loaded:
            self.load()
        return self._stage.get_pseudo_root() if self._stage else None
    
    def get_root_prims(self) -> List[AssetPrim]:
        """获取所有根prim"""
        pseudo_root = self.get_pseudo_root()
        return pseudo_root.get_children() if pseudo_root else []
    
    def get_all_prims(self) -> List[AssetPrim]:
        """获取所有prim（除伪根外）"""
        return self.traverse(lambda p: not p.path.path_string == '/')
    
    # ============ 机器人特定的Schema应用接口 ============
    
    def create_robot_prim(self, path: Union[str, SdfPath], name: str = None) -> AssetPrim:
        """创建机器人prim"""
        prim = self.define_prim(path, "Robot")
        prim.apply_api_schema("RobotAPI")
        if name:
            prim.set_metadata("displayName", name)
        return prim
    
    def create_link_prim(self, path: Union[str, SdfPath], 
                        mass: float = None,
                        inertia: List[float] = None) -> AssetPrim:
        """创建链接prim"""
        prim = self.define_prim(path, "Link")
        prim.apply_api_schema("LinkAPI")
        prim.apply_api_schema("PhysicsAPI")
        
        if mass is not None:
            mass_attr = prim.create_attribute("physics:mass", "float")
            mass_attr.set(mass)
        
        if inertia is not None:
            inertia_attr = prim.create_attribute("physics:inertia", "matrix3d")
            inertia_attr.set(inertia)
        
        return prim
    
    def create_joint_prim(self, path: Union[str, SdfPath],
                         joint_type: str,
                         parent_link: str,
                         child_link: str,
                         axis: List[float] = None) -> AssetPrim:
        """创建关节prim"""
        prim = self.define_prim(path, "Joint")
        prim.apply_api_schema("JointAPI")
        
        # 设置关节类型
        type_attr = prim.create_attribute("joint:type", "token")
        type_attr.set(joint_type)
        
        # 设置父子链接关系
        parent_rel = prim.create_relationship("joint:parentLink")
        parent_rel.add_target(SdfPath(parent_link))
        
        child_rel = prim.create_relationship("joint:childLink")
        child_rel.add_target(SdfPath(child_link))
        
        # 设置轴向
        if axis is not None:
            axis_attr = prim.create_attribute("joint:axis", "vector3f")
            axis_attr.set(axis)
        
        return prim
    
    def create_geometry_prim(self, path: Union[str, SdfPath],
                            mesh_path: str = None,
                            collision_mesh_path: str = None) -> AssetPrim:
        """创建几何prim"""
        prim = self.define_prim(path, "Geometry")
        prim.apply_api_schema("GeometryAPI")
        
        if mesh_path:
            mesh_attr = prim.create_attribute("geometry:mesh", "asset")
            mesh_attr.set(mesh_path)
        
        if collision_mesh_path:
            collision_attr = prim.create_attribute("geometry:collisionMesh", "asset")
            collision_attr.set(collision_mesh_path)
        
        return prim
    
    # ============ 元数据管理接口 ============
    
    def get_metadata(self, key: str) -> Any:
        """获取stage级别元数据"""
        if not self._is_loaded:
            self.load()
        return self._stage.get_metadata(key) if self._stage else None
    
    def set_metadata(self, key: str, value: Any) -> bool:
        """设置stage级别元数据"""
        if not self._is_loaded:
            self.load()
        if not self._stage:
            self.create_stage()
        return self._stage.set_metadata(key, value)
    
    def has_metadata(self, key: str) -> bool:
        """是否有指定元数据"""
        if not self._is_loaded:
            self.load()
        return self._stage.has_metadata(key) if self._stage else False
    
    # ============ 查询和搜索接口 ============
    
    def find_prims_by_type(self, type_name: str) -> List[AssetPrim]:
        """根据类型查找prim"""
        return self.traverse(lambda p: p.type_name == type_name)
    
    def find_prims_by_schema(self, schema_name: str) -> List[AssetPrim]:
        """根据Schema查找prim"""
        return self.traverse(lambda p: p.has_api_schema(schema_name))
    
    def find_prims_by_name_pattern(self, pattern: str) -> List[AssetPrim]:
        """根据名称模式查找prim"""
        import re
        regex = re.compile(pattern)
        return self.traverse(lambda p: regex.search(p.name))
    
    def get_all_links(self) -> List[AssetPrim]:
        """获取所有链接"""
        return self.find_prims_by_type("Link")
    
    def get_all_joints(self) -> List[AssetPrim]:
        """获取所有关节"""
        return self.find_prims_by_type("Joint")
    
    def get_all_geometries(self) -> List[AssetPrim]:
        """获取所有几何体"""
        return self.find_prims_by_type("Geometry")
    
    
    # ============ 保持现有接口的兼容性 ============
    
    def _detect_format(self) -> AssetFormat:
        """检测资产格式"""
        if not self.asset_path.exists():
            return AssetFormat.UNKNOWN
        
        suffix = self.asset_path.suffix.lower()
        if suffix == '.urdf':
            return AssetFormat.URDF
        elif suffix in ['.xml', '.mjcf']:
            return AssetFormat.MJCF
        elif suffix in ['.usd', '.usda', '.usdc']:
            return AssetFormat.USD
        elif suffix == '.json':
            # 简单检查是否是Genesis格式
            try:
                with open(self.asset_path, 'r') as f:
                    content = f.read(100)  # 只读前100个字符
                    if 'genesis' in content.lower():
                        return AssetFormat.GENESIS_JSON
            except:
                pass
        
        return AssetFormat.UNKNOWN
    
    def load(self) -> bool:
        """从文件加载资产"""
        if self._is_loaded:
            return True
        
        if not self.asset_path.exists():
            return False
        
        # 创建新的stage
        self._stage = AssetStage.create_new(str(self.asset_path))
        
        try:
            if self.format == AssetFormat.URDF:
                success = self._load_urdf()
            elif self.format == AssetFormat.MJCF:
                success = self._load_mjcf()
            elif self.format == AssetFormat.USD:
                success = self._load_usd()
            elif self.format == AssetFormat.GENESIS_JSON:
                success = self._load_genesis_json()
            else:
                success = False
            
            if success:
                self._is_loaded = True
                return True
            else:
                self._stage = None
                return False
                
        except Exception as e:
            print(f"Error loading asset: {e}")
            self._stage = None
            return False
    
    def _load_urdf(self) -> bool:
        """加载URDF格式"""
        try:
            import xml.etree.ElementTree as ET
            
            tree = ET.parse(self.asset_path)
            root = tree.getroot()
            
            if root.tag != 'robot':
                return False
            
            robot_name = root.get('name', 'robot')
            
            # 创建机器人根prim
            robot_prim = self.create_robot_prim(f"/{robot_name}", robot_name)
            self._stage.set_default_prim(robot_prim)
            
            # 解析链接
            for link_elem in root.findall('link'):
                link_name = link_elem.get('name')
                link_path = f"/{robot_name}/{link_name}"
                
                # 获取质量和惯性信息
                inertial_elem = link_elem.find('inertial')
                mass = None
                inertia = None
                
                if inertial_elem is not None:
                    mass_elem = inertial_elem.find('mass')
                    if mass_elem is not None:
                        mass = float(mass_elem.get('value', 0))
                    
                    inertia_elem = inertial_elem.find('inertia')
                    if inertia_elem is not None:
                        inertia = [
                            float(inertia_elem.get('ixx', 0)),
                            float(inertia_elem.get('iyy', 0)),
                            float(inertia_elem.get('izz', 0)),
                            float(inertia_elem.get('ixy', 0)),
                            float(inertia_elem.get('ixz', 0)),
                            float(inertia_elem.get('iyz', 0))
                        ]
                
                link_prim = self.create_link_prim(link_path, mass, inertia)
                
                # 解析几何体
                for visual_elem in link_elem.findall('visual'):
                    geom_elem = visual_elem.find('geometry')
                    if geom_elem is not None:
                        mesh_elem = geom_elem.find('mesh')
                        if mesh_elem is not None:
                            mesh_filename = mesh_elem.get('filename')
                            if mesh_filename:
                                geom_path = f"{link_path}/visual_geometry"
                                self.create_geometry_prim(geom_path, mesh_filename)
                
                for collision_elem in link_elem.findall('collision'):
                    geom_elem = collision_elem.find('geometry')
                    if geom_elem is not None:
                        mesh_elem = geom_elem.find('mesh')
                        if mesh_elem is not None:
                            mesh_filename = mesh_elem.get('filename')
                            if mesh_filename:
                                geom_path = f"{link_path}/collision_geometry"
                                self.create_geometry_prim(geom_path, None, mesh_filename)
            
            # 解析关节
            for joint_elem in root.findall('joint'):
                joint_name = joint_elem.get('name')
                joint_type = joint_elem.get('type')
                joint_path = f"/{robot_name}/{joint_name}"
                
                parent_elem = joint_elem.find('parent')
                child_elem = joint_elem.find('child')
                
                if parent_elem is not None and child_elem is not None:
                    parent_link = f"/{robot_name}/{parent_elem.get('link')}"
                    child_link = f"/{robot_name}/{child_elem.get('link')}"
                    
                    # 获取轴向信息
                    axis = None
                    axis_elem = joint_elem.find('axis')
                    if axis_elem is not None:
                        xyz = axis_elem.get('xyz', '0 0 0').split()
                        axis = [float(x) for x in xyz]
                    
                    self.create_joint_prim(joint_path, joint_type, parent_link, child_link, axis)
            
            return True
            
        except Exception as e:
            print(f"Error parsing URDF: {e}")
            return False
    
    def _load_mjcf(self) -> bool:
        """加载MJCF格式"""
        # TODO: 实现MJCF加载逻辑
        print("MJCF loading not yet implemented")
        return False
    
    def _load_usd(self) -> bool:
        """加载USD格式"""
        # TODO: 实现USD加载逻辑
        print("USD loading not yet implemented")
        return False
    
    def _load_genesis_json(self) -> bool:
        """加载Genesis JSON格式"""
        # TODO: 实现Genesis JSON加载逻辑
        print("Genesis JSON loading not yet implemented")
        return False
    
    def save(self, output_path: Optional[Union[str, Path]] = None) -> bool:
        """保存资产"""
        if not self._is_loaded or not self._stage:
            return False
        
        save_path = Path(output_path) if output_path else self.asset_path
        
        try:
            if self.format == AssetFormat.URDF:
                return self._save_urdf(save_path)
            elif self.format == AssetFormat.MJCF:
                return self._save_mjcf(save_path)
            elif self.format == AssetFormat.USD:
                return self._save_usd(save_path)
            elif self.format == AssetFormat.GENESIS_JSON:
                return self._save_genesis_json(save_path)
            else:
                return False
        except Exception as e:
            print(f"Error saving asset: {e}")
            return False
    
    def _save_urdf(self, output_path: Path) -> bool:
        """保存为URDF格式"""
        try:
            import xml.etree.ElementTree as ET
            
            default_prim = self.get_default_prim()
            if not default_prim:
                return False
            
            # 创建XML根元素
            root = ET.Element('robot')
            root.set('name', default_prim.name)
            
            # 获取所有链接
            links = self.get_all_links()
            for link_prim in links:
                link_elem = ET.SubElement(root, 'link')
                link_elem.set('name', link_prim.name)
                
                # 添加质量和惯性信息
                mass_attr = link_prim.get_attribute('physics:mass')
                inertia_attr = link_prim.get_attribute('physics:inertia')
                
                if mass_attr or inertia_attr:
                    inertial_elem = ET.SubElement(link_elem, 'inertial')
                    
                    if mass_attr and mass_attr.has_authored_value():
                        mass_elem = ET.SubElement(inertial_elem, 'mass')
                        mass_elem.set('value', str(mass_attr.get()))
                    
                    if inertia_attr and inertia_attr.has_authored_value():
                        inertia_values = inertia_attr.get()
                        if inertia_values and len(inertia_values) >= 6:
                            inertia_elem = ET.SubElement(inertial_elem, 'inertia')
                            inertia_elem.set('ixx', str(inertia_values[0]))
                            inertia_elem.set('iyy', str(inertia_values[1]))
                            inertia_elem.set('izz', str(inertia_values[2]))
                            inertia_elem.set('ixy', str(inertia_values[3]))
                            inertia_elem.set('ixz', str(inertia_values[4]))
                            inertia_elem.set('iyz', str(inertia_values[5]))
                
                # 查找相关几何体
                for child in link_prim.get_children():
                    if child.type_name == 'Geometry':
                        mesh_attr = child.get_attribute('geometry:mesh')
                        collision_attr = child.get_attribute('geometry:collisionMesh')
                        
                        if mesh_attr and mesh_attr.has_authored_value():
                            visual_elem = ET.SubElement(link_elem, 'visual')
                            geom_elem = ET.SubElement(visual_elem, 'geometry')
                            mesh_elem = ET.SubElement(geom_elem, 'mesh')
                            mesh_elem.set('filename', str(mesh_attr.get()))
                        
                        if collision_attr and collision_attr.has_authored_value():
                            collision_elem = ET.SubElement(link_elem, 'collision')
                            geom_elem = ET.SubElement(collision_elem, 'geometry')
                            mesh_elem = ET.SubElement(geom_elem, 'mesh')
                            mesh_elem.set('filename', str(collision_attr.get()))
            
            # 获取所有关节
            joints = self.get_all_joints()
            for joint_prim in joints:
                joint_elem = ET.SubElement(root, 'joint')
                joint_elem.set('name', joint_prim.name)
                
                # 获取关节类型
                type_attr = joint_prim.get_attribute('joint:type')
                if type_attr and type_attr.has_authored_value():
                    joint_elem.set('type', str(type_attr.get()))
                
                # 获取父子链接关系
                parent_rel = joint_prim.get_relationship('joint:parentLink')
                child_rel = joint_prim.get_relationship('joint:childLink')
                
                if parent_rel and parent_rel.has_authored_targets():
                    parent_targets = parent_rel.get_targets()
                    if parent_targets:
                        parent_elem = ET.SubElement(joint_elem, 'parent')
                        parent_elem.set('link', parent_targets[0].get_name())
                
                if child_rel and child_rel.has_authored_targets():
                    child_targets = child_rel.get_targets()
                    if child_targets:
                        child_elem = ET.SubElement(joint_elem, 'child')
                        child_elem.set('link', child_targets[0].get_name())
                
                # 获取轴向
                axis_attr = joint_prim.get_attribute('joint:axis')
                if axis_attr and axis_attr.has_authored_value():
                    axis_values = axis_attr.get()
                    if axis_values and len(axis_values) >= 3:
                        axis_elem = ET.SubElement(joint_elem, 'axis')
                        axis_elem.set('xyz', f"{axis_values[0]} {axis_values[1]} {axis_values[2]}")
            
            # 写入文件
            tree = ET.ElementTree(root)
            ET.indent(tree, space="  ", level=0)
            tree.write(output_path, encoding='utf-8', xml_declaration=True)
            
            return True
            
        except Exception as e:
            print(f"Error saving URDF: {e}")
            return False
    
    def _save_mjcf(self, output_path: Path) -> bool:
        """保存为MJCF格式"""
        # TODO: 实现MJCF保存逻辑
        print("MJCF saving not yet implemented")
        return False
    
    def _save_usd(self, output_path: Path) -> bool:
        """保存为USD格式"""
        # TODO: 实现USD保存逻辑
        print("USD saving not yet implemented")
        return False
    
    def _save_genesis_json(self, output_path: Path) -> bool:
        """保存为Genesis JSON格式"""
        # TODO: 实现Genesis JSON保存逻辑
        print("Genesis JSON saving not yet implemented")
        return False
    
    # ============ 便利方法和向后兼容性 ============
    
    @property
    def is_loaded(self) -> bool:
        """是否已加载"""
        return self._is_loaded
    
    def get_robot_name(self) -> Optional[str]:
        """获取机器人名称"""
        default_prim = self.get_default_prim()
        return default_prim.name if default_prim else None
    
    def get_link_names(self) -> List[str]:
        """获取所有链接名称"""
        return [prim.name for prim in self.get_all_links()]
    
    def get_joint_names(self) -> List[str]:
        """获取所有关节名称"""
        return [prim.name for prim in self.get_all_joints()]
    
    def get_link_info(self, link_name: str) -> Optional[Dict[str, Any]]:
        """获取链接信息"""
        link_prim = self.get_prim_at_path(f"/{self.get_robot_name()}/{link_name}")
        if not link_prim:
            return None
        
        info = {
            'name': link_name,
            'path': str(link_prim.path),
            'type': link_prim.type_name,
            'schemas': link_prim.get_applied_schemas(),
            'properties': {}
        }
        
        # 收集属性信息
        for attr in link_prim.get_attributes():
            if attr.has_authored_value():
                info['properties'][attr.name] = attr.get()
        
        return info
    
    def get_joint_info(self, joint_name: str) -> Optional[Dict[str, Any]]:
        """获取关节信息"""
        joint_prim = self.get_prim_at_path(f"/{self.get_robot_name()}/{joint_name}")
        if not joint_prim:
            return None
        
        info = {
            'name': joint_name,
            'path': str(joint_prim.path),
            'type': joint_prim.type_name,
            'schemas': joint_prim.get_applied_schemas(),
            'properties': {},
            'relationships': {}
        }
        
        # 收集属性信息
        for attr in joint_prim.get_attributes():
            if attr.has_authored_value():
                info['properties'][attr.name] = attr.get()
        
        # 收集关系信息
        for rel in joint_prim.get_relationships():
            if rel.has_authored_targets():
                info['relationships'][rel.name] = [str(target) for target in rel.get_targets()]
        
        return info
    
    def __repr__(self) -> str:
        format_str = self.format.value if hasattr(self.format, 'value') else str(self.format)
        return f"Asset(path='{self.asset_path}', format='{format_str}', loaded={self._is_loaded})"

