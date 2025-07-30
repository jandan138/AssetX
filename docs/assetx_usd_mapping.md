# AssetX与USD对应关系详解

## 概述

本文档详细说明AssetX系统如何将机器人仿真格式（如URDF、MJCF）映射到USD架构，以及两者之间的精确对应关系。

## 整体架构对应

### USD原始架构
```
USD Framework
├── Stage (场景容器)
├── Prim (场景对象)
├── Property (属性系统)
│   ├── Attribute (数据属性)
│   └── Relationship (关系引用)
├── SdfPath (层次化路径)
└── Schema (类型系统)
```

### AssetX对应实现
```
AssetX Core
├── AssetStage (对应USD Stage)
├── AssetPrim (对应USD Prim)
├── AssetProperty (对应USD Property)
│   ├── AssetAttribute (对应USD Attribute)
│   └── AssetRelationship (对应USD Relationship)
├── SdfPath (直接对应USD SdfPath)
└── Schema系统 (通过API Schema实现)
```

## 详细对应关系

### 1. Stage层级对应

#### USD Stage概念
```cpp
// USD原始接口 (C++)
class UsdStage {
    UsdPrim GetDefaultPrim();
    UsdPrim DefinePrim(const SdfPath& path, const TfToken& typeName);
    bool RemovePrim(const SdfPath& path);
    UsdPrimRange Traverse();
};
```

#### AssetX实现对应
```python
class AssetStage:
    """直接对应USD Stage的Python实现"""
    
    def get_default_prim(self) -> Optional[AssetPrim]:
        """对应UsdStage::GetDefaultPrim()"""
        return self._default_prim
    
    def define_prim(self, path: SdfPath, type_name: str = None) -> AssetPrim:
        """对应UsdStage::DefinePrim()"""
        prim = AssetPrim(self, path, type_name)
        self._prims[path] = prim
        return prim
    
    def remove_prim(self, path: SdfPath) -> bool:
        """对应UsdStage::RemovePrim()"""
        return self._prims.pop(path, None) is not None
    
    def traverse(self) -> List[AssetPrim]:
        """对应UsdStage::Traverse()"""
        return list(self._prims.values())
```

**功能对应表**:
| USD Stage方法 | AssetX实现 | 说明 |
|--------------|------------|------|
| `GetDefaultPrim()` | `get_default_prim()` | 获取默认Prim |
| `DefinePrim()` | `define_prim()` | 定义新Prim |
| `RemovePrim()` | `remove_prim()` | 移除Prim |
| `Traverse()` | `traverse()` | 遍历所有Prim |
| `GetMetadata()` | `get_metadata()` | 获取元数据 |
| `SetMetadata()` | `set_metadata()` | 设置元数据 |

### 2. Prim层级对应

#### USD Prim概念
```cpp
// USD原始接口
class UsdPrim {
    SdfPath GetPath();
    TfToken GetTypeName();
    UsdPrim GetParent();
    UsdPrimSiblingRange GetChildren();
    UsdAttribute CreateAttribute(const TfToken& name, const SdfValueTypeName& typeName);
    UsdRelationship CreateRelationship(const TfToken& name);
};
```

#### AssetX实现对应
```python
class AssetPrim:
    """直接对应USD Prim的Python实现"""
    
    @property
    def path(self) -> SdfPath:
        """对应UsdPrim::GetPath()"""
        return self._path
    
    @property  
    def type_name(self) -> str:
        """对应UsdPrim::GetTypeName()"""
        return self._type_name
    
    def get_parent(self) -> Optional['AssetPrim']:
        """对应UsdPrim::GetParent()"""
        parent_path = self._path.get_parent_path()
        return self._stage.get_prim_at_path(parent_path)
    
    def get_children(self) -> List['AssetPrim']:
        """对应UsdPrim::GetChildren()"""
        return list(self._children.values())
    
    def create_attribute(self, name: str, type_name: str) -> AssetAttribute:
        """对应UsdPrim::CreateAttribute()"""
        attr = AssetAttribute(self, name, type_name)
        self._properties[name] = attr
        return attr
    
    def create_relationship(self, name: str) -> AssetRelationship:
        """对应UsdPrim::CreateRelationship()"""
        rel = AssetRelationship(self, name)
        self._properties[name] = rel
        return rel
```

**机器人特定类型映射**:
| 机器人概念 | USD类型 | AssetX类型 | 说明 |
|-----------|---------|------------|------|
| 机器人整体 | `Robot` | `AssetPrim(type="Robot")` | 机器人根节点 |
| 刚体链接 | `Link` | `AssetPrim(type="Link")` | 机器人链接 |
| 关节连接 | `Joint` | `AssetPrim(type="Joint")` | 机器人关节 |
| 传感器 | `Sensor` | `AssetPrim(type="Sensor")` | 传感器组件 |
| 执行器 | `Actuator` | `AssetPrim(type="Actuator")` | 执行器组件 |

### 3. Property层级对应

#### USD Property系统
```cpp
// USD Attribute
class UsdAttribute : public UsdProperty {
    bool Set(const VtValue& value, UsdTimeCode time = UsdTimeCode::Default());
    bool Get(VtValue* value, UsdTimeCode time = UsdTimeCode::Default());
    SdfValueTypeName GetTypeName();
};

// USD Relationship  
class UsdRelationship : public UsdProperty {
    bool SetTargets(const SdfPathVector& targets);
    bool GetTargets(SdfPathVector* targets);
};
```

#### AssetX实现对应
```python
class AssetAttribute(AssetProperty):
    """对应USD Attribute"""
    
    def set(self, value: Any, time: Optional[float] = None) -> bool:
        """对应UsdAttribute::Set()"""
        if time is None:
            self._default_value = value
        else:
            self._time_samples[time] = value
        return True
    
    def get(self, time: Optional[float] = None) -> Any:
        """对应UsdAttribute::Get()"""
        if time is None:
            return self._default_value
        return self._time_samples.get(time, self._default_value)

class AssetRelationship(AssetProperty):
    """对应USD Relationship"""
    
    def set_targets(self, targets: List[SdfPath]) -> bool:
        """对应UsdRelationship::SetTargets()"""
        self._targets = targets.copy()
        return True
    
    def get_targets(self) -> List[SdfPath]:
        """对应UsdRelationship::GetTargets()"""
        return self._targets.copy()
```

### 4. SdfPath系统对应

#### USD SdfPath
```cpp
class SdfPath {
    SdfPath GetParentPath() const;
    std::string GetName() const;
    SdfPath AppendChild(const TfToken& childName) const;
    bool IsRootPrimPath() const;
};
```

#### AssetX完全对应实现
```python
class SdfPath:
    """与USD SdfPath完全对应的实现"""
    
    def get_parent_path(self) -> 'SdfPath':
        """完全对应SdfPath::GetParentPath()"""
        if self._path == '/':
            return SdfPath('/')
        parent = '/'.join(self._path.split('/')[:-1])
        return SdfPath(parent if parent else '/')
    
    def get_name(self) -> str:
        """完全对应SdfPath::GetName()"""
        return self._path.split('/')[-1] if self._path != '/' else ''
    
    def append_child(self, child_name: str) -> 'SdfPath':
        """完全对应SdfPath::AppendChild()"""
        if self._path == '/':
            return SdfPath(f'/{child_name}')
        return SdfPath(f'{self._path}/{child_name}')
    
    def is_root_prim_path(self) -> bool:
        """完全对应SdfPath::IsRootPrimPath()"""
        return self._path.count('/') == 1 and self._path != '/'
```

## 机器人格式到USD的映射

### 1. URDF到USD映射

#### URDF结构
```xml
<robot name="my_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" .../>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
</robot>
```

#### 对应的USD结构
```python
# 创建机器人Stage
stage = AssetStage("my_robot.urdf")

# 机器人根Prim (对应<robot>)
robot_prim = stage.define_prim(SdfPath("/my_robot"), "Robot")
robot_prim.apply_api_schema("RobotAPI")
robot_prim.set_metadata("displayName", "my_robot")

# 链接Prim (对应<link>)
base_link = stage.define_prim(SdfPath("/my_robot/base_link"), "Link")
base_link.apply_api_schema("LinkAPI")
base_link.apply_api_schema("PhysicsAPI")

# 物理属性 (对应<inertial>)
mass_attr = base_link.create_attribute("physics:mass", "float")
mass_attr.set(1.0)

inertia_attr = base_link.create_attribute("physics:inertia", "matrix3d") 
inertia_attr.set([0.1, 0.1, 0.1, 0.0, 0.0, 0.0])

# 关节Prim (对应<joint>)
joint1 = stage.define_prim(SdfPath("/my_robot/joint1"), "Joint")
joint1.apply_api_schema("JointAPI")

# 关节关系 (对应parent/child)
parent_rel = joint1.create_relationship("physics:parent")
parent_rel.set_targets([SdfPath("/my_robot/base_link")])

child_rel = joint1.create_relationship("physics:child") 
child_rel.set_targets([SdfPath("/my_robot/link1")])

# 关节类型 (对应type="revolute")
joint_type_attr = joint1.create_attribute("physics:type", "token")
joint_type_attr.set("revolute")
```

### 2. MJCF到USD映射

#### MJCF结构
```xml
<mujoco>
  <worldbody>
    <body name="base_body">
      <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
      <body name="child_body">
        <joint type="hinge" axis="0 0 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
```

#### 对应的USD结构
```python
# MJCF世界体 -> USD Stage
stage = AssetStage("robot.xml")

# worldbody -> 伪根Prim
world_prim = stage.define_prim(SdfPath("/World"), "World")

# body -> Link Prim  
base_body = stage.define_prim(SdfPath("/World/base_body"), "Link")
base_body.apply_api_schema("LinkAPI")
base_body.apply_api_schema("PhysicsAPI")

# geom -> 几何属性
geom_type = base_body.create_attribute("geom:type", "token")
geom_type.set("box")

geom_size = base_body.create_attribute("geom:size", "float3")
geom_size.set([0.1, 0.1, 0.1])

mass_attr = base_body.create_attribute("physics:mass", "float")
mass_attr.set(1.0)

# 子body -> 子Link
child_body = stage.define_prim(SdfPath("/World/base_body/child_body"), "Link")

# joint -> Joint Prim
joint_prim = stage.define_prim(SdfPath("/World/base_body/child_joint"), "Joint") 
joint_prim.apply_api_schema("JointAPI")

joint_type = joint_prim.create_attribute("physics:type", "token")
joint_type.set("hinge")

joint_axis = joint_prim.create_attribute("physics:axis", "float3")
joint_axis.set([0, 0, 1])
```

## Schema系统对应

### USD Schema架构
```cpp
// USD中的Schema定义
class UsdGeomMesh : public UsdGeomPointBased {
    static UsdGeomMesh Define(const UsdStagePtr &stage, const SdfPath &path);
    UsdAttribute GetPointsAttr() const;
    UsdAttribute GetFaceVertexIndicesAttr() const;
};
```

### AssetX Schema实现
```python
# 通过API Schema模拟USD Schema系统
class PhysicsAPI:
    """物理API Schema"""
    
    @staticmethod
    def apply(prim: AssetPrim) -> bool:
        """将PhysicsAPI应用到Prim"""
        prim._api_schemas.add("PhysicsAPI")
        return True
    
    @staticmethod
    def create_mass_attr(prim: AssetPrim) -> AssetAttribute:
        """创建质量属性"""
        return prim.create_attribute("physics:mass", "float")
    
    @staticmethod  
    def create_inertia_attr(prim: AssetPrim) -> AssetAttribute:
        """创建惯性属性"""
        return prim.create_attribute("physics:inertia", "matrix3d")

# 使用方式
link_prim.apply_api_schema("PhysicsAPI")
mass_attr = PhysicsAPI.create_mass_attr(link_prim)
mass_attr.set(2.5)
```

### 机器人特定Schema

#### LinkAPI Schema
```python
class LinkAPI:
    """链接API Schema - 机器人特定"""
    
    SCHEMA_ATTRIBUTES = {
        "link:id": "int",           # 链接ID
        "link:parent_joint": "relationship",  # 父关节关系
        "link:child_joints": "relationship",  # 子关节关系列表
    }
    
    @staticmethod
    def apply(prim: AssetPrim) -> bool:
        prim._api_schemas.add("LinkAPI")
        return True
```

#### JointAPI Schema
```python
class JointAPI:
    """关节API Schema - 机器人特定"""
    
    SCHEMA_ATTRIBUTES = {
        "joint:type": "token",      # revolute, prismatic, fixed, etc.
        "joint:axis": "float3",     # 关节轴向量
        "joint:limits": "float2",   # [min, max] 关节限制
        "joint:parent": "relationship",  # 父链接关系
        "joint:child": "relationship",   # 子链接关系
    }
```

## 类型系统对应

### USD类型系统
```cpp
// USD标准类型
SdfValueTypeNames->Float
SdfValueTypeNames->Double  
SdfValueTypeNames->Int
SdfValueTypeNames->String
SdfValueTypeNames->Bool
SdfValueTypeNames->Vector3f
SdfValueTypeNames->Matrix4d
```

### AssetX类型映射
```python
# AssetX类型系统 - 直接对应USD
USD_TYPE_MAPPING = {
    # 基础类型
    "bool": bool,
    "int": int, 
    "float": float,
    "double": float,
    "string": str,
    "token": str,
    
    # 向量类型
    "float2": List[float],    # 2D向量
    "float3": List[float],    # 3D向量 
    "float4": List[float],    # 4D向量
    
    # 矩阵类型
    "matrix3d": List[float],  # 3x3矩阵 (9个元素)
    "matrix4d": List[float],  # 4x4矩阵 (16个元素)
    
    # 特殊类型
    "color3f": List[float],   # RGB颜色
    "relationship": List[SdfPath],  # 关系引用
}

# 使用示例
position_attr = prim.create_attribute("transform:translation", "float3")
position_attr.set([1.0, 2.0, 3.0])

rotation_attr = prim.create_attribute("transform:rotation", "float4") 
rotation_attr.set([0.0, 0.0, 0.0, 1.0])  # quaternion
```

## 元数据系统对应

### USD元数据
```cpp
// USD元数据访问
stage->SetMetadata("comment", "Generated by AssetX");
prim.SetMetadata("displayName", "Base Link");
attr.SetMetadata("documentation", "Mass in kilograms");
```

### AssetX元数据实现
```python
# 完全对应的元数据系统
stage.set_metadata("comment", "Generated by AssetX")
stage.set_metadata("version", "1.0")

prim.set_metadata("displayName", "Base Link")
prim.set_metadata("description", "Robot base link")

attr.set_metadata("documentation", "Mass in kilograms")
attr.set_metadata("units", "kg")
```

## 遍历系统对应

### USD遍历
```cpp
// USD遍历所有Prim
for (auto prim : stage->Traverse()) {
    if (prim.IsA<UsdGeomMesh>()) {
        // 处理网格
    }
}

// USD Range-based查询
auto range = stage->TraverseAll();
auto meshes = std::find_if(range.begin(), range.end(), 
    [](const UsdPrim& prim) { return prim.IsA<UsdGeomMesh>(); });
```

### AssetX遍历对应
```python
# 对应USD的遍历功能
for prim in stage.traverse():
    if prim.type_name == "Link":
        # 处理链接
        pass

# 对应USD的条件查询
links = asset.traverse(lambda prim: prim.type_name == "Link")
physics_prims = asset.traverse(lambda prim: prim.has_api_schema("PhysicsAPI"))

# 类型查询 (对应USD的IsA<>())
all_links = asset.find_prims_by_type("Link")
all_joints = asset.find_prims_by_type("Joint")
```

## 性能优化对应

### USD性能特性
- **延迟加载**: 只在需要时加载数据
- **路径缓存**: SdfPath的高效哈希和比较
- **层合成**: 多层数据的高效合并

### AssetX对应优化
```python
class AssetStage:
    def __init__(self, identifier: str = None):
        # 对应USD的路径缓存
        self._path_cache: Dict[str, SdfPath] = {}
        
        # 对应USD的延迟加载
        self._lazy_load_enabled = True
        
        # 对应USD的线程安全
        self._lock = RLock()
    
    def get_prim_at_path(self, path: SdfPath) -> Optional[AssetPrim]:
        """高效的路径查找 - 对应USD性能"""
        with self._lock:
            return self._prims.get(path)
    
    def _cache_path(self, path: SdfPath) -> SdfPath:
        """路径缓存优化"""
        path_str = str(path)
        if path_str not in self._path_cache:
            self._path_cache[path_str] = path
        return self._path_cache[path_str]
```

## 总结

AssetX与USD的对应关系总结：

### 1. 架构级对应
- **100%概念对应**: AssetX的每个核心概念都直接对应USD概念
- **API兼容性**: 方法名和参数尽可能保持与USD一致
- **行为一致性**: 相同操作产生相同的结果

### 2. 功能级对应
- **完整性**: 覆盖USD的所有核心功能
- **扩展性**: 支持机器人特定的Schema和类型
- **兼容性**: 可以与USD工具链集成

### 3. 性能级对应  
- **效率**: 采用USD的性能优化策略
- **可扩展性**: 支持大规模场景
- **内存管理**: 优化的数据结构和缓存

### 4. 生态级对应
- **工具支持**: 可以使用USD生态系统的工具
- **格式转换**: 可以导出为标准USD格式
- **可视化**: 支持USD查看器和编辑器

AssetX成功地将USD的强大架构引入机器人仿真领域，提供了现代化、可扩展、高性能的资产管理解决方案。
