# USD (Universal Scene Description) 架构深度解析

## 概述

USD (Universal Scene Description) 是由Pixar开发的一个开源框架，用于交换和增强3D图形数据。本文档深入解析USD的核心架构概念，以及AssetX如何采用这些设计原理。

## USD 核心概念详解

### 1. Stage (场景舞台)

#### USD中的Stage
- **定义**: Stage是USD中的顶级容器，代表一个完整的场景
- **职责**: 管理场景的所有内容，包括Prim层次结构、时间轴、元数据
- **特点**: 
  - 每个Stage可以包含多个Layer
  - 支持场景级别的元数据
  - 管理默认Prim和伪根(Pseudo Root)

#### AssetX中的实现
```python
class AssetStage:
    """USD风格的Stage类 - 资产的主要容器"""
    
    def __init__(self, identifier: str = None):
        self._identifier = identifier or "anonymous"
        self._root_prim: Optional[AssetPrim] = None
        self._prims: Dict[SdfPath, AssetPrim] = {}
        self._metadata: Dict[str, Any] = {}
```

**关键特性**:
- 维护Prim的字典映射 (`_prims`)
- 支持默认Prim设置 (`_root_prim`)
- 场景级元数据管理 (`_metadata`)
- 线程安全的操作 (使用`threading.RLock`)

### 2. Prim (Primitive 场景对象)

#### USD中的Prim
- **定义**: Prim是场景中的基本构建块，代表场景中的一个对象
- **层次结构**: Prim组织成树状结构，有父子关系
- **类型系统**: 每个Prim都有一个类型(TypeName)，如 "Mesh", "Xform", "Camera"
- **Schema支持**: 通过API Schema扩展功能

#### AssetX中的实现
```python
class AssetPrim:
    """USD风格的Prim类 - 场景对象的基本构建块"""
    
    def __init__(self, stage: 'AssetStage', path: SdfPath, type_name: str = None):
        self._stage = stage
        self._path = path
        self._type_name = type_name or ""
        self._properties: Dict[str, AssetProperty] = {}
        self._children: Dict[str, 'AssetPrim'] = {}
        self._metadata: Dict[str, Any] = {}
        self._api_schemas: Set[str] = set()
```

**机器人特定的类型**:
- `Robot`: 机器人根节点
- `Link`: 机器人链接
- `Joint`: 机器人关节

**Schema应用示例**:
```python
# 为链接应用物理API
link_prim.apply_api_schema("PhysicsAPI")
link_prim.apply_api_schema("LinkAPI")

# 检查Schema
if link_prim.has_api_schema("PhysicsAPI"):
    # 可以安全地设置物理属性
    mass_attr = link_prim.create_attribute("physics:mass", "float")
    mass_attr.set(2.5)
```

### 3. Property (属性系统)

#### USD中的Property
- **分类**: Property分为Attribute(属性)和Relationship(关系)两大类
- **Attribute**: 存储数据值，如位置、颜色、质量等
- **Relationship**: 存储对其他Prim的引用关系

#### AssetX中的实现

**基础Property类**:
```python
class AssetProperty:
    """USD风格的Property基类"""
    
    def __init__(self, prim: 'AssetPrim', name: str, type_name: str):
        self._prim = prim
        self._name = name
        self._type_name = type_name
        self._variability = VariabilityType.VARYING
        self._metadata: Dict[str, Any] = {}
```

**Attribute实现**:
```python
class AssetAttribute(AssetProperty):
    """USD风格的Attribute类 - 存储数据值"""
    
    def __init__(self, prim: 'AssetPrim', name: str, type_name: str):
        super().__init__(prim, name, type_name)
        self._default_value: Any = None
        self._time_samples: Dict[float, Any] = {}
```

**Relationship实现**:
```python
class AssetRelationship(AssetProperty):
    """USD风格的Relationship类 - 存储引用关系"""
    
    def __init__(self, prim: 'AssetPrim', name: str):
        super().__init__(prim, name, "relationship")
        self._targets: List[SdfPath] = []
```

**物理属性示例**:
```python
# 创建质量属性
mass_attr = link_prim.create_attribute("physics:mass", "float")
mass_attr.set(1.5)

# 创建惯性矩阵属性
inertia_attr = link_prim.create_attribute("physics:inertia", "matrix3d")
inertia_attr.set([0.1, 0.1, 0.1, 0.0, 0.0, 0.0])

# 创建关节关系
parent_rel = joint_prim.create_relationship("physics:parent")
parent_rel.set_targets([parent_link.path])
```

### 4. SdfPath (场景描述路径)

#### USD中的SdfPath
- **定义**: SdfPath是USD中用于标识场景对象的层次化路径系统
- **格式**: 类似文件系统路径，如 `/World/Robot/base_link`
- **特点**: 
  - 唯一标识场景中的对象
  - 支持路径操作(父路径、子路径、相对路径)
  - 高效的路径比较和哈希

#### AssetX中的实现
```python
class SdfPath:
    """USD风格的SdfPath类 - 层次化路径系统"""
    
    def __init__(self, path: str):
        if not path.startswith('/'):
            raise ValueError("SdfPath must start with '/'")
        self._path = path.rstrip('/')
    
    def get_parent_path(self) -> 'SdfPath':
        """获取父路径"""
        if self._path == '/':
            return SdfPath('/')
        parent = '/'.join(self._path.split('/')[:-1])
        return SdfPath(parent if parent else '/')
    
    def append_child(self, child_name: str) -> 'SdfPath':
        """追加子路径"""
        if self._path == '/':
            return SdfPath(f'/{child_name}')
        return SdfPath(f'{self._path}/{child_name}')
```

**路径操作示例**:
```python
# 创建路径
robot_path = SdfPath("/my_robot")
link_path = robot_path.append_child("base_link")
joint_path = robot_path.append_child("joint1")

# 路径操作
print(link_path.get_parent_path())  # /my_robot
print(link_path.get_name())         # base_link
print(link_path.is_root_prim_path()) # False
```

### 5. 元数据系统 (Metadata)

#### USD中的元数据
- **范围**: Stage级别、Prim级别、Property级别都可以有元数据
- **用途**: 存储不影响渲染但对工具有用的信息
- **示例**: 版本信息、作者、描述、工具特定的标记

#### AssetX中的实现
```python
# Stage级元数据
stage.set_metadata("version", "1.0")
stage.set_metadata("creator", "AssetX")

# Prim级元数据  
robot_prim.set_metadata("displayName", "My Robot")
robot_prim.set_metadata("description", "A sample robot for testing")

# Property级元数据
mass_attr.set_metadata("units", "kg")
mass_attr.set_metadata("description", "Link mass in kilograms")
```

## USD设计模式在AssetX中的应用

### 1. 组合模式 (Composite Pattern)

USD使用组合模式来构建场景层次结构，AssetX遵循同样的模式：

```python
# 机器人层次结构
robot_prim = asset.create_robot_prim("/my_robot")
├── base_link = asset.create_link_prim("/my_robot/base_link")
├── arm_link = asset.create_link_prim("/my_robot/arm_link")  
└── gripper_link = asset.create_link_prim("/my_robot/gripper_link")
    └── finger1 = asset.create_link_prim("/my_robot/gripper_link/finger1")
```

### 2. 策略模式 (Strategy Pattern)

通过API Schema系统实现不同的行为策略：

```python
# 物理策略
link_prim.apply_api_schema("PhysicsAPI")
# 现在可以设置质量、惯性等物理属性

# 可视化策略  
link_prim.apply_api_schema("MeshAPI")
# 现在可以设置网格、材质等视觉属性

# 机器人特定策略
link_prim.apply_api_schema("LinkAPI")
# 现在可以设置机器人链接特有的属性
```

### 3. 访问者模式 (Visitor Pattern)

通过遍历系统实现不同的处理逻辑：

```python
# 查找所有有质量的链接
physics_links = asset.traverse(
    lambda prim: prim.has_api_schema("PhysicsAPI") and 
                 prim.type_name == "Link"
)

# 计算总质量
total_mass = sum(
    prim.get_property("physics:mass").get() 
    for prim in physics_links
    if prim.get_property("physics:mass")
)
```

## USD vs 传统机器人格式的优势

### 1. 层次化组织
- **传统URDF**: 扁平的XML结构，难以表达复杂层次
- **USD风格**: 自然的树状结构，支持任意深度的嵌套

### 2. 可扩展性
- **传统格式**: 固定的标签和属性集合
- **USD风格**: 通过Schema系统动态扩展功能

### 3. 元数据支持
- **传统格式**: 有限的元数据支持
- **USD风格**: 多层级的丰富元数据系统

### 4. 类型安全
- **传统格式**: 字符串驱动，运行时错误
- **USD风格**: 强类型系统，编译时检查

### 5. 工具生态
- **传统格式**: 每种格式需要专门工具
- **USD风格**: 统一的工具链，更好的互操作性

## 最佳实践

### 1. 路径命名规范
```python
# 好的路径命名
robot_path = SdfPath("/robots/my_robot")
base_link_path = robot_path.append_child("base_link")
joint_path = robot_path.append_child("joints").append_child("joint1")

# 避免的路径命名
bad_path = SdfPath("/robot123/link_0001")  # 无意义的数字
```

### 2. Schema应用策略
```python
# 先应用基础Schema，再应用特定Schema
link_prim.apply_api_schema("PhysicsAPI")  # 基础物理
link_prim.apply_api_schema("LinkAPI")     # 机器人特定
link_prim.apply_api_schema("MeshAPI")     # 可视化
```

### 3. 属性命名空间
```python
# 使用命名空间避免冲突
mass_attr = link_prim.create_attribute("physics:mass", "float")
visual_color = link_prim.create_attribute("display:color", "color3f")
robot_id = link_prim.create_attribute("robot:link_id", "int")
```

### 4. 元数据使用
```python
# 为工具和用户提供有用的元数据
link_prim.set_metadata("displayName", "Base Link")
link_prim.set_metadata("description", "Main base link of the robot")
link_prim.set_metadata("version", "1.0")
link_prim.set_metadata("author", "Robot Designer")
```

## 总结

AssetX的USD风格架构提供了：

1. **统一的数据模型**: 所有格式都映射到同一套USD概念
2. **强大的扩展性**: 通过Schema系统支持任意属性和行为
3. **清晰的层次结构**: 自然表达机器人的组件关系
4. **类型安全**: 强类型的属性系统减少错误
5. **丰富的元数据**: 支持工具和工作流的额外信息
6. **高效的查询**: SdfPath系统提供快速的对象定位

这种架构为机器人仿真领域带来了现代化的数据交换和管理方案，既保持了与现有格式的兼容性，又提供了面向未来的扩展能力。
