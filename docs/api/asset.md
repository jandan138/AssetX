# AssetX Core API 详细参考

## 模块化架构概述

AssetX 采用模块化设计，基于 USD (Universal Scene Description) 架构原理，将功能拆分为独立的模块：

- **`assetx.core.asset`** - 主Asset接口，负责格式检测和加载
- **`assetx.core.stage`** - USD Stage概念，场景容器
- **`assetx.core.prim`** - USD Prim概念，场景对象
- **`assetx.core.property`** - USD Property概念，属性和关系
- **`assetx.core.sdf_path`** - USD SdfPath概念，层次化路径
- **`assetx.core.enums`** - 系统枚举定义

## 核心类 API

### `assetx.Asset` 类

主要的资产表示类，提供统一的多格式机器人仿真资产接口。基于USD架构设计，支持层次化场景管理。

#### 构造函数

```python
Asset(asset_path: Union[str, Path])
```

**参数:**
- `asset_path` (Union[str, Path]): 资产文件路径，支持 `.urdf`, `.xml`, `.usd`, `.json` 等格式

**示例:**
```python
from assetx import Asset, AssetFormat

# 加载 URDF 文件
robot = Asset("path/to/robot.urdf")
print(f"格式: {robot.format}")  # AssetFormat.URDF
robot.load()

# 检查是否已加载
print(f"已加载: {robot.is_loaded}")  # True
```

#### 属性

- **`asset_path`** (Path): 资产文件路径
- **`format`** (AssetFormat): 检测到的文件格式枚举
- **`is_loaded`** (bool): 是否已加载标志

#### 核心方法

##### `load() -> None`

加载并解析资产文件，创建内部USD风格的场景结构。

**示例:**
```python
asset = Asset("robot.urdf")
asset.load()
```

##### `get_stage() -> AssetStage`

获取资产的Stage对象，这是USD架构中的场景容器。

**返回:**
- `AssetStage`: 场景容器对象

**示例:**
```python
stage = asset.get_stage()
print(f"Stage标识符: {stage._identifier}")
```

##### `get_default_prim() -> Optional[AssetPrim]`

获取默认Prim对象，通常是机器人的根节点。

**返回:**
- `Optional[AssetPrim]`: 默认Prim对象或None

**示例:**
```python
default_prim = asset.get_default_prim()
if default_prim:
    print(f"默认Prim: {default_prim.path}")
    print(f"类型: {default_prim.type_name}")
```

#### USD风格的层次结构操作

##### `get_pseudo_root() -> Optional[AssetPrim]`

获取伪根Prim，USD场景的顶层节点。

**返回:**
- `Optional[AssetPrim]`: 伪根Prim对象

##### `get_root_prims() -> List[AssetPrim]`

获取所有根Prim列表。

**返回:**
- `List[AssetPrim]`: 根Prim对象列表

**示例:**
```python
root_prims = asset.get_root_prims()
print(f"根Prim数量: {len(root_prims)}")
for prim in root_prims:
    print(f"  - {prim.path.get_name()} ({prim.type_name})")
```

##### `define_prim(path: Union[str, SdfPath], type_name: str = None) -> AssetPrim`

在指定路径定义新的Prim对象。

**参数:**
- `path`: Prim路径
- `type_name`: Prim类型名称（可选）

**返回:**
- `AssetPrim`: 创建的Prim对象

**示例:**
```python
# 定义新的链接
new_link = asset.define_prim("/robot/new_link", "Link")
new_link.apply_api_schema("PhysicsAPI")
```

##### `remove_prim(path: Union[str, SdfPath]) -> bool`

移除指定路径的Prim对象。

**参数:**
- `path`: 要移除的Prim路径

**返回:**
- `bool`: 是否移除成功

#### 查询和遍历方法

##### `find_prims_by_type(type_name: str) -> List[AssetPrim]`

根据类型查找所有匹配的Prim对象。

**参数:**
- `type_name`: Prim类型名称（如 "Link", "Joint", "Robot"）

**返回:**
- `List[AssetPrim]`: 匹配的Prim对象列表

**示例:**
```python
# 查找所有链接
links = asset.find_prims_by_type("Link")
print(f"找到 {len(links)} 个链接")

# 查找所有关节
joints = asset.find_prims_by_type("Joint")
print(f"找到 {len(joints)} 个关节")
```

##### `traverse(condition: Callable[[AssetPrim], bool] = None) -> List[AssetPrim]`

遍历场景中的所有Prim，可选择性地应用过滤条件。

**参数:**
- `condition`: 可选的过滤函数

**返回:**
- `List[AssetPrim]`: 匹配条件的Prim列表

**示例:**
```python
# 遍历所有Prim
all_prims = asset.traverse()
print(f"总共 {len(all_prims)} 个Prim")

# 只遍历有PhysicsAPI的Prim
physics_prims = asset.traverse(
    lambda prim: prim.has_api_schema("PhysicsAPI")
)
print(f"有物理属性的Prim: {len(physics_prims)}")
```

#### 机器人特定的Schema应用接口

##### `create_robot_prim(path: Union[str, SdfPath], name: str = None) -> AssetPrim`

创建机器人根Prim，自动应用RobotAPI Schema。

**参数:**
- `path`: Prim路径
- `name`: 机器人名称（可选）

**返回:**
- `AssetPrim`: 创建的机器人Prim

**示例:**
```python
robot_prim = asset.create_robot_prim("/my_robot", "MyRobot")
print(f"应用的Schema: {robot_prim.get_applied_api_schemas()}")
```

##### `create_link_prim(path: Union[str, SdfPath], mass: float = None, inertia: List[float] = None) -> AssetPrim`

创建链接Prim，自动应用LinkAPI和PhysicsAPI Schema。

**参数:**
- `path`: Prim路径
- `mass`: 质量（可选）
- `inertia`: 惯性矩阵（可选）

**返回:**
- `AssetPrim`: 创建的链接Prim

##### `create_joint_prim(path: Union[str, SdfPath], joint_type: str = None) -> AssetPrim`

创建关节Prim，自动应用JointAPI Schema。

**参数:**
- `path`: Prim路径  
- `joint_type`: 关节类型（可选）

**返回:**
- `AssetPrim`: 创建的关节Prim

---

## AssetStage 类 - USD场景容器

### 概述

`AssetStage` 实现了USD的Stage概念，作为场景的根容器，管理所有Prim对象和场景级别的元数据。

### 构造函数

```python
AssetStage(identifier: str = None)
```

**参数:**
- `identifier`: Stage标识符，通常是文件路径

### 主要方法

#### Prim管理

##### `define_prim(path: SdfPath, type_name: str = None) -> AssetPrim`

在Stage中定义新的Prim。

**参数:**
- `path`: Prim路径
- `type_name`: Prim类型名称（可选）

**返回:**
- `AssetPrim`: 新创建的Prim对象

**示例:**
```python
stage = AssetStage("my_scene")
robot_prim = stage.define_prim(SdfPath("/robot"), "Robot")
```

##### `get_prim_at_path(path: SdfPath) -> Optional[AssetPrim]`

获取指定路径的Prim。

**参数:**
- `path`: 要查找的Prim路径

**返回:**
- `Optional[AssetPrim]`: Prim对象或None

##### `remove_prim(path: SdfPath) -> bool`

从Stage中移除Prim。

**参数:**
- `path`: 要移除的Prim路径

**返回:**
- `bool`: 是否移除成功

#### 遍历和查询

##### `traverse() -> List[AssetPrim]`

遍历Stage中的所有Prim。

**返回:**
- `List[AssetPrim]`: 所有Prim的列表

##### `get_default_prim() -> Optional[AssetPrim]`

获取默认Prim。

**返回:**
- `Optional[AssetPrim]`: 默认Prim或None

##### `set_default_prim(prim: AssetPrim) -> bool`

设置默认Prim。

**参数:**
- `prim`: 要设置为默认的Prim

**返回:**
- `bool`: 是否设置成功

#### 元数据管理

##### `get_metadata(key: str) -> Any`

获取Stage级元数据。

##### `set_metadata(key: str, value: Any) -> bool`

设置Stage级元数据。

**示例:**
```python
stage.set_metadata("version", "1.0")
stage.set_metadata("creator", "AssetX")
version = stage.get_metadata("version")
```

---

## AssetPrim 类 - USD场景对象

### 概述

`AssetPrim` 实现了USD的Prim概念，表示场景中的对象实例，可以有属性、关系和子对象。

### 构造函数

```python
AssetPrim(stage: AssetStage, path: SdfPath, type_name: str = None)
```

### 属性

- **`path`** (SdfPath): Prim的路径
- **`type_name`** (str): Prim的类型名称
- **`stage`** (AssetStage): 所属的Stage

### 层次结构操作

#### `get_parent() -> Optional[AssetPrim]`

获取父Prim。

**返回:**
- `Optional[AssetPrim]`: 父Prim对象或None

#### `get_children() -> List[AssetPrim]`

获取所有子Prim。

**返回:**
- `List[AssetPrim]`: 子Prim列表

#### `get_child(name: str) -> Optional[AssetPrim]`

获取指定名称的子Prim。

**参数:**
- `name`: 子Prim名称

**返回:**
- `Optional[AssetPrim]`: 子Prim对象或None

#### `add_child(child: AssetPrim) -> bool`

添加子Prim。

**参数:**
- `child`: 要添加的子Prim

**返回:**
- `bool`: 是否添加成功

### 属性和关系管理

#### `create_attribute(name: str, type_name: str) -> AssetAttribute`

创建新的属性。

**参数:**
- `name`: 属性名称
- `type_name`: 属性类型

**返回:**
- `AssetAttribute`: 创建的属性对象

**示例:**
```python
mass_attr = prim.create_attribute("physics:mass", "float")
mass_attr.set(2.5)
```

#### `create_relationship(name: str) -> AssetRelationship`

创建新的关系。

**参数:**
- `name`: 关系名称

**返回:**
- `AssetRelationship`: 创建的关系对象

#### `get_properties() -> List[AssetProperty]`

获取所有属性和关系。

**返回:**
- `List[AssetProperty]`: 属性列表

#### `get_property(name: str) -> Optional[AssetProperty]`

获取指定名称的属性或关系。

**参数:**
- `name`: 属性名称

**返回:**
- `Optional[AssetProperty]`: 属性对象或None

#### `get_attributes() -> List[AssetAttribute]`

获取所有Attribute。

#### `get_relationships() -> List[AssetRelationship]`

获取所有Relationship。

### Schema应用

#### `apply_api_schema(schema_name: str) -> bool`

应用API Schema到此Prim。

**参数:**
- `schema_name`: Schema名称

**返回:**
- `bool`: 是否应用成功

**示例:**
```python
link_prim.apply_api_schema("PhysicsAPI")
link_prim.apply_api_schema("LinkAPI")
```

#### `remove_api_schema(schema_name: str) -> bool`

移除API Schema。

#### `has_api_schema(schema_name: str) -> bool`

检查是否应用了指定Schema。

#### `get_applied_api_schemas() -> List[str]`

获取已应用的所有Schema。

### 状态管理

#### `is_active() -> bool`

检查Prim是否处于活跃状态。

#### `set_active(active: bool) -> bool`

设置Prim的活跃状态。

### 元数据管理

#### `get_metadata(key: str) -> Any`

获取Prim级元数据。

#### `set_metadata(key: str, value: Any) -> bool`

设置Prim级元数据。

**示例:**
```python
prim.set_metadata("displayName", "Base Link")
prim.set_metadata("description", "Robot base link")
```

---

## AssetProperty 基类

### 概述

`AssetProperty` 是USD Property系统的基类，包括Attribute和Relationship。

### 通用属性

- **`name`** (str): 属性名称
- **`type_name`** (str): 属性类型
- **`prim`** (AssetPrim): 所属的Prim

### 通用方法

#### `get_metadata(key: str) -> Any`

获取属性级元数据。

#### `set_metadata(key: str, value: Any) -> bool`

设置属性级元数据。

---

## AssetAttribute 类 - 数据属性

### 概述

`AssetAttribute` 继承自`AssetProperty`，用于存储数据值，支持时间采样。

### 值操作

#### `get(time: Optional[float] = None) -> Any`

获取属性值。

**参数:**
- `time`: 时间点（可选，默认获取默认值）

**返回:**
- `Any`: 属性值

#### `set(value: Any, time: Optional[float] = None) -> bool`

设置属性值。

**参数:**
- `value`: 要设置的值
- `time`: 时间点（可选，默认设置默认值）

**返回:**
- `bool`: 是否设置成功

**示例:**
```python
# 设置默认值
mass_attr.set(1.5)

# 设置时间采样值
mass_attr.set(2.0, time=1.0)
mass_attr.set(2.5, time=2.0)

# 获取值
default_mass = mass_attr.get()
mass_at_time_1 = mass_attr.get(time=1.0)
```

#### `get_time_samples() -> Dict[float, Any]`

获取所有时间采样值。

**返回:**
- `Dict[float, Any]`: 时间->值的映射

#### `has_authored_value() -> bool`

检查是否有已设置的值。

#### `clear_default_value() -> bool`

清除默认值。

---

## AssetRelationship 类 - 关系引用

### 概述

`AssetRelationship` 继承自`AssetProperty`，用于存储对其他Prim的引用关系。

### 目标管理

#### `get_targets() -> List[SdfPath]`

获取所有目标路径。

**返回:**
- `List[SdfPath]`: 目标路径列表

#### `set_targets(targets: List[SdfPath]) -> bool`

设置目标路径。

**参数:**
- `targets`: 目标路径列表

**返回:**
- `bool`: 是否设置成功

#### `add_target(target: SdfPath) -> bool`

添加单个目标。

#### `remove_target(target: SdfPath) -> bool`

移除单个目标。

#### `clear_targets() -> bool`

清除所有目标。

**示例:**
```python
# 创建关节的父子关系
parent_rel = joint_prim.create_relationship("physics:parent")
parent_rel.set_targets([SdfPath("/robot/base_link")])

child_rel = joint_prim.create_relationship("physics:child")
child_rel.add_target(SdfPath("/robot/arm_link"))

# 获取关系目标
parent_targets = parent_rel.get_targets()
print(f"父链接: {parent_targets[0]}")
```

---

## SdfPath 类 - 层次化路径

### 概述

`SdfPath` 实现USD的路径系统，用于唯一标识场景中的对象。

### 构造函数

```python
SdfPath(path: str)
```

**参数:**
- `path`: 路径字符串，必须以'/'开头

### 路径操作

#### `get_parent_path() -> SdfPath`

获取父路径。

#### `get_name() -> str`

获取路径的最后一个组件名称。

#### `append_child(child_name: str) -> SdfPath`

追加子路径。

#### `is_root_prim_path() -> bool`

检查是否是根Prim路径。

**示例:**
```python
# 路径操作
robot_path = SdfPath("/my_robot")
link_path = robot_path.append_child("base_link")
joint_path = robot_path.append_child("joint1")

print(link_path)                    # /my_robot/base_link
print(link_path.get_parent_path())  # /my_robot  
print(link_path.get_name())         # base_link
print(link_path.is_root_prim_path()) # False
```

---

## 使用示例

### 基本工作流程

```python
from assetx import Asset, AssetFormat

# 1. 创建和加载资产
asset = Asset("robot.urdf")
asset.load()

# 2. 获取Stage和默认Prim
stage = asset.get_stage()
robot_prim = asset.get_default_prim()

# 3. 遍历和查询
links = asset.find_prims_by_type("Link")
joints = asset.find_prims_by_type("Joint")

# 4. 属性操作
for link in links:
    mass_attr = link.get_property("physics:mass")
    if mass_attr:
        print(f"{link.path.get_name()} 质量: {mass_attr.get()}")

# 5. 创建新对象
new_link = asset.create_link_prim("/robot/new_link", mass=1.0)
new_joint = asset.create_joint_prim("/robot/new_joint", joint_type="revolute")
```

### 高级用法

```python
# 条件遍历
physics_prims = asset.traverse(
    lambda prim: prim.has_api_schema("PhysicsAPI")
)

# Schema应用
for link in links:
    if not link.has_api_schema("MeshAPI"):
        link.apply_api_schema("MeshAPI")
        mesh_attr = link.create_attribute("mesh:file", "string")
        mesh_attr.set(f"meshes/{link.path.get_name()}.obj")

# 元数据管理
stage.set_metadata("generator", "AssetX v1.0")
stage.set_metadata("creation_time", "2025-01-01T00:00:00Z")

for prim in asset.traverse():
    prim.set_metadata("processed", True)
```

这个更新的API文档全面反映了AssetX的模块化架构和USD风格设计，为用户提供了完整的参考信息。

遍历Stage中的所有Prim。

---

## AssetPrim 类 - USD场景对象

### 概述

`AssetPrim` 实现了USD的Prim概念，表示场景中的对象实例，可以有属性、关系和子对象。

### 主要方法

#### 层次结构操作

- `get_parent() -> Optional[AssetPrim]` - 获取父Prim
- `get_children() -> List[AssetPrim]` - 获取子Prim列表
- `get_child(name: str) -> Optional[AssetPrim]` - 获取指定名称的子Prim

#### 属性和关系管理

- `create_attribute(name: str, type_name: str) -> AssetAttribute` - 创建属性
- `create_relationship(name: str) -> AssetRelationship` - 创建关系
- `get_properties() -> List[AssetProperty]` - 获取所有属性
- `get_property(name: str) -> Optional[AssetProperty]` - 获取指定属性

#### Schema应用

- `apply_api_schema(schema_name: str) -> bool` - 应用API Schema
- `remove_api_schema(schema_name: str) -> bool` - 移除API Schema
- `has_api_schema(schema_name: str) -> bool` - 检查是否有指定Schema
- `get_applied_api_schemas() -> List[str]` - 获取已应用的Schema列表

---

#### 类属性

- **`supported_conversions`** (Dict): 支持的转换路径映射

#### 方法

##### `convert(source_path: str, target_format: str, output_path: str) -> bool`

执行格式转换。

**参数:**
- `source_path` (str): 源文件路径
- `target_format` (str): 目标格式 ('urdf', 'mjcf', 'usd', 'genesis')
- `output_path` (str): 输出文件路径

**返回:** 转换是否成功

**示例:**
```python
from assetx import FormatConverter

converter = FormatConverter()

# URDF 转 MJCF
success = converter.convert(
    source_path="robot.urdf",
    target_format="mjcf", 
    output_path="robot.xml"
)

if success:
    print("✅ 转换完成")
else:
    print("❌ 转换失败")
```

##### `urdf_to_mjcf(urdf_path: str, mjcf_path: str) -> bool`

专用的 URDF 到 MJCF 转换方法。

**参数:**
- `urdf_path` (str): URDF 文件路径
- `mjcf_path` (str): 输出的 MJCF 文件路径

**返回:** 转换是否成功

**示例:**
```python
converter = FormatConverter()
result = converter.urdf_to_mjcf("input.urdf", "output.xml")
```

##### `mjcf_to_urdf(mjcf_path: str, urdf_path: str) -> bool`

专用的 MJCF 到 URDF 转换方法（计划中）。

---

### `assetx.PhysicsValidator` 类

物理参数验证器，检查不同格式间的物理参数一致性。

#### 构造函数

```python
PhysicsValidator()
```

#### 方法

##### `validate_asset(asset: Asset) -> ValidationResult`

验证单个资产的物理参数。

**参数:**
- `asset` (Asset): 要验证的资产对象

**返回:** ValidationResult 对象

**示例:**
```python
from assetx import Asset, PhysicsValidator

asset = Asset("robot.urdf")
asset.load()

validator = PhysicsValidator()
result = validator.validate_asset(asset)

print(f"验证结果: {'通过' if result.is_valid else '失败'}")
print(f"错误数量: {len(result.errors)}")
print(f"警告数量: {len(result.warnings)}")
```

##### `compare_assets(asset1: Asset, asset2: Asset) -> ValidationResult`

比较两个资产之间的物理参数一致性。

**参数:**
- `asset1` (Asset): 参考资产
- `asset2` (Asset): 目标资产

**返回:** ValidationResult 对象

**示例:**
```python
# 比较 URDF 和转换后的 MJCF
urdf_asset = Asset("robot.urdf")
mjcf_asset = Asset("robot.xml")
urdf_asset.load()
mjcf_asset.load()

validator = PhysicsValidator()
comparison = validator.compare_assets(urdf_asset, mjcf_asset)

if comparison.is_valid:
    print("✅ 两个资产的物理参数一致")
else:
    print("❌ 发现物理参数差异:")
    for error in comparison.errors:
        print(f"  - {error}")
```

##### `validate_mass_properties(asset: Asset) -> List[str]`

验证质量和惯性参数。

**返回:** 错误消息列表

##### `validate_geometry_consistency(asset: Asset) -> List[str]`

验证几何体一致性。

**返回:** 错误消息列表

##### `validate_joint_limits(asset: Asset) -> List[str]`

验证关节限制参数。

**返回:** 错误消息列表

---

### `assetx.MeshProcessor` 类

3D 网格处理工具，用于优化和转换几何体资产。

#### 构造函数

```python
MeshProcessor()
```

#### 方法

##### `process_mesh(mesh_path: str, **kwargs) -> str`

处理网格文件，应用各种优化操作。

**参数:**
- `mesh_path` (str): 输入网格文件路径
- `**kwargs`: 处理参数
  - `scale` (float): 缩放因子
  - `units` (str): 目标单位
  - `simplify` (bool): 是否简化
  - `target_faces` (int): 目标面数

**返回:** 处理后的网格文件路径

**示例:**
```python
from assetx import MeshProcessor

processor = MeshProcessor()

# 缩放并简化网格
output_path = processor.process_mesh(
    mesh_path="detailed_mesh.obj",
    scale=0.001,  # 从毫米转换到米
    units="meters",
    simplify=True,
    target_faces=1000
)

print(f"处理后的网格: {output_path}")
```

##### `simplify_mesh(mesh_path: str, target_faces: int) -> str`

简化网格，减少面数。

**参数:**
- `mesh_path` (str): 输入网格路径
- `target_faces` (int): 目标面数

**返回:** 简化后的网格路径

##### `scale_mesh(mesh_path: str, scale_factor: float) -> str`

缩放网格大小。

**参数:**
- `mesh_path` (str): 输入网格路径  
- `scale_factor` (float): 缩放因子

**返回:** 缩放后的网格路径

##### `convert_units(mesh_path: str, from_unit: str, to_unit: str) -> str`

转换网格单位。

**参数:**
- `mesh_path` (str): 输入网格路径
- `from_unit` (str): 源单位 ('mm', 'cm', 'm', 'inch', 'ft')
- `to_unit` (str): 目标单位

**返回:** 转换后的网格路径

##### `generate_collision_mesh(visual_mesh_path: str) -> str`

从视觉网格生成碰撞网格。

**参数:**
- `visual_mesh_path` (str): 视觉网格路径

**返回:** 生成的碰撞网格路径

**示例:**
```python
collision_mesh = processor.generate_collision_mesh("visual.obj")
print(f"生成碰撞网格: {collision_mesh}")
```

##### `center_mesh(mesh_path: str) -> str`

将网格居中到原点。

**参数:**
- `mesh_path` (str): 输入网格路径

**返回:** 居中后的网格路径

---

### `assetx.MetaManager` 类

元数据管理器，用于管理资产的版本信息、标签和历史记录。

#### 构造函数

```python
MetaManager(base_path: str = ".")
```

**参数:**
- `base_path` (str): 工作目录路径

#### 方法

##### `create_meta(asset_path: str, **metadata) -> str`

为资产创建元数据文件。

**参数:**
- `asset_path` (str): 资产文件路径
- `**metadata`: 额外的元数据信息

**返回:** 元数据文件路径

**示例:**
```python
from assetx import MetaManager

meta_manager = MetaManager()

meta_path = meta_manager.create_meta(
    asset_path="robot.urdf",
    category="manipulator",
    author="AssetX User",
    description="六自由度机械臂",
    tags=["arm", "industrial", "6dof"]
)

print(f"元数据文件: {meta_path}")
```

##### `load_meta(meta_path: str) -> Dict`

加载元数据文件。

**参数:**
- `meta_path` (str): 元数据文件路径

**返回:** 元数据字典

##### `update_meta(meta_path: str, **updates) -> None`

更新元数据文件。

**参数:**
- `meta_path` (str): 元数据文件路径
- `**updates`: 要更新的字段

**示例:**
```python
meta_manager.update_meta(
    meta_path="robot_meta.yaml",
    version="1.1.0",
    last_modified="2024-01-15"
)
```

##### `register_conversion(meta_path: str, source_format: str, target_format: str) -> None`

记录格式转换历史。

**参数:**
- `meta_path` (str): 元数据文件路径
- `source_format` (str): 源格式
- `target_format` (str): 目标格式

##### `get_conversion_history(meta_path: str) -> List[Dict]`

获取转换历史记录。

**参数:**
- `meta_path` (str): 元数据文件路径

**返回:** 转换历史列表

---

### `assetx.Previewer` 类

资产可视化预览器，支持 3D 渲染和交互式查看。

#### 构造函数

```python
Previewer(backend: str = "auto")
```

**参数:**
- `backend` (str): 渲染后端 ('trimesh', 'open3d', 'auto')

#### 方法

##### `preview_asset(asset: Asset) -> None`

预览资产的 3D 模型。

**参数:**
- `asset` (Asset): 要预览的资产对象

**示例:**
```python
from assetx import Asset, Previewer

asset = Asset("robot.urdf")
asset.load()

previewer = Previewer(backend="open3d")
previewer.preview_asset(asset)  # 打开 3D 查看器
```

##### `preview_mesh(mesh_path: str) -> None`

预览单个网格文件。

**参数:**
- `mesh_path` (str): 网格文件路径

##### `compare_meshes(mesh1_path: str, mesh2_path: str) -> None`

并排比较两个网格。

**参数:**
- `mesh1_path` (str): 第一个网格路径
- `mesh2_path` (str): 第二个网格路径

##### `take_screenshot(asset: Asset, output_path: str) -> None`

保存资产的截图。

**参数:**
- `asset` (Asset): 资产对象
- `output_path` (str): 截图保存路径

---

## 数据类型

### `PhysicsProperties`

物理属性数据类。

```python
@dataclass
class PhysicsProperties:
    total_mass: float = 0.0
    center_of_mass: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    inertia_matrix: List[List[float]] = field(default_factory=lambda: [[0.0]*3]*3)
    joint_limits: Dict[str, Dict[str, float]] = field(default_factory=dict)
```

### `ValidationResult`

验证结果数据类。

```python
@dataclass  
class ValidationResult:
    is_valid: bool = True
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    summary: str = ""
```

### `GeometryInfo`

几何体信息数据类。

```python
@dataclass
class GeometryInfo:
    type: str  # 'mesh', 'box', 'cylinder', 'sphere'
    dimensions: List[float]
    mesh_path: str = ""
    scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
```

### `JointInfo`

关节信息数据类。

```python
@dataclass
class JointInfo:
    name: str
    type: str  # 'revolute', 'prismatic', 'fixed', etc.
    parent_link: str
    child_link: str
    axis: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    limits: Dict[str, float] = field(default_factory=dict)
```

---

## 异常类型

### `AssetXError`

AssetX 基础异常类。

### `UnsupportedFormatError`

不支持的格式异常。

### `ConversionError`

转换过程异常。

### `ValidationError`

验证过程异常。

---

## 使用模式

### 基本工作流

```python
from assetx import Asset, FormatConverter, PhysicsValidator

# 1. 加载资产
robot = Asset("robot.urdf")
robot.load()

# 2. 验证资产
validator = PhysicsValidator()
result = validator.validate_asset(robot)

# 3. 转换格式
if result.is_valid:
    converter = FormatConverter()
    converter.convert("robot.urdf", "mjcf", "robot.xml")
```

### 高级处理流程

```python
from assetx import *

# 完整的资产处理管道
def process_robot_asset(input_path: str):
    # 加载和验证
    asset = Asset(input_path)
    asset.load()
    
    # 物理验证
    validator = PhysicsValidator()
    validation = validator.validate_asset(asset)
    
    if not validation.is_valid:
        print("资产验证失败，需要修复:")
        for error in validation.errors:
            print(f"  - {error}")
        return
    
    # 网格优化
    processor = MeshProcessor()
    for link in asset.links:
        if link.visual_mesh:
            optimized = processor.process_mesh(
                link.visual_mesh,
                scale=0.001,  # mm -> m
                simplify=True,
                target_faces=500
            )
            link.visual_mesh = optimized
    
    # 创建元数据
    meta_manager = MetaManager()
    meta_path = meta_manager.create_meta(
        asset_path=input_path,
        category="mobile_robot",
        version="1.0.0"
    )
    
    # 格式转换
    converter = FormatConverter()
    mjcf_path = input_path.replace('.urdf', '.xml')
    converter.convert(input_path, "mjcf", mjcf_path)
    
    # 记录转换历史
    meta_manager.register_conversion(meta_path, "urdf", "mjcf")
    
    # 预览结果
    previewer = Previewer()
    previewer.preview_asset(asset)
    
    print(f"✅ 资产处理完成: {mjcf_path}")

# 使用示例
process_robot_asset("my_robot.urdf")
```
