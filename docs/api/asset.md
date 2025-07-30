# AssetX API 详细参考

## 核心模块 API

### `assetx.Asset` 类

统一的资产表示类，支持多种格式的机器人仿真资产。

#### 构造函数

```python
Asset(file_path: str)
```

**参数:**
- `file_path` (str): 资产文件路径，支持 `.urdf`, `.xml`, `.usd`, `.json` 等格式

**示例:**
```python
from assetx import Asset

# 加载 URDF 文件
robot = Asset("path/to/robot.urdf")
robot.load()

# 加载 MJCF 文件  
robot_mjcf = Asset("path/to/robot.xml")
robot_mjcf.load()
```

#### 属性

- **`format`** (str): 检测到的文件格式 ('urdf', 'mjcf', 'usd', 'genesis')
- **`name`** (str): 资产名称
- **`links`** (List[LinkInfo]): 链接信息列表
- **`joints`** (List[JointInfo]): 关节信息列表
- **`physics`** (PhysicsProperties): 物理属性
- **`file_path`** (str): 原始文件路径

#### 方法

##### `load() -> None`

加载并解析资产文件。

**示例:**
```python
asset = Asset("robot.urdf")
asset.load()
print(f"加载了 {len(asset.links)} 个链接")
```

##### `get_format() -> str`

获取资产的文件格式。

**返回:** 格式字符串 ('urdf', 'mjcf', 'usd', 'genesis', 'unknown')

**示例:**
```python
format_type = asset.get_format()
print(f"资产格式: {format_type}")
```

##### `validate() -> ValidationResult`

验证资产的物理参数和结构完整性。

**返回:** ValidationResult 对象，包含验证详情

**示例:**
```python
result = asset.validate()
if result.is_valid:
    print("✅ 资产验证通过")
else:
    print(f"❌ 发现 {len(result.errors)} 个错误")
    for error in result.errors:
        print(f"  - {error}")
```

---

### `assetx.FormatConverter` 类

多格式转换引擎，支持不同仿真格式之间的转换。

#### 构造函数

```python
FormatConverter()
```

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
