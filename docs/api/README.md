# AssetX API 参考文档

这里提供AssetX Python API的完整参考文档。

## 📚 模块索引

### 核心模块
- [Asset](asset.md) - 统一资产表示类
- [FormatConverter](converter.md) - 格式转换器
- [PhysicsValidator](validator.md) - 物理参数验证器

### 辅助模块  
- [MeshProcessor](mesh-processor.md) - 网格处理工具
- [MetaManager](meta-manager.md) - 元数据管理器
- [Previewer](previewer.md) - 可视化预览器

### CLI模块
- [CLI Commands](cli.md) - 命令行工具参考

## 🚀 快速导入

```python
# 导入主要类
from assetx import Asset, FormatConverter, PhysicsValidator
from assetx import MeshProcessor, MetaManager, Previewer

# 或者按模块导入
from assetx.core.asset import Asset
from assetx.core.converter import FormatConverter
from assetx.core.validator import PhysicsValidator
from assetx.mesh.processor import MeshProcessor
from assetx.meta.manager import MetaManager
from assetx.viewer.preview import Previewer
```

## 🔄 基本工作流API

### 加载和转换资产

```python
from assetx import Asset, FormatConverter

# 加载资产
asset = Asset("robot.urdf")
asset.load()

# 转换格式
converter = FormatConverter()
output_path = converter.convert("robot.urdf", "mjcf", "robot.xml")
```

### 验证物理参数

```python
from assetx import PhysicsValidator

validator = PhysicsValidator()

# 验证单个资产
result = validator.validate_asset(asset)
print(f"Valid: {result.is_valid}")

# 比较两个资产
asset2 = Asset("robot.xml")
asset2.load()
comparison = validator.compare_assets(asset, asset2)
```

### 处理网格

```python
from assetx import MeshProcessor

processor = MeshProcessor()

# 简化网格
simplified = processor.simplify_mesh("mesh.obj", target_faces=1000)

# 生成碰撞体
collision = processor.generate_collision_mesh("visual.obj")
```

### 管理元数据

```python
from assetx import MetaManager

meta_manager = MetaManager("./assets")

# 创建资产元数据
meta_data = meta_manager.create_asset_meta(
    asset_name="robot.urdf",
    original_format="urdf", 
    semantic_category="robot_arm",
    description="My robot description"
)

# 更新元数据
meta_manager.update_asset_meta("robot.urdf", meta_data)
```

## 🏗️ 高级用法

### 自定义转换器

```python
from assetx.core.converter import FormatConverter

class CustomConverter(FormatConverter):
    def __init__(self):
        super().__init__()
        # 添加自定义转换支持
        self.supported_conversions['custom'] = ['urdf', 'mjcf']
    
    def _custom_to_urdf(self, asset, output_path):
        # 实现自定义格式到URDF的转换
        pass
```

### 批量处理

```python
from pathlib import Path
from assetx import Asset, FormatConverter

def batch_convert(input_dir, output_dir, target_format):
    """批量转换目录中的所有资产"""
    converter = FormatConverter()
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    for urdf_file in input_path.glob("*.urdf"):
        try:
            output_file = output_path / f"{urdf_file.stem}.xml"
            converter.convert(urdf_file, target_format, output_file)
            print(f"✅ Converted: {urdf_file.name}")
        except Exception as e:
            print(f"❌ Failed: {urdf_file.name} - {e}")

# 使用示例
batch_convert("./urdf_models", "./mjcf_models", "mjcf")
```

### 验证工作流

```python
from assetx import Asset, PhysicsValidator, MetaManager

def validate_and_record(asset_path, meta_dir):
    """验证资产并记录结果"""
    # 加载资产
    asset = Asset(asset_path)
    asset.load()
    
    # 验证物理参数
    validator = PhysicsValidator()
    result = validator.validate_asset(asset)
    
    # 记录验证结果
    meta_manager = MetaManager(meta_dir)
    meta_manager.add_validation_record(
        asset_name=asset_path.name,
        validation_type="physics",
        result=result.get_summary()
    )
    
    return result

# 使用示例
result = validate_and_record(Path("robot.urdf"), Path("."))
```

## 🔧 配置和设置

### 日志配置

```python
import logging

# 启用详细日志
logging.basicConfig(level=logging.DEBUG)

# 或配置AssetX特定日志
logger = logging.getLogger('assetx')
logger.setLevel(logging.INFO)
```

### 自定义容差

```python
from assetx import PhysicsValidator

# 设置更严格的验证容差
validator = PhysicsValidator(tolerance=1e-8)

# 或更宽松的容差（适用于单精度数据）
validator = PhysicsValidator(tolerance=1e-4)
```

### 网格处理设置

```python
from assetx import MeshProcessor

processor = MeshProcessor()

# 检查可用的处理后端
if processor.has_trimesh:
    print("Trimesh available for mesh processing")

if processor.has_open3d:
    print("Open3D available for advanced visualization")
```

## 🐛 错误处理

### 常见异常类型

```python
from assetx.core.asset import Asset
from assetx.core.converter import FormatConverter

try:
    asset = Asset("nonexistent.urdf")
    asset.load()
except FileNotFoundError:
    print("Asset file not found")
except ImportError as e:
    print(f"Missing dependency: {e}")
except ValueError as e:
    print(f"Invalid asset format: {e}")

try:
    converter = FormatConverter()
    converter.convert("input.urdf", "unsupported_format")
except ValueError as e:
    print(f"Unsupported conversion: {e}")
except RuntimeError as e:
    print(f"Conversion failed: {e}")
```

### 优雅的降级

```python
from assetx import MeshProcessor, Previewer

# 网格处理的后备方案
try:
    processor = MeshProcessor()
    info = processor.get_mesh_info("mesh.obj")
except ImportError:
    print("Mesh processing not available, install with: pip install 'assetx[mesh]'")
    info = {"error": "trimesh not available"}

# 可视化的后备方案
try:
    previewer = Previewer("open3d")
except ImportError:
    try:
        previewer = Previewer("trimesh")
    except ImportError:
        print("No visualization backend available")
        previewer = None
```

## 📊 性能优化

### 大文件处理

```python
from assetx import Asset
import gc

def process_large_assets(asset_paths):
    """处理大量资产时的内存管理"""
    for asset_path in asset_paths:
        asset = Asset(asset_path)
        asset.load()
        
        # 处理资产
        summary = asset.get_summary()
        print(f"Processed: {summary['path']}")
        
        # 主动释放内存
        del asset
        gc.collect()
```

### 并行处理

```python
from concurrent.futures import ThreadPoolExecutor
from assetx import FormatConverter

def convert_single(args):
    source, target_format, output = args
    converter = FormatConverter()
    return converter.convert(source, target_format, output)

def parallel_convert(file_pairs, max_workers=4):
    """并行转换多个文件"""
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        results = list(executor.map(convert_single, file_pairs))
    return results

# 使用示例
file_pairs = [
    ("robot1.urdf", "mjcf", "robot1.xml"),
    ("robot2.urdf", "mjcf", "robot2.xml"),
]
results = parallel_convert(file_pairs)
```

## 🔍 调试技巧

### 详细信息输出

```python
from assetx import Asset

asset = Asset("robot.urdf")
asset.load()

# 检查加载的数据
print(f"Links: {list(asset.links.keys())}")
print(f"Joints: {list(asset.joints.keys())}")
print(f"Physics properties: {list(asset.physics_properties.keys())}")

# 详细的链接信息
for link_name, link_data in asset.links.items():
    print(f"Link {link_name}:")
    print(f"  Visual elements: {len(link_data.get('visual', []))}")
    print(f"  Collision elements: {len(link_data.get('collision', []))}")
```

### 验证详情

```python
from assetx import PhysicsValidator

validator = PhysicsValidator()
result = validator.validate_asset(asset)

# 详细的验证报告
summary = result.get_summary()
print(f"Validation summary: {summary}")

# 单独检查警告和错误
for warning in result.warnings:
    print(f"⚠️ Warning: {warning}")

for error in result.errors:
    print(f"❌ Error: {error}")
```

## 📖 扩展阅读

- [Asset类详细文档](asset.md)
- [Converter类详细文档](converter.md)
- [高级使用案例](../advanced/README.md)
- [贡献开发指南](../development.md)

---

**需要帮助？** 查看 [FAQ](../faq.md) 或在 [GitHub Issues](https://github.com/jandan138/AssetX/issues) 中提问。
