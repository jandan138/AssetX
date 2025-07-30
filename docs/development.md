# AssetX Development Guide

## 快速开始

### 安装开发环境

```bash
# 克隆仓库
git clone https://github.com/your-name/assetx.git
cd assetx

# 创建虚拟环境
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# 安装开发依赖
pip install -e ".[dev,all]"
```

### 运行测试

```bash
# 运行所有测试
pytest

# 运行特定测试
pytest tests/test_basic.py

# 生成覆盖率报告
pytest --cov=assetx --cov-report=html
```

### 代码格式化

```bash
# 格式化代码
black assetx/
isort assetx/

# 类型检查
mypy assetx/

# 代码质量检查
flake8 assetx/
```

## 项目结构详解

```
assetx/
├── core/           # 核心功能模块
│   ├── asset.py    # 统一资产表示
│   ├── converter.py # 格式转换器
│   └── validator.py # 物理验证器
├── mesh/           # 网格处理
│   └── processor.py
├── meta/           # 元数据管理
│   └── manager.py
├── viewer/         # 可视化预览
│   └── preview.py
└── cli.py          # 命令行界面
```

## 添加新的转换器

### 1. 扩展 FormatConverter

```python
# 在 converter.py 中添加新的转换方法
def _mjcf_to_usd(self, asset: Asset, output_path: Path) -> None:
    """MJCF转USD"""
    # 实现转换逻辑
    pass

# 更新支持的转换映射
self.supported_conversions = {
    'mjcf': ['urdf', 'usd'],  # 新增 usd 支持
    # ...
}
```

### 2. 添加格式解析

```python
# 在 asset.py 中添加新格式的加载方法
def _load_new_format(self) -> None:
    """加载新格式"""
    # 解析新格式文件
    # 填充 self.links, self.joints 等数据结构
    pass
```

## 添加新的验证规则

### 扩展 PhysicsValidator

```python
def _validate_new_property(self, asset: Asset, result: ValidationResult) -> None:
    """验证新的物理属性"""
    for link_name, props in asset.physics_properties.items():
        # 添加新的验证逻辑
        if some_condition:
            result.add_error(f"Validation failed for {link_name}")
```

## 添加新的网格操作

### 扩展 MeshProcessor

```python
def new_mesh_operation(self, 
                      mesh_path: Union[str, Path],
                      parameters: dict,
                      output_path: Optional[Union[str, Path]] = None) -> Path:
    """新的网格操作"""
    try:
        import trimesh
        mesh = trimesh.load(str(mesh_path))
        
        # 实现新的操作
        processed_mesh = self._apply_new_operation(mesh, parameters)
        
        # 保存结果
        if output_path is None:
            output_path = mesh_path.with_suffix(f'_processed{mesh_path.suffix}')
        
        processed_mesh.export(str(output_path))
        return output_path
        
    except Exception as e:
        raise RuntimeError(f"New operation failed: {e}")
```

## CLI 命令扩展

### 添加新命令

```python
@main.command()
@click.option('--param', help='New parameter')
def new_command(param):
    """New CLI command description"""
    try:
        # 实现命令逻辑
        click.echo(f"✅ Command completed with param: {param}")
    except Exception as e:
        click.echo(f"❌ Command failed: {e}", err=True)
        raise click.ClickException(str(e))
```

## 最佳实践

### 错误处理

1. **使用特定的异常类型**
```python
class AssetXError(Exception):
    """AssetX 基础异常"""
    pass

class ConversionError(AssetXError):
    """转换错误"""
    pass
```

2. **记录详细的错误信息**
```python
try:
    result = risky_operation()
except Exception as e:
    logger.error(f"Operation failed with input {input_data}: {e}")
    raise ConversionError(f"Failed to convert {source} to {target}: {e}")
```

### 日志记录

```python
import logging

logger = logging.getLogger(__name__)

def some_function():
    logger.info("Starting operation")
    logger.debug(f"Processing with parameters: {params}")
    logger.warning("This might be problematic")
    logger.error("Operation failed")
```

### 类型提示

```python
from typing import Optional, Union, List, Dict
from pathlib import Path

def process_asset(asset_path: Union[str, Path], 
                 options: Optional[Dict[str, str]] = None) -> List[Path]:
    """
    处理资产文件
    
    Args:
        asset_path: 资产文件路径
        options: 可选处理参数
        
    Returns:
        处理后的文件路径列表
        
    Raises:
        FileNotFoundError: 文件不存在
        ValidationError: 验证失败
    """
    pass
```

### 测试编写

```python
import pytest
import tempfile
from pathlib import Path

class TestNewFeature:
    """测试新功能"""
    
    def test_basic_functionality(self):
        """测试基本功能"""
        # Arrange
        input_data = "test_input"
        
        # Act
        result = process_function(input_data)
        
        # Assert
        assert result == expected_output
    
    def test_error_handling(self):
        """测试错误处理"""
        with pytest.raises(ValueError):
            process_function(invalid_input)
    
    def test_with_temp_files(self):
        """使用临时文件测试"""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test.urdf"
            test_file.write_text("test content")
            
            result = process_file(test_file)
            assert result.exists()
```

## 性能优化

### 大文件处理

```python
def process_large_mesh(mesh_path: Path, chunk_size: int = 10000):
    """分块处理大型网格"""
    import trimesh
    
    mesh = trimesh.load(str(mesh_path))
    
    # 分块处理顶点
    for i in range(0, len(mesh.vertices), chunk_size):
        chunk = mesh.vertices[i:i+chunk_size]
        # 处理chunk
```

### 缓存机制

```python
from functools import lru_cache

@lru_cache(maxsize=128)
def expensive_computation(input_hash: str):
    """缓存昂贵的计算结果"""
    # 执行计算
    return result
```

## 发布流程

### 版本更新

1. 更新 `pyproject.toml` 中的版本号
2. 更新 `assetx/__init__.py` 中的 `__version__`
3. 更新 `CHANGELOG.md`

### 构建和发布

```bash
# 构建包
python -m build

# 检查包
python -m twine check dist/*

# 上传到 PyPI
python -m twine upload dist/*
```

## 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/new-feature`)
3. 提交更改 (`git commit -am 'Add new feature'`)
4. 推送到分支 (`git push origin feature/new-feature`)
5. 创建 Pull Request

### Pull Request 要求

- [ ] 代码通过所有测试
- [ ] 添加了适当的测试覆盖
- [ ] 遵循代码风格规范
- [ ] 更新了相关文档
- [ ] 添加了类型提示
- [ ] 通过了 linting 检查
