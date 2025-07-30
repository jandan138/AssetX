# AssetX 开发指南

欢迎参与 AssetX 的开发！本指南将帮助您了解项目结构、开发环境配置、贡献流程和代码规范。

## 开发环境设置

### 前置要求

- **Python**: 3.8 或更高版本
- **Git**: 最新版本
- **IDE**: 推荐使用 VSCode 或 PyCharm

### 获取源码

```bash
# 克隆仓库
git clone https://github.com/jandan138/AssetX.git
cd AssetX

# 创建开发分支
git checkout -b feature/your-feature-name
```

### 配置开发环境

```bash
# 创建虚拟环境
python -m venv venv

# 激活虚拟环境
# Windows
venv\Scripts\activate
# Linux/macOS
source venv/bin/activate

# 安装开发依赖
pip install -e ".[dev,all]"

# 安装预提交钩子
pre-commit install
```

### 开发依赖说明

```toml
[project.optional-dependencies]
dev = [
    "pytest>=7.0.0",
    "pytest-cov>=4.0.0",
    "black>=22.0.0",
    "isort>=5.0.0",
    "flake8>=5.0.0",
    "mypy>=1.0.0",
    "pre-commit>=2.20.0",
    "sphinx>=5.0.0",
    "sphinx-rtd-theme>=1.0.0"
]
```

---

## 项目结构详解

```
AssetX/
├── assetx/                 # 主要源码包
│   ├── __init__.py         # 包初始化，版本定义
│   ├── cli.py              # 命令行界面
│   ├── core/               # 核心功能模块
│   │   ├── __init__.py
│   │   ├── asset.py        # 资产表示和加载
│   │   ├── converter.py    # 格式转换引擎
│   │   └── validator.py    # 物理参数验证
│   ├── mesh/               # 网格处理模块
│   │   ├── __init__.py
│   │   └── processor.py    # 网格处理器
│   ├── meta/               # 元数据管理模块
│   │   ├── __init__.py
│   │   └── manager.py      # 元数据管理器
│   └── viewer/             # 可视化模块
│       ├── __init__.py
│       └── previewer.py    # 3D 预览器
├── tests/                  # 测试代码
│   ├── test_asset.py
│   ├── test_converter.py
│   ├── test_validator.py
│   ├── test_mesh.py
│   ├── test_meta.py
│   └── test_cli.py
├── docs/                   # 文档
├── examples/               # 示例代码
├── scripts/                # 工具脚本
├── pyproject.toml          # 项目配置
├── README.md
└── CONTRIBUTING.md
```

### 核心模块架构

```python
# assetx/core/asset.py - 资产抽象
class Asset:
    """统一的资产表示"""
    def __init__(self, file_path: str)
    def load(self) -> None
    def get_format(self) -> str
    def validate(self) -> ValidationResult

# assetx/core/converter.py - 转换引擎
class FormatConverter:
    """多格式转换引擎"""
    def convert(self, source: str, target_format: str, output: str) -> bool
    def urdf_to_mjcf(self, urdf_path: str, mjcf_path: str) -> bool

# assetx/core/validator.py - 验证器
class PhysicsValidator:
    """物理参数验证器"""
    def validate_asset(self, asset: Asset) -> ValidationResult
    def compare_assets(self, asset1: Asset, asset2: Asset) -> ValidationResult
```

---

## 代码规范

### Python 编码规范

遵循 **PEP 8** 和项目特定的代码风格：

```python
# 导入顺序
import os
import sys
from pathlib import Path

import numpy as np
import yaml

from assetx.core.asset import Asset
from assetx.core.converter import FormatConverter

# 类定义
class ExampleProcessor:
    """示例处理器类
    
    这是一个示例类，展示了代码风格规范。
    
    Args:
        config_path: 配置文件路径
        debug: 是否启用调试模式
    
    Attributes:
        config: 加载的配置字典
        is_debug: 调试模式标志
    """
    
    def __init__(self, config_path: str, debug: bool = False):
        self.config = self._load_config(config_path)
        self.is_debug = debug
    
    def process_data(self, input_data: dict) -> dict:
        """处理输入数据
        
        Args:
            input_data: 要处理的数据字典
            
        Returns:
            处理后的数据字典
            
        Raises:
            ValueError: 当输入数据格式不正确时
        """
        if not isinstance(input_data, dict):
            raise ValueError("输入数据必须是字典类型")
        
        result = {}
        
        # 处理逻辑
        for key, value in input_data.items():
            processed_value = self._process_single_item(value)
            result[key] = processed_value
        
        return result
    
    def _load_config(self, config_path: str) -> dict:
        """加载配置文件 (私有方法)"""
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    
    def _process_single_item(self, item):
        """处理单个数据项 (私有方法)"""
        # 实现细节
        return item

# 常量定义
DEFAULT_CONFIG_PATH = "./config/default.yaml"
MAX_RETRY_COUNT = 3
SUPPORTED_FORMATS = ["urdf", "mjcf", "usd", "genesis"]

# 函数定义
def validate_file_path(file_path: str) -> bool:
    """验证文件路径是否有效
    
    Args:
        file_path: 要验证的文件路径
        
    Returns:
        如果路径有效返回 True，否则返回 False
    """
    return Path(file_path).exists() and Path(file_path).is_file()
```

### 类型提示规范

```python
from typing import List, Dict, Optional, Union, Any, Tuple, Callable
from pathlib import Path

# 函数类型提示
def process_assets(
    asset_paths: List[str],
    output_dir: Path,
    config: Optional[Dict[str, Any]] = None
) -> Tuple[bool, List[str]]:
    """处理多个资产文件"""
    pass

# 类属性类型提示
class AssetProcessor:
    def __init__(self):
        self.processed_count: int = 0
        self.error_messages: List[str] = []
        self.config: Optional[Dict[str, Any]] = None
```

### 错误处理规范

```python
# 自定义异常
class AssetXError(Exception):
    """AssetX 基础异常类"""
    pass

class UnsupportedFormatError(AssetXError):
    """不支持的格式异常"""
    pass

class ConversionError(AssetXError):
    """转换过程异常"""
    pass

# 异常处理示例
def load_asset(file_path: str) -> Asset:
    """加载资产文件"""
    try:
        if not Path(file_path).exists():
            raise FileNotFoundError(f"文件不存在: {file_path}")
        
        asset = Asset(file_path)
        asset.load()
        
        return asset
        
    except UnsupportedFormatError as e:
        logger.error(f"不支持的格式: {e}")
        raise
    except Exception as e:
        logger.error(f"加载资产失败: {e}")
        raise AssetXError(f"无法加载资产 {file_path}: {e}")
```

---

## 测试指南

### 测试结构

```python
# tests/test_asset.py
import pytest
import tempfile
from pathlib import Path

from assetx.core.asset import Asset
from assetx.core.validator import ValidationResult

class TestAsset:
    """Asset 类测试"""
    
    @pytest.fixture
    def sample_urdf_path(self):
        """创建临时 URDF 文件"""
        urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>"""
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
            f.write(urdf_content)
            return f.name
    
    def test_asset_creation(self, sample_urdf_path):
        """测试资产对象创建"""
        asset = Asset(sample_urdf_path)
        assert asset.file_path == sample_urdf_path
        assert asset.format is None  # 加载前为 None
    
    def test_asset_loading(self, sample_urdf_path):
        """测试资产加载"""
        asset = Asset(sample_urdf_path)
        asset.load()
        
        assert asset.format == "urdf"
        assert asset.name == "test_robot"
        assert len(asset.links) == 1
    
    def test_format_detection(self, sample_urdf_path):
        """测试格式检测"""
        asset = Asset(sample_urdf_path)
        detected_format = asset.get_format()
        assert detected_format == "urdf"
    
    def test_validation(self, sample_urdf_path):
        """测试资产验证"""
        asset = Asset(sample_urdf_path)
        asset.load()
        
        result = asset.validate()
        assert isinstance(result, ValidationResult)
    
    def test_invalid_file_path(self):
        """测试无效文件路径"""
        with pytest.raises(FileNotFoundError):
            asset = Asset("nonexistent.urdf")
            asset.load()
    
    @pytest.mark.parametrize("format_ext,expected_format", [
        (".urdf", "urdf"),
        (".xml", "mjcf"),
        (".usd", "usd"),
        (".json", "genesis")
    ])
    def test_format_detection_by_extension(self, format_ext, expected_format):
        """参数化测试格式检测"""
        with tempfile.NamedTemporaryFile(suffix=format_ext) as f:
            asset = Asset(f.name)
            assert asset.get_format() == expected_format
```

### 运行测试

```bash
# 运行所有测试
pytest

# 运行特定测试文件
pytest tests/test_asset.py

# 运行特定测试方法
pytest tests/test_asset.py::TestAsset::test_asset_loading

# 生成覆盖率报告
pytest --cov=assetx --cov-report=html

# 运行性能测试
pytest tests/test_performance.py -v
```

### 测试覆盖率要求

- **最低覆盖率**: 80%
- **核心模块覆盖率**: 90%+
- **关键函数覆盖率**: 95%+

---

## 贡献流程

### 1. 创建功能分支

```bash
# 从 main 分支创建新功能分支
git checkout main
git pull origin main
git checkout -b feature/add-usd-support

# 确保分支名称清晰描述功能
# 格式: feature/功能描述, fix/修复描述, docs/文档更新
```

### 2. 开发和测试

```bash
# 进行开发工作
# ...

# 运行测试确保代码质量
pytest

# 运行代码格式检查
black assetx/
isort assetx/
flake8 assetx/

# 运行类型检查
mypy assetx/
```

### 3. 提交代码

```bash
# 添加变更
git add .

# 提交变更 (使用规范的提交信息)
git commit -m "feat(converter): add USD format support

- Add USD to URDF conversion
- Add USD to MJCF conversion  
- Update format detection logic
- Add comprehensive tests for USD support

Fixes #123"
```

### 提交信息规范

使用 [Conventional Commits](https://www.conventionalcommits.org/) 规范：

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

**类型 (type)**:
- `feat`: 新功能
- `fix`: 错误修复
- `docs`: 文档更新
- `style`: 代码格式调整
- `refactor`: 代码重构
- `test`: 测试相关
- `chore`: 构建工具或辅助工具的变动

**示例**:
```
feat(converter): add USD format support
fix(validator): fix mass calculation error
docs(api): update converter documentation
test(asset): add edge case tests
```

### 4. 创建 Pull Request

1. 推送分支到远程仓库
```bash
git push origin feature/add-usd-support
```

2. 在 GitHub 上创建 Pull Request

3. 填写 PR 模板
```markdown
## 功能描述
添加 USD 格式支持，包括 USD 到 URDF 和 USD 到 MJCF 的转换功能。

## 变更内容
- [ ] 添加 USD 解析器
- [ ] 实现 USD 转换逻辑
- [ ] 更新格式检测
- [ ] 添加测试用例
- [ ] 更新文档

## 测试
- [ ] 单元测试通过
- [ ] 集成测试通过
- [ ] 性能测试通过
- [ ] 代码覆盖率满足要求

## 检查清单
- [ ] 代码遵循项目规范
- [ ] 添加了必要的测试
- [ ] 更新了相关文档
- [ ] 通过了 CI 检查
```

### 5. 代码审查

参与代码审查过程：
- 响应审查者的评论
- 根据反馈调整代码
- 确保所有 CI 检查通过

---

## 获取帮助

如果您在开发过程中遇到问题：

1. 查看 [GitHub Issues](https://github.com/jandan138/AssetX/issues)
2. 参与 [GitHub Discussions](https://github.com/jandan138/AssetX/discussions)
3. 查阅项目 [Wiki](https://github.com/jandan138/AssetX/wiki)
4. 联系维护者团队

感谢您对 AssetX 项目的贡献！
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
