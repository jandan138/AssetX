# AssetX 测试套件

本目录包含AssetX项目的完整测试套件，按功能和用途进行了分类组织。

## 📁 目录结构

```
tests/
├── __init__.py                 # 测试套件初始化
├── README.md                   # 本文件
├── run_tests.py               # 测试运行器
├── unit/                      # 单元测试
│   ├── __init__.py
│   ├── test_sdf_path.py       # SdfPath类测试
│   ├── test_asset_property.py # AssetProperty系统测试
│   ├── test_asset_prim.py     # AssetPrim类测试
│   └── test_usd_basic.py      # 基础USD功能测试
├── integration/               # 集成测试
│   ├── __init__.py
│   └── test_usd_asset_integration.py  # USD Asset完整集成测试
└── demos/                     # 演示程序
    ├── __init__.py
    └── demo_usd_features.py   # USD功能完整演示
```

## 🧪 测试分类

### Unit Tests (单元测试)
测试单个类和函数的功能，确保每个组件独立工作正常。

- **test_sdf_path.py**: SdfPath类的路径操作测试
- **test_asset_property.py**: AssetAttribute和AssetRelationship的属性系统测试
- **test_asset_prim.py**: AssetPrim的对象管理测试
- **test_usd_basic.py**: 基础USD风格接口测试

### Integration Tests (集成测试)
测试完整的功能流程，确保各组件协同工作正常。

- **test_usd_asset_integration.py**: 完整的USD Asset加载和操作测试

### Demos (演示程序)
展示系统功能的示例代码，可作为使用参考。

- **demo_usd_features.py**: USD风格Asset系统的完整功能演示

## 🚀 运行测试

### 运行所有测试
```bash
cd tests
python run_tests.py
```

### 只运行单元测试
```bash
python run_tests.py --unit
```

### 只运行集成测试
```bash
python run_tests.py --integration
```

### 运行单个测试文件
```bash
python -m unittest unit.test_sdf_path
python -m unittest integration.test_usd_asset_integration
```

### 运行演示程序
```bash
python demos/demo_usd_features.py
```

## 📊 测试覆盖

当前测试覆盖了以下核心功能：

### USD核心概念
- ✅ SdfPath - USD路径系统
- ✅ AssetProperty - 属性系统 (Attribute + Relationship)
- ✅ AssetPrim - 场景对象
- ✅ AssetStage - 场景容器
- ✅ Asset - 主入口接口

### 功能特性
- ✅ URDF格式加载
- ✅ 层次结构管理
- ✅ 属性读写操作
- ✅ Schema应用系统
- ✅ 元数据管理
- ✅ 类型查询和遍历
- ✅ 路径操作和导航

## 🎯 测试目标

1. **正确性**: 确保所有功能按照USD标准正确实现
2. **稳定性**: 验证系统在各种输入条件下的稳定性
3. **性能**: 确保关键操作的性能满足要求
4. **兼容性**: 验证与不同格式文件的兼容性

## 📝 添加新测试

### 添加单元测试
1. 在`unit/`目录下创建`test_*.py`文件
2. 继承`unittest.TestCase`
3. 使用`test_*`前缀命名测试方法
4. 运行测试验证功能

### 添加集成测试
1. 在`integration/`目录下创建`test_*.py`文件
2. 测试完整的用户场景和工作流程
3. 包含多个组件的交互测试

### 添加演示程序
1. 在`demos/`目录下创建演示脚本
2. 包含详细的注释和说明
3. 展示具体的使用场景和最佳实践

## 🔧 测试环境要求

- Python 3.7+
- AssetX核心模块
- unittest (Python标准库)
- 临时文件创建权限 (用于测试文件生成)

## 📈 持续改进

测试套件会随着项目发展持续改进，包括：
- 增加更多边界情况测试
- 提高测试覆盖率
- 添加性能基准测试
- 集成自动化测试流水线
