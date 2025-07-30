# FAQ - 常见问题解答

本文档收集了 AssetX 用户经常提出的问题和详细解答。

## 基础概念

### Q1: AssetX 是什么？它能解决什么问题？

**A:** AssetX 是一个多格式机器人仿真资产桥梁工具，主要解决以下问题：

1. **格式转换**: 在 URDF、MJCF、USD、Genesis JSON 等格式间自动转换
2. **物理验证**: 检查不同格式间物理参数的一致性
3. **网格处理**: 优化 3D 模型，减少文件大小，提高仿真性能
4. **资产管理**: 统一的元数据管理，版本控制和追踪
5. **可视化**: 便捷的 3D 预览和检查工具

### Q2: AssetX 支持哪些机器人仿真格式？

**A:** 目前支持状态：

| 格式 | 读取 | 写入 | 转换支持 | 状态 |
|------|------|------|----------|------|
| URDF | ✅ | ✅ | 作为中间格式 | 完全支持 |
| MJCF | ✅ | ✅ | URDF ↔ MJCF | 完全支持 |
| USD | 🔄 | 🔄 | 计划中 | 开发中 |
| Genesis JSON | 🔄 | 🔄 | 计划中 | 开发中 |

### Q3: AssetX 与其他工具的区别是什么？

**A:** AssetX 的独特优势：

- **一站式解决方案**: 集成转换、验证、优化、管理功能
- **物理参数保真**: 专注于保持物理参数在转换过程中的一致性
- **智能网格处理**: 自动优化网格，平衡质量和性能
- **元数据管理**: 完整的版本追踪和资产管理
- **教育友好**: 提供渐进式学习资源和教学支持

---

## 安装与配置

### Q4: AssetX 的系统要求是什么？

**A:** 最低系统要求：

- **操作系统**: Windows 10+, Ubuntu 18.04+, macOS 10.14+
- **Python**: 3.8 或更高版本
- **内存**: 4GB RAM (推荐 8GB+)
- **存储**: 1GB 可用空间
- **显卡**: 支持 OpenGL 3.0+ (用于 3D 预览)

推荐配置：
- **CPU**: 多核处理器 (支持并行处理)
- **内存**: 16GB+ (处理大文件)
- **显卡**: 独立显卡 (更好的 3D 渲染体验)

### Q5: 如何选择安装哪些可选依赖？

**A:** 根据您的使用需求选择：

```bash
# 基础安装 (仅核心功能)
pip install assetx

# 网格处理功能
pip install assetx[mesh]  # 添加 trimesh

# 3D 可视化功能  
pip install assetx[viewer]  # 添加 open3d

# URDF 支持
pip install assetx[urdf]  # 添加 urdfpy

# 完整安装 (所有功能)
pip install assetx[all]
```

**选择建议**:
- 初学者: `assetx[all]` (获得完整体验)
- 生产环境: 按需安装 (减小部署包大小)
- CI/CD: `assetx` (基础版本，快速安装)

### Q6: 安装过程中遇到网络问题怎么办？

**A:** 网络问题解决方案：

1. **使用国内镜像源**
```bash
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple assetx
```

2. **离线安装**
```bash
# 在有网络的机器上下载
pip download assetx[all] -d ./packages

# 在目标机器上安装
pip install --no-index --find-links ./packages assetx
```

3. **分步安装**
```bash
# 先安装基础依赖
pip install click pyyaml numpy

# 再安装 AssetX
pip install assetx --no-deps
```

---

## 使用问题

### Q7: 如何开始使用 AssetX？

**A:** 推荐的学习路径：

1. **基础转换** (15分钟)
```bash
# 下载示例文件或使用您的 URDF
assetx convert --source robot.urdf --to mjcf --output robot.xml
```

2. **验证结果** (5分钟)
```bash
# 验证转换质量
assetx validate --ref robot.urdf --target robot.xml
```

3. **预览模型** (5分钟)
```bash
# 3D 预览
assetx preview --path robot.urdf
```

4. **学习 Python API** (30分钟)
```python
from assetx import Asset, FormatConverter

asset = Asset("robot.urdf")
asset.load()

converter = FormatConverter()
converter.convert("robot.urdf", "mjcf", "robot.xml")
```

### Q8: 转换后的文件与原文件有什么区别？

**A:** 转换过程中的变化：

**保持不变的部分**:
- 关节层次结构
- 关节类型和限制
- 质量和惯性属性
- 几何形状定义

**可能变化的部分**:
- 文件格式语法
- 坐标系约定 (根据目标格式调整)
- 材质和纹理表示
- 传感器和控制器定义

**建议**:
- 转换后始终进行验证
- 在仿真器中测试功能
- 对比关键物理参数

### Q9: 如何处理网格文件路径问题？

**A:** 常见网格路径问题及解决方案：

**问题类型**:
1. 绝对路径 vs 相对路径
2. 不同操作系统的路径分隔符
3. 网格文件移动或缺失

**解决方案**:
```python
from assetx import Asset
import xml.etree.ElementTree as ET
from pathlib import Path

def fix_mesh_paths(urdf_path: str, mesh_directory: str):
    """修复 URDF 中的网格路径"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    for mesh in root.findall(".//mesh"):
        old_path = mesh.get("filename")
        if old_path and not Path(old_path).exists():
            # 尝试在指定目录查找
            mesh_name = Path(old_path).name
            new_path = Path(mesh_directory) / mesh_name
            
            if new_path.exists():
                # 使用相对路径
                mesh.set("filename", f"{mesh_directory}/{mesh_name}")
                print(f"修复: {old_path} -> {new_path}")
    
    # 保存修复后的文件
    fixed_path = urdf_path.replace('.urdf', '_fixed.urdf')
    tree.write(fixed_path)
    return fixed_path

# 使用示例
fixed_urdf = fix_mesh_paths("robot.urdf", "./meshes")
```

### Q10: 如何批量处理多个文件？

**A:** 批量处理的几种方法：

**方法1: 命令行批处理 (Bash)**
```bash
#!/bin/bash
for urdf_file in ./robots/*.urdf; do
    filename=$(basename "$urdf_file" .urdf)
    echo "处理: $filename"
    assetx convert -s "$urdf_file" -t mjcf -o "./output/${filename}.xml"
done
```

**方法2: Python 脚本**
```python
from pathlib import Path
from assetx import FormatConverter

def batch_convert(input_dir: str, output_dir: str, target_format: str):
    converter = FormatConverter()
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)
    
    # 查找所有 URDF 文件
    urdf_files = list(input_path.glob("*.urdf"))
    
    for urdf_file in urdf_files:
        output_file = output_path / f"{urdf_file.stem}.xml"
        
        print(f"转换: {urdf_file.name} -> {output_file.name}")
        
        try:
            success = converter.convert(
                str(urdf_file), 
                target_format, 
                str(output_file)
            )
            
            if success:
                print(f"✅ 成功: {urdf_file.name}")
            else:
                print(f"❌ 失败: {urdf_file.name}")
                
        except Exception as e:
            print(f"❌ 错误: {urdf_file.name} - {e}")

# 使用示例
batch_convert("./input_robots", "./output_robots", "mjcf")
```

**方法3: 并行处理**
```python
from concurrent.futures import ThreadPoolExecutor
from assetx import FormatConverter

def convert_single_file(args):
    source, target_format, output = args
    converter = FormatConverter()
    return converter.convert(source, target_format, output)

def parallel_batch_convert(file_list, max_workers=4):
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        results = list(executor.map(convert_single_file, file_list))
    
    success_count = sum(results)
    print(f"批量转换完成: {success_count}/{len(file_list)} 成功")

# 准备文件列表
files_to_convert = [
    ("robot1.urdf", "mjcf", "robot1.xml"),
    ("robot2.urdf", "mjcf", "robot2.xml"),
    # 更多文件...
]

parallel_batch_convert(files_to_convert)
```

---

## 高级功能

### Q11: 如何自定义验证规则？

**A:** 扩展验证功能：

```python
from assetx import PhysicsValidator, Asset
from assetx.core.validator import ValidationResult

class CustomValidator(PhysicsValidator):
    def __init__(self):
        super().__init__()
        
    def validate_joint_naming(self, asset: Asset) -> list:
        """验证关节命名规范"""
        errors = []
        
        for joint in asset.joints:
            # 检查命名规范: joint_[type]_[number]
            if not joint.name.startswith('joint_'):
                errors.append(f"关节命名不规范: {joint.name}")
                
            # 检查重复名称
            joint_names = [j.name for j in asset.joints]
            if joint_names.count(joint.name) > 1:
                errors.append(f"关节名称重复: {joint.name}")
        
        return errors
    
    def validate_mass_distribution(self, asset: Asset) -> list:
        """验证质量分布合理性"""
        errors = []
        
        masses = [link.mass for link in asset.links if hasattr(link, 'mass')]
        if not masses:
            return ["未找到质量信息"]
            
        total_mass = sum(masses)
        if total_mass < 0.1:
            errors.append(f"总质量过小: {total_mass} kg")
        elif total_mass > 1000:
            errors.append(f"总质量过大: {total_mass} kg")
            
        # 检查质量分布均匀性
        max_mass = max(masses)
        min_mass = min(masses)
        if max_mass / min_mass > 100:
            errors.append(f"质量分布不均: 最大/最小 = {max_mass/min_mass:.1f}")
            
        return errors
    
    def validate_asset(self, asset: Asset) -> ValidationResult:
        """扩展的验证方法"""
        # 调用基础验证
        result = super().validate_asset(asset)
        
        # 添加自定义验证
        joint_errors = self.validate_joint_naming(asset)
        mass_errors = self.validate_mass_distribution(asset)
        
        result.errors.extend(joint_errors + mass_errors)
        
        if joint_errors or mass_errors:
            result.is_valid = False
            
        return result

# 使用自定义验证器
validator = CustomValidator()
asset = Asset("robot.urdf")
asset.load()

result = validator.validate_asset(asset)
print(f"验证结果: {'通过' if result.is_valid else '失败'}")
for error in result.errors:
    print(f"  ❌ {error}")
```

### Q12: 如何优化大文件的处理性能？

**A:** 性能优化策略：

**1. 网格优化**
```python
from assetx import MeshProcessor

processor = MeshProcessor()

# 预处理：简化复杂网格
def optimize_meshes_for_simulation(asset_dir: str):
    mesh_files = Path(asset_dir).glob("**/*.obj")
    
    for mesh_file in mesh_files:
        # 检查文件大小
        file_size_mb = mesh_file.stat().st_size / (1024 * 1024)
        
        if file_size_mb > 10:  # 大于 10MB 的文件
            print(f"优化大文件: {mesh_file.name} ({file_size_mb:.1f}MB)")
            
            # 激进简化
            simplified = processor.simplify_mesh(
                str(mesh_file),
                target_faces=min(2000, int(file_size_mb * 100))
            )
            
            # 替换原文件
            mesh_file.replace(mesh_file.with_suffix('.obj.backup'))
            Path(simplified).replace(mesh_file)
```

**2. 内存管理**
```python
import gc
import psutil

class MemoryAwareProcessor:
    def __init__(self, memory_limit_mb=4000):
        self.memory_limit = memory_limit_mb
        
    def check_memory_usage(self):
        """检查内存使用情况"""
        process = psutil.Process()
        memory_mb = process.memory_info().rss / 1024 / 1024
        
        if memory_mb > self.memory_limit:
            print(f"内存使用过高: {memory_mb:.1f}MB, 强制回收")
            gc.collect()
            
        return memory_mb
    
    def process_file_with_memory_control(self, file_path: str):
        """带内存控制的文件处理"""
        self.check_memory_usage()
        
        # 处理文件
        asset = Asset(file_path)
        asset.load()
        
        # 检查内存
        memory_after = self.check_memory_usage()
        
        # 处理完成后清理
        del asset
        gc.collect()
        
        return memory_after
```

**3. 并行处理**
```python
from multiprocessing import Pool
import os

def process_single_robot(args):
    """单个机器人处理函数"""
    robot_path, output_dir = args
    
    try:
        # 在子进程中处理
        from assetx import FormatConverter
        converter = FormatConverter()
        
        output_path = Path(output_dir) / f"{Path(robot_path).stem}.xml"
        success = converter.convert(robot_path, "mjcf", str(output_path))
        
        return robot_path, success, None
        
    except Exception as e:
        return robot_path, False, str(e)

def parallel_process_robots(robot_files: list, output_dir: str):
    """并行处理多个机器人"""
    # 使用 CPU 核心数
    num_workers = min(len(robot_files), os.cpu_count())
    
    args_list = [(robot_file, output_dir) for robot_file in robot_files]
    
    with Pool(processes=num_workers) as pool:
        results = pool.map(process_single_robot, args_list)
    
    # 统计结果
    success_count = sum(1 for _, success, _ in results if success)
    print(f"并行处理完成: {success_count}/{len(robot_files)} 成功")
    
    # 报告错误
    for robot_path, success, error in results:
        if not success:
            print(f"❌ {Path(robot_path).name}: {error}")
```

### Q13: 如何集成 AssetX 到 CI/CD 流程？

**A:** CI/CD 集成示例：

**GitHub Actions 配置** (`.github/workflows/assetx-validation.yml`)
```yaml
name: AssetX Asset Validation

on:
  push:
    paths:
      - 'robots/**/*.urdf'
      - 'robots/**/*.xml'
  pull_request:
    paths:
      - 'robots/**/*.urdf'
      - 'robots/**/*.xml'

jobs:
  validate-assets:
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.9'
    
    - name: Install AssetX
      run: |
        pip install assetx[all]
    
    - name: Validate URDF files
      run: |
        find robots/ -name "*.urdf" -exec assetx validate --path {} \;
    
    - name: Test conversions
      run: |
        mkdir -p converted_assets
        find robots/ -name "*.urdf" | while read urdf_file; do
          base_name=$(basename "$urdf_file" .urdf)
          assetx convert --source "$urdf_file" --to mjcf --output "converted_assets/${base_name}.xml"
        done
    
    - name: Upload conversion results
      uses: actions/upload-artifact@v3
      with:
        name: converted-assets
        path: converted_assets/
```

**Docker 集成**
```dockerfile
# Dockerfile
FROM python:3.9-slim

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# 安装 AssetX
RUN pip install assetx[all]

# 设置工作目录
WORKDIR /workspace

# 复制验证脚本
COPY validate_assets.py /usr/local/bin/
RUN chmod +x /usr/local/bin/validate_assets.py

ENTRYPOINT ["python", "/usr/local/bin/validate_assets.py"]
```

**Jenkins Pipeline**
```groovy
pipeline {
    agent any
    
    stages {
        stage('Setup') {
            steps {
                sh 'python -m venv venv'
                sh 'source venv/bin/activate && pip install assetx[all]'
            }
        }
        
        stage('Validate Assets') {
            steps {
                sh '''
                    source venv/bin/activate
                    find . -name "*.urdf" | xargs -I {} assetx validate --path {}
                '''
            }
        }
        
        stage('Convert Assets') {
            steps {
                sh '''
                    source venv/bin/activate
                    mkdir -p converted
                    find . -name "*.urdf" | while read file; do
                        base=$(basename "$file" .urdf)
                        assetx convert --source "$file" --to mjcf --output "converted/${base}.xml"
                    done
                '''
            }
        }
    }
    
    post {
        always {
            archiveArtifacts artifacts: 'converted/**/*', fingerprint: true
        }
    }
}
```

---

## 故障排除

### Q14: 转换结果在仿真器中表现异常怎么办？

**A:** 系统性故障排除流程：

**第1步: 验证转换质量**
```bash
# 使用 AssetX 验证
assetx validate --ref original.urdf --target converted.xml

# 检查物理参数
assetx validate --path converted.xml --output validation_report.html --format html
```

**第2步: 检查关键参数**
```python
from assetx import Asset

def compare_key_parameters(original_path: str, converted_path: str):
    """比较关键参数"""
    
    original = Asset(original_path)
    converted = Asset(converted_path)
    original.load()
    converted.load()
    
    print("=== 基本信息对比 ===")
    print(f"原始文件 - 链接数: {len(original.links)}, 关节数: {len(original.joints)}")
    print(f"转换文件 - 链接数: {len(converted.links)}, 关节数: {len(converted.joints)}")
    
    print("\n=== 质量对比 ===")
    orig_total_mass = sum(getattr(link, 'mass', 0) for link in original.links)
    conv_total_mass = sum(getattr(link, 'mass', 0) for link in converted.links)
    print(f"原始总质量: {orig_total_mass:.3f} kg")
    print(f"转换总质量: {conv_total_mass:.3f} kg")
    print(f"差异: {abs(orig_total_mass - conv_total_mass):.3f} kg")
    
    print("\n=== 关节限制对比 ===")
    for orig_joint in original.joints:
        # 查找对应关节
        conv_joint = next(
            (j for j in converted.joints if j.name == orig_joint.name), 
            None
        )
        
        if conv_joint:
            print(f"关节 {orig_joint.name}:")
            if hasattr(orig_joint, 'limits') and hasattr(conv_joint, 'limits'):
                print(f"  原始限制: {orig_joint.limits}")
                print(f"  转换限制: {conv_joint.limits}")
        else:
            print(f"⚠️ 关节 {orig_joint.name} 在转换文件中未找到")

# 使用示例
compare_key_parameters("robot.urdf", "robot.xml")
```

**第3步: 仿真器特定检查**

对于 **MuJoCo**:
```bash
# 使用 MuJoCo 自带工具验证
simulate robot.xml

# 检查 MuJoCo 特定警告
python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('robot.xml')
print('模型加载成功，DOF:', model.nv)
"
```

对于 **Gazebo**:
```bash
# 检查 URDF 语法
check_urdf robot.urdf

# 在 Gazebo 中验证
gazebo robot.urdf
```

### Q15: 如何处理大规模资产库的管理？

**A:** 企业级资产管理方案：

**1. 目录结构规范**
```
robot_assets/
├── manufacturers/
│   ├── universal_robots/
│   │   ├── ur5e/
│   │   │   ├── urdf/
│   │   │   ├── mjcf/
│   │   │   ├── meshes/
│   │   │   └── metadata/
│   │   └── ur10e/
│   └── franka_emika/
├── categories/
│   ├── manipulators/
│   ├── mobile_robots/
│   └── humanoids/
└── templates/
    ├── base_templates/
    └── sensor_configs/
```

**2. 资产注册系统**
```python
import sqlite3
import json
from pathlib import Path
from datetime import datetime

class AssetRegistry:
    def __init__(self, db_path: str = "asset_registry.db"):
        self.db_path = db_path
        self._init_database()
    
    def _init_database(self):
        """初始化数据库"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS assets (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT UNIQUE NOT NULL,
                category TEXT NOT NULL,
                manufacturer TEXT,
                version TEXT,
                formats TEXT,  -- JSON string
                file_paths TEXT,  -- JSON string
                metadata_path TEXT,
                created_at TIMESTAMP,
                updated_at TIMESTAMP,
                validation_status TEXT,
                tags TEXT  -- JSON string
            )
        ''')
        
        conn.commit()
        conn.close()
    
    def register_asset(self, asset_info: dict):
        """注册新资产"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        now = datetime.now().isoformat()
        
        cursor.execute('''
            INSERT OR REPLACE INTO assets 
            (name, category, manufacturer, version, formats, file_paths, 
             metadata_path, created_at, updated_at, validation_status, tags)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            asset_info['name'],
            asset_info['category'],
            asset_info.get('manufacturer', ''),
            asset_info.get('version', '1.0.0'),
            json.dumps(asset_info.get('formats', [])),
            json.dumps(asset_info.get('file_paths', {})),
            asset_info.get('metadata_path', ''),
            now, now,
            asset_info.get('validation_status', 'pending'),
            json.dumps(asset_info.get('tags', []))
        ))
        
        conn.commit()
        conn.close()
    
    def search_assets(self, **criteria):
        """搜索资产"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        where_clauses = []
        params = []
        
        for key, value in criteria.items():
            if key in ['name', 'category', 'manufacturer']:
                where_clauses.append(f"{key} LIKE ?")
                params.append(f"%{value}%")
        
        where_sql = " AND ".join(where_clauses) if where_clauses else "1=1"
        
        cursor.execute(f'''
            SELECT name, category, manufacturer, version, formats, file_paths
            FROM assets WHERE {where_sql}
        ''', params)
        
        results = cursor.fetchall()
        conn.close()
        
        return [
            {
                'name': row[0],
                'category': row[1], 
                'manufacturer': row[2],
                'version': row[3],
                'formats': json.loads(row[4]),
                'file_paths': json.loads(row[5])
            }
            for row in results
        ]

# 使用示例
registry = AssetRegistry()

# 注册资产
registry.register_asset({
    'name': 'UR5e',
    'category': 'manipulator',
    'manufacturer': 'Universal Robots',
    'version': '1.0.0',
    'formats': ['urdf', 'mjcf'],
    'file_paths': {
        'urdf': './ur5e/ur5e.urdf',
        'mjcf': './ur5e/ur5e.xml'
    },
    'tags': ['collaborative', '6dof', 'industrial']
})

# 搜索资产
results = registry.search_assets(category='manipulator', manufacturer='Universal')
for asset in results:
    print(f"找到资产: {asset['name']} - {asset['manufacturer']}")
```

**3. 自动化管道**
```python
import schedule
import time
from pathlib import Path

class AssetMaintenancePipeline:
    def __init__(self, asset_root: str):
        self.asset_root = Path(asset_root)
        self.registry = AssetRegistry()
        
    def scan_and_register_new_assets(self):
        """扫描并注册新资产"""
        print("🔍 扫描新资产...")
        
        urdf_files = self.asset_root.rglob("*.urdf")
        
        for urdf_file in urdf_files:
            # 检查是否已注册
            asset_name = urdf_file.stem
            existing = self.registry.search_assets(name=asset_name)
            
            if not existing:
                print(f"发现新资产: {asset_name}")
                self._process_new_asset(urdf_file)
    
    def _process_new_asset(self, urdf_path: Path):
        """处理新发现的资产"""
        from assetx import Asset, FormatConverter, PhysicsValidator
        
        # 加载和验证
        asset = Asset(str(urdf_path))
        asset.load()
        
        validator = PhysicsValidator()
        validation = validator.validate_asset(asset)
        
        # 自动转换到其他格式
        converter = FormatConverter()
        formats = ['urdf']
        file_paths = {'urdf': str(urdf_path)}
        
        # 转换到 MJCF
        mjcf_path = urdf_path.with_suffix('.xml')
        if converter.convert(str(urdf_path), 'mjcf', str(mjcf_path)):
            formats.append('mjcf')
            file_paths['mjcf'] = str(mjcf_path)
        
        # 注册到数据库
        self.registry.register_asset({
            'name': urdf_path.stem,
            'category': self._detect_category(asset),
            'formats': formats,
            'file_paths': file_paths,
            'validation_status': 'valid' if validation.is_valid else 'invalid'
        })
    
    def _detect_category(self, asset):
        """自动检测资产类别"""
        joint_count = len(asset.joints)
        
        if joint_count == 0:
            return 'static_object'
        elif 'wheel' in str(asset.joints).lower():
            return 'mobile_robot'
        elif joint_count >= 6:
            return 'manipulator'
        else:
            return 'mechanism'
    
    def run_maintenance(self):
        """运行维护任务"""
        print("🔧 开始资产维护...")
        self.scan_and_register_new_assets()
        # 其他维护任务...
        print("✅ 维护完成")

# 设置定时任务
pipeline = AssetMaintenancePipeline("./robot_assets")

# 每天凌晨2点执行维护
schedule.every().day.at("02:00").do(pipeline.run_maintenance)

# 保持运行
while True:
    schedule.run_pending()
    time.sleep(3600)  # 每小时检查一次
```

---

## 社区与支持

### Q16: 如何报告 bug 或请求新功能？

**A:** 参与社区的方式：

**报告 Bug**:
1. 访问 [GitHub Issues](https://github.com/jandan138/AssetX/issues)
2. 使用 Bug 报告模板
3. 提供以下信息：
   - 操作系统和 Python 版本
   - AssetX 版本
   - 完整的错误信息
   - 重现步骤和最小示例
   - 相关文件 (如果可能)

**功能请求**:
1. 先在 [Discussions](https://github.com/jandan138/AssetX/discussions) 中讨论
2. 描述使用场景和预期行为
3. 提供相似工具的参考
4. 考虑向后兼容性

### Q17: 如何贡献代码？

**A:** 贡献代码的流程：

1. **Fork 仓库**并创建功能分支
2. **阅读开发指南** (`docs/development.md`)
3. **编写代码**并遵循代码规范
4. **添加测试**确保功能正确
5. **更新文档**反映变更
6. **提交 Pull Request**

**贡献类型**:
- 🐛 Bug 修复
- ✨ 新功能开发
- 📚 文档改进
- 🧪 测试增强
- 🎨 代码优化

### Q18: 有哪些学习资源？

**A:** 推荐的学习资源：

**官方资源**:
- [完整文档](https://github.com/jandan138/AssetX/tree/main/docs)
- [API 参考](https://github.com/jandan138/AssetX/tree/main/docs/api)
- [示例代码](https://github.com/jandan138/AssetX/tree/main/examples)
- [教程视频](https://github.com/jandan138/AssetX/discussions)

**社区资源**:
- GitHub Discussions - 技术讨论
- 用户贡献的示例和工具
- 第三方教程和博客文章

**相关技术学习**:
- **URDF**: [ROS URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- **MuJoCo**: [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- **USD**: [USD Documentation](https://graphics.pixar.com/usd/docs/index.html)
- **机器人学**: 《现代机器人学》等经典教材

---

如果您的问题没有在这里找到答案，请不要犹豫在 [GitHub Discussions](https://github.com/jandan138/AssetX/discussions) 中提问。社区成员很乐意帮助您！
