# AssetX 故障排除指南

遇到问题时，本指南将帮助您快速诊断和解决常见的 AssetX 使用问题。

## 安装问题

### 依赖安装失败

**问题**: 安装 AssetX 时依赖包下载失败

```bash
ERROR: Could not find a version that satisfies the requirement...
```

**解决方案**:

1. **检查 Python 版本**
```bash
python --version  # 需要 3.8+
```

2. **升级 pip**
```bash
python -m pip install --upgrade pip
```

3. **使用国内镜像源**
```bash
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple assetx
```

4. **分步安装依赖**
```bash
# 先安装核心依赖
pip install click pyyaml numpy

# 再安装 AssetX
pip install assetx
```

### 可选依赖安装问题

**问题**: trimesh 或 open3d 安装失败

**解决方案**:

1. **Windows 用户**
```bash
# 使用预编译版本
pip install trimesh[easy]
pip install open3d --no-deps
```

2. **Linux 用户**
```bash
# 安装系统依赖
sudo apt-get install libegl1-mesa-dev libgl1-mesa-dev
pip install trimesh open3d
```

3. **macOS 用户**
```bash
# 使用 conda
conda install -c conda-forge trimesh open3d
```

4. **离线安装**
```bash
# 下载 wheel 文件离线安装
pip download trimesh open3d
pip install --no-index --find-links ./downloads trimesh open3d
```

---

## 格式转换问题

### URDF 转换失败

**问题**: URDF 文件转换时出错

```
ConversionError: Failed to parse URDF file
```

**诊断步骤**:

1. **检查 URDF 语法**
```bash
# 使用 check_urdf 工具验证
check_urdf robot.urdf

# 或者使用 AssetX 验证
assetx validate --path robot.urdf
```

2. **常见 URDF 问题**
```xml
<!-- 错误：缺少必需属性 -->
<joint type="revolute">  <!-- 缺少 name 属性 -->

<!-- 正确写法 -->
<joint name="joint1" type="revolute">
```

3. **检查文件编码**
```bash
file -i robot.urdf  # 确保是 UTF-8 编码
```

4. **简化测试**
```python
from assetx import Asset

try:
    asset = Asset("robot.urdf")
    asset.load()
    print("✅ URDF 加载成功")
except Exception as e:
    print(f"❌ 错误: {e}")
```

### 网格文件问题

**问题**: 网格文件路径错误或文件不存在

```
FileNotFoundError: mesh file not found: meshes/link1.dae
```

**解决方案**:

1. **检查相对路径**
```python
# 检查网格文件引用
import xml.etree.ElementTree as ET

tree = ET.parse("robot.urdf")
for mesh in tree.findall(".//mesh"):
    filename = mesh.get("filename")
    print(f"网格文件: {filename}")
    
    # 检查文件是否存在
    from pathlib import Path
    if not Path(filename).exists():
        print(f"❌ 文件不存在: {filename}")
```

2. **修复网格路径**
```python
# 自动修复网格路径的脚本
def fix_mesh_paths(urdf_path: str, mesh_dir: str):
    tree = ET.parse(urdf_path)
    
    for mesh in tree.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename and not Path(filename).exists():
            # 尝试在 mesh_dir 中查找文件
            mesh_name = Path(filename).name
            new_path = Path(mesh_dir) / mesh_name
            
            if new_path.exists():
                mesh.set("filename", str(new_path))
                print(f"修复路径: {filename} -> {new_path}")
    
    # 保存修复后的文件
    tree.write(urdf_path.replace('.urdf', '_fixed.urdf'))

fix_mesh_paths("robot.urdf", "./meshes")
```

---

## 性能问题

### 转换速度慢

**问题**: 大文件转换耗时过长

**优化策略**:

1. **网格简化**
```python
from assetx import MeshProcessor

processor = MeshProcessor()

# 在转换前简化网格
simplified = processor.simplify_mesh(
    mesh_path="complex_mesh.obj",
    target_faces=1000  # 减少面数
)
```

2. **批量处理优化**
```python
from concurrent.futures import ThreadPoolExecutor
from assetx import FormatConverter

def convert_file(args):
    source, target_format, output = args
    converter = FormatConverter()
    return converter.convert(source, target_format, output)

# 并行转换多个文件
files_to_convert = [
    ("robot1.urdf", "mjcf", "robot1.xml"),
    ("robot2.urdf", "mjcf", "robot2.xml"),
    # ...
]

with ThreadPoolExecutor(max_workers=4) as executor:
    results = list(executor.map(convert_file, files_to_convert))
```

3. **内存使用监控**
```python
import psutil
import os

def monitor_memory():
    process = psutil.Process(os.getpid())
    memory_mb = process.memory_info().rss / 1024 / 1024
    print(f"内存使用: {memory_mb:.1f} MB")

# 在处理过程中监控内存
monitor_memory()
# 执行转换操作
monitor_memory()
```

### 内存不足

**问题**: 处理大文件时内存溢出

```
MemoryError: Unable to allocate memory
```

**解决方案**:

1. **流式处理**
```python
# 分块处理大文件
def process_large_file(file_path: str, chunk_size: int = 1000):
    with open(file_path, 'r') as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            # 处理块数据
            process_chunk(chunk)
```

2. **临时文件管理**
```python
import tempfile
import shutil

# 使用临时目录
with tempfile.TemporaryDirectory() as temp_dir:
    # 在临时目录中处理文件
    temp_file = Path(temp_dir) / "temp_output.xml"
    # 处理完成后移动到目标位置
    shutil.move(str(temp_file), "final_output.xml")
```

---

## 验证问题

### 物理参数验证失败

**问题**: 验证报告大量物理参数错误

```
ValidationError: Mass matrix is not positive definite
```

**调试步骤**:

1. **检查质量分布**
```python
from assetx import Asset, PhysicsValidator

asset = Asset("robot.urdf")
asset.load()

# 检查每个链接的质量
for link in asset.links:
    if hasattr(link, 'mass'):
        print(f"链接 {link.name}: 质量 = {link.mass}")
        if link.mass <= 0:
            print(f"❌ 链接 {link.name} 质量为 {link.mass}，应该 > 0")
```

2. **修复质量问题**
```python
def fix_mass_properties(urdf_path: str):
    tree = ET.parse(urdf_path)
    
    for mass_elem in tree.findall(".//mass"):
        value = float(mass_elem.get("value", 0))
        if value <= 0:
            # 设置默认质量
            mass_elem.set("value", "1.0")
            print(f"修复质量: {value} -> 1.0")
    
    # 保存修复后的文件
    tree.write(urdf_path.replace('.urdf', '_mass_fixed.urdf'))
```

3. **检查惯性矩阵**
```python
def validate_inertia_matrix(ixx, iyy, izz, ixy, ixz, iyz):
    """检查惯性矩阵是否合理"""
    # 对角线元素应为正
    if ixx <= 0 or iyy <= 0 or izz <= 0:
        return False, "对角线元素必须为正"
    
    # 检查三角不等式 (简化检查)
    if ixx + iyy < izz or iyy + izz < ixx or izz + ixx < iyy:
        return False, "不满足三角不等式"
    
    return True, "惯性矩阵合理"

# 使用示例
is_valid, message = validate_inertia_matrix(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
print(f"惯性矩阵验证: {message}")
```

---

## 可视化问题

### 预览窗口无法打开

**问题**: 3D 预览窗口启动失败

```
RuntimeError: Cannot initialize OpenGL context
```

**解决方案**:

1. **检查图形驱动**
```bash
# Linux 检查 OpenGL 支持
glxinfo | grep "OpenGL version"

# 更新图形驱动
sudo apt-get update
sudo apt-get install mesa-utils
```

2. **使用无头模式**
```python
from assetx import Previewer

# 无头模式生成截图
previewer = Previewer(backend='trimesh')
previewer.take_screenshot(asset, "preview.png")
```

3. **虚拟显示器 (Linux)**
```bash
# 安装 xvfb
sudo apt-get install xvfb

# 使用虚拟显示器
xvfb-run -a python your_script.py
```

### 网格显示异常

**问题**: 模型显示不正确或缺失部分

**诊断方法**:

1. **检查网格完整性**
```python
import trimesh

mesh = trimesh.load("model.obj")
print(f"顶点数: {len(mesh.vertices)}")
print(f"面数: {len(mesh.faces)}")
print(f"是否水密: {mesh.is_watertight}")
print(f"是否有体积: {mesh.is_volume}")

# 检查网格问题
if not mesh.is_watertight:
    print("❌ 网格不是水密的，可能有孔洞")

if mesh.volume <= 0:
    print("❌ 网格体积为负或零")
```

2. **修复网格**
```python
from assetx import MeshProcessor

processor = MeshProcessor()

# 修复网格问题
repaired_mesh = processor.repair_mesh(
    mesh_path="broken_mesh.obj",
    fill_holes=True,
    remove_duplicates=True
)
```

---

## 命令行问题

### CLI 命令无法识别

**问题**: 安装后 `assetx` 命令不可用

```bash
assetx: command not found
```

**解决方案**:

1. **检查安装方式**
```bash
# 查看安装位置
pip show assetx

# 检查可执行文件
python -m assetx --version
```

2. **PATH 环境变量**
```bash
# Windows (添加到 PATH)
set PATH=%PATH%;C:\Python39\Scripts

# Linux/macOS
export PATH=$PATH:~/.local/bin
```

3. **使用模块方式**
```bash
# 如果命令不可用，使用模块方式
python -m assetx convert --source robot.urdf --to mjcf --output robot.xml
```

### 配置文件问题

**问题**: 配置文件无法加载

```
ConfigError: Cannot load configuration file
```

**解决方案**:

1. **检查配置文件格式**
```yaml
# 确保 YAML 格式正确
defaults:
  backend: "open3d"  # 注意引号
  validate_after_convert: true  # 布尔值不加引号

mesh:
  default_target_faces: 1000  # 数字不加引号
```

2. **验证 YAML 语法**
```python
import yaml

try:
    with open('config.yaml', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    print("✅ 配置文件格式正确")
except yaml.YAMLError as e:
    print(f"❌ YAML 格式错误: {e}")
```

---

## 调试技巧

### 启用详细日志

```python
import logging

# 设置日志级别
logging.basicConfig(level=logging.DEBUG)

# 或者设置环境变量
import os
os.environ['ASSETX_LOG_LEVEL'] = 'DEBUG'
```

### 使用调试模式

```bash
# 启用调试输出
assetx --verbose convert --source robot.urdf --to mjcf --output robot.xml

# 生成调试日志
assetx convert --source robot.urdf --to mjcf --output robot.xml > debug.log 2>&1
```

### 交互式调试

```python
# 在代码中添加断点
import pdb; pdb.set_trace()

# 或者使用 IPython
import IPython; IPython.embed()

# 检查对象状态
print(f"Asset format: {asset.format}")
print(f"Links count: {len(asset.links)}")
print(f"Joints count: {len(asset.joints)}")
```

---

## 获取帮助

### 自助诊断

1. **运行诊断脚本**
```python
from assetx import Asset, FormatConverter, PhysicsValidator

def run_diagnostics():
    """运行系统诊断"""
    print("🔍 AssetX 系统诊断")
    print("="*50)
    
    # 检查依赖
    try:
        import numpy
        print("✅ NumPy 可用")
    except ImportError:
        print("❌ NumPy 不可用")
    
    try:
        import trimesh
        print("✅ Trimesh 可用")
    except ImportError:
        print("⚠️ Trimesh 不可用 (可选)")
    
    try:
        import open3d
        print("✅ Open3D 可用")
    except ImportError:
        print("⚠️ Open3D 不可用 (可选)")
    
    # 测试核心功能
    try:
        converter = FormatConverter()
        print("✅ 格式转换器初始化成功")
    except Exception as e:
        print(f"❌ 格式转换器初始化失败: {e}")
    
    try:
        validator = PhysicsValidator()
        print("✅ 物理验证器初始化成功")
    except Exception as e:
        print(f"❌ 物理验证器初始化失败: {e}")

if __name__ == "__main__":
    run_diagnostics()
```

2. **检查系统环境**
```bash
# 系统信息
python --version
pip --version

# AssetX 信息
python -c "import assetx; print(assetx.__version__)"

# 依赖版本
pip list | grep -E "(numpy|trimesh|open3d|click|pyyaml)"
```

### 寻求帮助

如果问题仍未解决：

1. **GitHub Issues**: [报告问题](https://github.com/jandan138/AssetX/issues)
2. **讨论社区**: [参与讨论](https://github.com/jandan138/AssetX/discussions)
3. **文档**: [查阅完整文档](https://assetx.readthedocs.io/)

**报告问题时请提供**:
- 操作系统版本
- Python 版本  
- AssetX 版本
- 完整的错误信息
- 重现问题的最小示例
- 相关文件 (如有可能)

---

## 常见错误代码

| 错误码 | 含义 | 解决方案 |
|--------|------|----------|
| AX001 | 文件不存在 | 检查文件路径 |
| AX002 | 格式不支持 | 确认格式支持状态 |
| AX003 | 转换失败 | 检查源文件完整性 |
| AX004 | 验证失败 | 查看详细验证报告 |
| AX005 | 网格处理错误 | 检查网格文件格式 |
| AX006 | 权限不足 | 检查文件写入权限 |
| AX007 | 内存不足 | 减少处理文件大小 |
| AX008 | 依赖缺失 | 安装相关依赖 |

通过本指南，您应该能够解决大部分 AssetX 使用过程中遇到的问题。如果问题依然存在，请不要犹豫寻求社区帮助！
