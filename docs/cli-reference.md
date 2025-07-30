# CLI 命令参考

AssetX 提供了功能丰富的命令行界面，支持所有核心功能的便捷调用。

## 全局选项

所有 AssetX 命令都支持以下全局选项：

```bash
assetx [COMMAND] [OPTIONS]

全局选项:
  --help    显示帮助信息
  --version 显示版本信息
  --verbose 启用详细输出
```

---

## `convert` - 格式转换

在不同的机器人仿真格式之间进行转换。

### 语法

```bash
assetx convert [OPTIONS]
```

### 选项

- `--source, -s` (必需): 源文件路径
- `--to, -t` (必需): 目标格式 (urdf|mjcf|usd|genesis)  
- `--output, -o` (必需): 输出文件路径
- `--validate` (可选): 转换后自动验证
- `--backup` (可选): 创建源文件备份

### 支持的转换

| 源格式 | 目标格式 | 状态 |
|--------|----------|------|
| URDF   | MJCF     | ✅ 支持 |
| MJCF   | URDF     | 🔄 开发中 |
| URDF   | USD      | 🔄 开发中 |
| USD    | Genesis  | 🔄 开发中 |

### 示例

```bash
# 基本转换: URDF -> MJCF
assetx convert --source robot.urdf --to mjcf --output robot.xml

# 转换并验证结果
assetx convert -s robot.urdf -t mjcf -o robot.xml --validate

# 转换时创建备份
assetx convert -s robot.urdf -t mjcf -o robot.xml --backup
```

### 输出示例

```
🔄 开始转换: robot.urdf -> mjcf
📁 源文件: robot.urdf (URDF 格式)
🎯 目标格式: MJCF
📝 输出文件: robot.xml

✅ 转换完成
📊 统计信息:
   - 链接数量: 7
   - 关节数量: 6
   - 网格文件: 7 个
   - 转换耗时: 0.25 秒
```

---

## `validate` - 物理验证

验证资产的物理参数和结构完整性，或比较两个资产之间的一致性。

### 语法

```bash
assetx validate [OPTIONS]
```

### 选项

- `--path, -p`: 验证单个资产文件
- `--ref, -r`: 参考资产文件（用于比较）
- `--target, -t`: 目标资产文件（用于比较）
- `--output, -o`: 验证报告输出路径
- `--format`: 报告格式 (text|json|html)

### 验证项目

- ✅ 质量属性检查
- ✅ 惯性矩阵验证
- ✅ 几何体一致性
- ✅ 关节限制检查
- ✅ 碰撞体验证
- ✅ 网格文件存在性

### 示例

```bash
# 验证单个资产
assetx validate --path robot.urdf

# 比较两个资产的一致性
assetx validate --ref robot.urdf --target robot.xml

# 生成详细的验证报告
assetx validate -p robot.urdf -o validation_report.html --format html
```

### 输出示例

```
🔍 验证资产: robot.urdf

📋 验证结果:
✅ 质量属性: 通过
✅ 惯性矩阵: 通过  
✅ 几何体一致性: 通过
✅ 关节限制: 通过
⚠️  碰撞体: 发现 2 个警告
❌ 网格文件: 发现 1 个错误

⚠️  警告信息:
   - link_2: 碰撞体过于复杂，建议简化
   - link_4: 缺少碰撞体定义

❌ 错误信息:  
   - link_1: 网格文件不存在: meshes/link1.dae

🎯 总体评分: 85/100 (良好)
```

---

## `preview` - 可视化预览

在 3D 查看器中预览资产模型。

### 语法

```bash
assetx preview [OPTIONS]
```

### 选项

- `--path, -p` (必需): 资产文件路径
- `--backend, -b`: 渲染后端 (trimesh|open3d|auto)
- `--screenshot, -s`: 保存截图路径
- `--no-gui`: 仅生成截图，不显示GUI

### 示例

```bash
# 预览资产 (自动选择后端)
assetx preview --path robot.urdf

# 使用 Open3D 后端预览
assetx preview -p robot.urdf --backend open3d

# 生成截图
assetx preview -p robot.urdf --screenshot robot_preview.png

# 批量生成截图 (无GUI)
assetx preview -p robot.urdf -s robot.png --no-gui
```

### 输出示例

```
🎨 预览资产: robot.urdf

📊 资产信息:
   - 格式: URDF
   - 链接数: 7
   - 关节数: 6
   - 网格文件: 7 个

🖥️  渲染后端: Open3D
📸 截图保存: robot_preview.png

🚀 启动 3D 查看器...
```

---

## `mesh` - 网格处理

处理和优化 3D 网格文件。

### 语法

```bash
assetx mesh [OPTIONS] COMMAND [ARGS]
```

### 子命令

#### `process` - 处理网格

```bash
assetx mesh process [OPTIONS]
```

**选项:**
- `--input, -i` (必需): 输入网格文件
- `--output, -o`: 输出文件路径
- `--scale`: 缩放因子
- `--units`: 转换单位 (mm|cm|m|inch|ft)
- `--simplify`: 简化网格
- `--target-faces`: 目标面数
- `--center`: 居中网格

#### `simplify` - 简化网格

```bash
assetx mesh simplify [OPTIONS]
```

**选项:**
- `--input, -i` (必需): 输入网格文件
- `--output, -o`: 输出文件路径  
- `--faces, -f` (必需): 目标面数

#### `collision` - 生成碰撞网格

```bash
assetx mesh collision [OPTIONS]
```

**选项:**
- `--visual, -v` (必需): 视觉网格文件
- `--output, -o`: 碰撞网格输出路径
- `--method`: 生成方法 (convex|decomp|simplified)

### 示例

```bash
# 综合处理网格
assetx mesh process -i detailed.obj -o optimized.obj \
  --scale 0.001 --units m --simplify --target-faces 1000 --center

# 简化网格到 500 面
assetx mesh simplify -i complex.obj -o simple.obj --faces 500

# 生成凸包碰撞网格
assetx mesh collision -v visual.obj -o collision.obj --method convex
```

### 输出示例

```
🔧 处理网格: detailed.obj

📊 输入信息:
   - 顶点数: 15,847
   - 面数: 31,694
   - 文件大小: 2.3 MB

⚙️  处理步骤:
✅ 缩放 (0.001x)
✅ 单位转换 (mm -> m)  
✅ 网格简化 (31694 -> 1000 面)
✅ 几何居中

📊 输出信息:
   - 顶点数: 502
   - 面数: 1,000
   - 文件大小: 156 KB
   - 处理耗时: 1.2 秒

💾 保存到: optimized.obj
```

---

## `meta` - 元数据管理

管理资产的元数据、版本信息和标签。

### 语法

```bash
assetx meta [OPTIONS] COMMAND [ARGS]
```

### 子命令

#### `create` - 创建元数据

```bash
assetx meta create [OPTIONS]
```

**选项:**
- `--asset, -a` (必需): 资产文件路径
- `--category, -c`: 资产类别
- `--author`: 作者信息
- `--description, -d`: 描述信息
- `--tags, -t`: 标签列表 (逗号分隔)
- `--version, -v`: 版本号

#### `update` - 更新元数据

```bash
assetx meta update [OPTIONS]
```

**选项:**
- `--meta, -m` (必需): 元数据文件路径
- `--version, -v`: 更新版本号
- `--description, -d`: 更新描述
- `--add-tags`: 添加标签
- `--remove-tags`: 移除标签

#### `show` - 显示元数据

```bash
assetx meta show [OPTIONS]
```

**选项:**
- `--meta, -m` (必需): 元数据文件路径
- `--format, -f`: 输出格式 (yaml|json|table)

### 示例

```bash
# 创建资产元数据
assetx meta create -a robot.urdf -c manipulator \
  --author "AssetX User" -d "六自由度工业机械臂" \
  -t "arm,industrial,6dof" -v "1.0.0"

# 更新版本和描述
assetx meta update -m robot_meta.yaml -v "1.1.0" \
  -d "优化后的六自由度工业机械臂"

# 显示元数据
assetx meta show -m robot_meta.yaml --format table
```

### 输出示例

```
📋 创建元数据: robot_meta.yaml

📊 资产信息:
   - 文件: robot.urdf
   - 格式: URDF
   - 类别: manipulator
   - 版本: 1.0.0
   - 作者: AssetX User

🏷️  标签: arm, industrial, 6dof
📝 描述: 六自由度工业机械臂

✅ 元数据文件已创建
```

---

## `register` - 资产注册

注册资产到本地仓库或远程仓库。

### 语法

```bash
assetx register [OPTIONS]
```

### 选项

- `--asset, -a` (必需): 资产文件路径
- `--repo, -r`: 仓库路径或URL
- `--name, -n`: 注册名称
- `--public`: 公开注册
- `--force`: 强制覆盖

### 示例

```bash
# 注册到本地仓库
assetx register -a robot.urdf -r ./local_repo -n my_robot

# 注册到远程仓库  
assetx register -a robot.urdf -r https://assets.example.com \
  -n industrial_arm --public
```

---

## 配置文件

AssetX 支持使用配置文件简化命令行参数。

### 配置文件位置

- **Windows**: `%USERPROFILE%\.assetx\config.yaml`
- **Linux/macOS**: `~/.assetx/config.yaml`

### 配置文件格式

```yaml
# AssetX 配置文件
defaults:
  backend: "open3d"
  output_format: "text"
  validate_after_convert: true
  
mesh:
  default_target_faces: 1000
  default_units: "meters"
  
conversion:
  create_backup: false
  preserve_materials: true

repositories:
  local: "./assets"
  remote: "https://assetx-repo.com"
```

### 使用配置文件

```bash
# 使用默认配置
assetx convert -s robot.urdf -t mjcf -o robot.xml

# 覆盖配置文件设置
assetx convert -s robot.urdf -t mjcf -o robot.xml --no-validate
```

---

## 批处理脚本

### Windows PowerShell

```powershell
# 批量转换 URDF 文件
Get-ChildItem -Path ".\urdf\" -Filter "*.urdf" | ForEach-Object {
    $output = $_.BaseName + ".xml"
    assetx convert -s $_.FullName -t mjcf -o ".\mjcf\$output"
}
```

### Linux/macOS Bash

```bash
#!/bin/bash
# 批量转换和验证
for urdf_file in ./urdf/*.urdf; do
    filename=$(basename "$urdf_file" .urdf)
    mjcf_file="./mjcf/${filename}.xml"
    
    echo "转换: $urdf_file -> $mjcf_file"
    assetx convert -s "$urdf_file" -t mjcf -o "$mjcf_file" --validate
done
```

---

## 错误码参考

| 错误码 | 含义 | 解决方案 |
|--------|------|----------|
| 1 | 文件不存在 | 检查文件路径 |
| 2 | 格式不支持 | 确认格式支持状态 |
| 3 | 转换失败 | 检查源文件完整性 |
| 4 | 验证失败 | 查看详细错误信息 |
| 5 | 网格处理错误 | 检查网格文件格式 |
| 6 | 权限不足 | 检查文件写入权限 |

---

## 调试技巧

### 启用详细输出

```bash
assetx --verbose convert -s robot.urdf -t mjcf -o robot.xml
```

### 生成调试日志

```bash
export ASSETX_DEBUG=1
assetx convert -s robot.urdf -t mjcf -o robot.xml > debug.log 2>&1
```

### 检查依赖项

```bash
python -c "import assetx; print(assetx.check_dependencies())"
```
