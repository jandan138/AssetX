# AssetX 快速开始教程

欢迎使用AssetX！这个教程将带你在15分钟内掌握AssetX的核心功能。

## 🎯 学习目标

完成本教程后，你将学会：
- ✅ 转换机器人资产格式
- ✅ 验证物理参数一致性  
- ✅ 处理和优化3D网格
- ✅ 管理资产元数据
- ✅ 使用可视化预览功能

## 📁 准备工作

### 1. 确认安装

```bash
assetx --version
# 输出: assetx, version 0.1.0
```

### 2. 创建工作目录

```bash
mkdir assetx_tutorial
cd assetx_tutorial
```

### 3. 下载示例文件

我们使用AssetX内置的示例：

```bash
# 复制示例文件
cp -r "$(python -c 'import assetx; print(assetx.__path__[0])')/../examples" .
cd examples
```

或者手动创建一个简单的URDF文件：

```xml
<!-- simple_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
  </link>
  
  <link name="arm_link">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## 🔄 第一步：格式转换

### URDF → MJCF

将URDF格式转换为MuJoCo的MJCF格式：

```bash
assetx convert --source simple_robot.urdf --to mjcf --output simple_robot.xml
```

**预期输出：**
```
✅ Conversion completed successfully!
   Source: simple_robot.urdf
   Output: simple_robot.xml
   Time: 0.23s
   Meta data updated
```

**生成的MJCF文件：**
```xml
<?xml version='1.0' encoding='utf-8'?>
<mujoco model="simple_robot">
  <compiler angle="radian" />
  <worldbody>
    <body name="base_link">
      <inertial mass="1.0" />
    </body>
    <body name="arm_link">
      <inertial mass="0.5" />
    </body>
  </worldbody>
</mujoco>
```

### 查看支持的转换

```bash
assetx convert --help
```

支持的格式转换：
- URDF → MJCF ✅
- URDF → USD 🔄
- MJCF → URDF 🔄  
- USD → Genesis 🔄

## ✅ 第二步：验证物理参数

### 验证单个资产

```bash
assetx validate --ref simple_robot.urdf
```

**输出示例：**
```
🔍 Validation Results:
   Asset: simple_robot.urdf
   ✅ Validation passed
   ⚠️ Warnings: 0
   ❌ Errors: 0
   Meta data updated
```

### 比较两个格式

验证转换后的MJCF文件是否与原始URDF保持物理参数一致：

```bash
assetx validate --ref simple_robot.urdf --target simple_robot.xml
```

**输出示例：**
```
🔍 Comparison Results:
   Reference: simple_robot.urdf
   Target: simple_robot.xml
   ✅ Validation passed
   ⚠️ Warnings: 1
     - Mass difference for link 'arm_link': 0.000001
   Meta data updated
```

### 常见验证警告

- **质量差异**: 转换过程中的精度损失
- **惯性矩阵不一致**: 不同格式的计算方法差异
- **关节限制差异**: 单位或范围的微小差别

这些警告通常不影响仿真，但值得注意。

## 🎨 第三步：网格处理

如果你有3D网格文件，可以进行各种处理：

### 获取网格信息

```bash
# 假设你有一个mesh.obj文件
assetx mesh --path mesh.obj --operation info
```

**输出示例：**
```
📊 Mesh Information:
   Path: mesh.obj
   Vertices: 1234
   Faces: 2468
   Volume: 0.045000
   Surface Area: 0.234000
   Watertight: True
```

### 简化网格

为了提高仿真性能，减少面数：

```bash
assetx mesh --path mesh.obj --operation simplify --target-faces 500
```

### 中心化网格

将网格中心移动到原点：

```bash
assetx mesh --path mesh.obj --operation center
```

### 生成碰撞体

从视觉网格生成简化的碰撞网格：

```bash
assetx mesh --path mesh.obj --operation collision --collision-method convex_hull
```

## 📋 第四步：元数据管理

AssetX使用YAML文件跟踪所有资产的元信息。

### 注册新资产

```bash
assetx register \
  --name simple_robot \
  --path simple_robot.urdf \
  --category robot_arm \
  --description "Simple 2-DOF tutorial robot" \
  --tags "tutorial,simple,2dof"
```

**输出：**
```
✅ Asset registered successfully!
   Name: simple_robot
   Path: simple_robot.urdf
   Category: robot_arm
   Format: urdf
   Tags: tutorial, simple, 2dof
```

### 查看资产库

```bash
assetx meta
```

**输出：**
```
📋 Found 2 assets:

  • simple_robot
    Category: robot_arm
    Format: urdf
    Tags: tutorial, simple, 2dof
    Physics Validated: ✅
    Description: Simple 2-DOF tutorial robot

  • panda_arm.urdf
    Category: robot_arm
    Format: urdf
    Tags: 7dof, franka, manipulator, research
    Physics Validated: ✅
    Description: Franka Emika Panda 7-DOF robot arm
```

### 过滤查看

```bash
# 按类别过滤
assetx meta --category robot_arm

# 按标签过滤
assetx meta --tag tutorial

# 按格式过滤
assetx meta --format urdf
```

### 导出报告

```bash
assetx meta --export project_report.yaml
```

生成详细的项目资产报告，包括：
- 资产统计
- 转换历史
- 验证记录
- 格式分布

## 👁️ 第五步：可视化预览

如果安装了可视化依赖：

```bash
# 预览URDF模型（如果有网格文件）
assetx preview --path simple_robot.urdf

# 预览单个网格
assetx preview --path mesh.obj --axes --wireframe
```

这会打开3D查看器显示模型。

## 🔧 实际工作流示例

这是一个完整的工作流，展示如何使用AssetX管理机器人资产：

```bash
# 1. 注册原始资产
assetx register --name my_robot --path robot.urdf --category mobile_robot

# 2. 转换为多种格式
assetx convert --source robot.urdf --to mjcf
assetx convert --source robot.urdf --to usd

# 3. 验证转换质量
assetx validate --ref robot.urdf --target robot.xml

# 4. 优化网格文件
assetx mesh --path robot_mesh.obj --operation simplify --target-faces 1000
assetx mesh --path robot_mesh_simplified.obj --operation collision

# 5. 生成项目报告
assetx meta --export final_report.yaml

# 6. 预览结果
assetx preview --path robot.urdf
```

## 📊 检查你的进度

运行以下命令确认你掌握了核心功能：

```bash
# 1. 转换功能 ✅
assetx convert --source simple_robot.urdf --to mjcf

# 2. 验证功能 ✅
assetx validate --ref simple_robot.urdf --target simple_robot.xml

# 3. 元数据管理 ✅
assetx meta

# 4. 网格处理 ✅ (如果有mesh文件)
assetx mesh --path mesh.obj --operation info

# 5. 帮助系统 ✅
assetx --help
```

## 🎯 下一步学习

恭喜！你已经掌握了AssetX的基本用法。接下来可以：

### 深入学习
- [API文档](api/README.md) - 学习Python API
- [高级用法](advanced/README.md) - 批量处理、自定义转换
- [最佳实践](best-practices.md) - 专业工作流建议

### 实际项目
- [案例研究](case-studies/README.md) - 真实项目示例
- [常见问题](faq.md) - 解决实际问题
- [贡献指南](../CONTRIBUTING.md) - 参与开源开发

### 社区支持
- [GitHub Issues](https://github.com/jandan138/AssetX/issues) - 报告问题
- [GitHub Discussions](https://github.com/jandan138/AssetX/discussions) - 技术讨论
- [Wiki](https://github.com/jandan138/AssetX/wiki) - 社区知识库

## 💡 小贴士

- 🔄 **定期验证**: 每次转换后都运行验证确保质量
- 📋 **元数据优先**: 先注册资产再进行处理，便于追踪
- 🎨 **渐进优化**: 从简单网格开始，逐步优化性能
- 🧪 **小步测试**: 先在小文件上测试，再处理大型资产
- 📚 **记录过程**: 使用 `--export` 生成处理记录

---

**🎉 教程完成！** 你现在已经掌握了AssetX的核心功能。开始在你的项目中使用AssetX吧！
