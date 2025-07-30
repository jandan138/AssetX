# AssetX Meta Schema 元数据规范

## 概述

AssetX 使用 `meta.yaml` 文件来管理资产的元信息，包括语义分类、转换历史、验证记录等。

## 字段规范

### 全局配置 (Root Level)

```yaml
schema_version: "1.0"           # 元数据模式版本
created_at: "2025-01-01T00:00:00"
updated_at: "2025-01-01T12:00:00"

global_settings:
  default_unit: "m"             # 默认长度单位: m, cm, mm
  default_mass_unit: "kg"       # 默认质量单位: kg, g
  coordinate_system: "right_handed"  # 坐标系: right_handed, left_handed

assets: {}                      # 资产字典
collections: {}                 # 资产集合
```

### 资产记录 (Asset Entry)

```yaml
assets:
  panda_arm.urdf:
    # === 基本信息 ===
    name: "panda_arm.urdf"
    description: "Franka Emika Panda 7-DOF robot arm"
    original_format: "urdf"      # 原始格式
    semantic_category: "robot_arm"  # 语义类别
    tags: ["7dof", "franka", "manipulator", "research"]
    
    # === 时间戳 ===
    created_at: "2025-01-01T10:00:00"
    updated_at: "2025-01-01T15:30:00"
    
    # === 物理属性验证状态 ===
    physics_validated: true
    
    # === 转换历史 ===
    conversions:
      mjcf:
        output_path: "./converted/panda_arm.xml"
        conversion_time: 2.34
        success: true
        notes: "Successful URDF to MJCF conversion"
        timestamp: "2025-01-01T14:20:00"
      
      usd:
        output_path: "./converted/panda_arm.usd"
        conversion_time: 5.67
        success: false
        notes: "USD conversion failed: missing material definitions"
        timestamp: "2025-01-01T15:10:00"
    
    # === 验证记录 ===
    validations:
      physics:
        result:
          is_valid: true
          error_count: 0
          warning_count: 2
          warnings: 
            - "Very small mass for link 'panda_hand': 0.0001"
            - "Mass difference for link 'panda_link1': 0.000012"
        compared_with: null
        timestamp: "2025-01-01T14:45:00"
      
      comparison:
        result:
          is_valid: true
          error_count: 0
          warning_count: 1
        compared_with: "panda_arm.xml"
        timestamp: "2025-01-01T15:00:00"
    
    # === 网格信息 ===
    mesh_info:
      visual_meshes:
        - path: "./meshes/visual/link0.obj"
          vertices: 1234
          faces: 2468
          simplified: false
        - path: "./meshes/visual/link1.obj"
          vertices: 890
          faces: 1780
          simplified: true
      
      collision_meshes:
        - path: "./meshes/collision/link0_collision.obj"
          generation_method: "convex_hull"
          original_mesh: "./meshes/visual/link0.obj"
    
    # === 自定义字段 ===
    custom_fields:
      manufacturer: "Franka Emika"
      model_year: 2017
      workspace_radius: 0.855     # 工作空间半径 (m)
      payload_capacity: 3.0       # 载荷能力 (kg)
      joint_limits:
        panda_joint1: [-2.8973, 2.8973]
        panda_joint2: [-1.7628, 1.7628]
        # ... 其他关节限制
      
      simulation_tested:
        isaac_sim: true
        mujoco: true
        sapien: false
        genesis: false
      
      real_robot_validated: true
      sim2real_gap: "minimal"     # minimal, moderate, significant
```

## 语义类别 (Semantic Categories)

推荐的标准化语义类别：

### 机器人类别
- `robot_arm`: 机械臂
- `mobile_robot`: 移动机器人  
- `humanoid`: 人形机器人
- `quadruped`: 四足机器人
- `gripper`: 夹爪/末端执行器
- `industrial_robot`: 工业机器人

### 物体类别  
- `everyday_object`: 日常物品
- `tool`: 工具
- `furniture`: 家具
- `container`: 容器
- `food`: 食物
- `electronic_device`: 电子设备

### 环境类别
- `indoor_scene`: 室内场景
- `outdoor_scene`: 户外场景
- `warehouse`: 仓库环境
- `kitchen`: 厨房环境
- `office`: 办公环境

## 标签系统 (Tags)

建议使用的标准标签：

### 技术标签
- 自由度: `6dof`, `7dof`, `mobile`
- 制造商: `franka`, `ur`, `kuka`, `boston_dynamics`
- 复杂度: `simple`, `medium`, `complex`
- 验证状态: `sim_tested`, `real_tested`, `benchmark`

### 应用标签  
- 用途: `manipulation`, `navigation`, `inspection`, `assembly`
- 场景: `household`, `industrial`, `research`, `educational`
- 数据集: `ycb`, `shapenet`, `partnernet`

## 集合管理 (Collections)

```yaml
collections:
  franka_family:
    name: "Franka Robot Family"
    description: "Complete set of Franka Emika robots and accessories"
    assets: ["panda_arm.urdf", "panda_hand.urdf"]
    maintainer: "AssetX Team"
    
  ycb_objects:
    name: "YCB Object Set"
    description: "Yale-CMU-Berkeley Object and Model Set"
    assets: ["002_master_chef_can.urdf", "003_cracker_box.urdf"]
    source_url: "https://www.ycbbenchmarks.com/"
```

## 最佳实践

1. **一致性**: 统一使用语义类别和标签
2. **可追溯性**: 记录所有转换和验证历史
3. **版本控制**: 配合 git 管理元数据文件
4. **文档化**: 为每个资产提供清晰的描述
5. **标准化**: 遵循单位和坐标系约定

## 工具集成

元数据可以通过以下方式管理：

```bash
# 注册新资产
assetx register --name panda_arm.urdf --path ./panda.urdf --category robot_arm

# 查看资产列表
assetx meta --category robot_arm

# 导出详细报告
assetx meta --export meta_report.yaml
```
