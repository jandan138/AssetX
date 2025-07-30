# AssetX 高级使用指南

本指南介绍 AssetX 的高级功能和最佳实践，帮助你构建专业的机器人仿真资产处理工作流。

## 自定义工作流

### 工作流设计模式

AssetX 支持多种工作流模式，适应不同的应用场景。

#### 1. 管道式处理

```python
from assetx import *

class AssetProcessingPipeline:
    def __init__(self):
        self.validator = PhysicsValidator()
        self.converter = FormatConverter()
        self.mesh_processor = MeshProcessor()
        self.meta_manager = MetaManager()
        
    def process(self, input_path: str, target_formats: list):
        """完整的资产处理管道"""
        
        # 第1步: 加载和初始验证
        asset = Asset(input_path)
        asset.load()
        
        validation = self.validator.validate_asset(asset)
        if not validation.is_valid:
            return self._handle_validation_errors(validation)
        
        # 第2步: 网格优化
        optimized_meshes = self._optimize_meshes(asset)
        
        # 第3步: 格式转换
        converted_assets = {}
        for target_format in target_formats:
            output_path = self._get_output_path(input_path, target_format)
            if self.converter.convert(input_path, target_format, output_path):
                converted_assets[target_format] = output_path
        
        # 第4步: 创建和更新元数据
        meta_path = self._create_metadata(input_path, converted_assets)
        
        # 第5步: 最终验证
        self._validate_conversions(asset, converted_assets)
        
        return {
            'original': input_path,
            'converted': converted_assets,
            'metadata': meta_path,
            'meshes': optimized_meshes
        }
    
    def _optimize_meshes(self, asset: Asset):
        """优化所有网格文件"""
        optimized = {}
        
        for link in asset.links:
            if hasattr(link, 'visual_mesh') and link.visual_mesh:
                # 视觉网格优化
                optimized_visual = self.mesh_processor.process_mesh(
                    link.visual_mesh,
                    scale=0.001,  # mm to m
                    simplify=True,
                    target_faces=2000
                )
                
                # 生成碰撞网格
                collision_mesh = self.mesh_processor.generate_collision_mesh(
                    optimized_visual
                )
                
                optimized[link.name] = {
                    'visual': optimized_visual,
                    'collision': collision_mesh
                }
        
        return optimized

# 使用示例
pipeline = AssetProcessingPipeline()
result = pipeline.process(
    input_path="complex_robot.urdf",
    target_formats=["mjcf", "usd"]
)
```

#### 2. 批处理模式

```python
import os
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

class BatchProcessor:
    def __init__(self, max_workers=4):
        self.pipeline = AssetProcessingPipeline()
        self.max_workers = max_workers
    
    def process_directory(self, input_dir: str, output_dir: str):
        """批量处理目录中的所有资产"""
        
        input_path = Path(input_dir)
        output_path = Path(output_dir)
        output_path.mkdir(exist_ok=True)
        
        # 查找所有支持的资产文件
        asset_files = []
        for ext in ['.urdf', '.xml', '.usd']:
            asset_files.extend(input_path.glob(f"**/*{ext}"))
        
        print(f"发现 {len(asset_files)} 个资产文件")
        
        # 并行处理
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            futures = []
            
            for asset_file in asset_files:
                future = executor.submit(
                    self._process_single_asset,
                    asset_file,
                    output_path
                )
                futures.append((asset_file, future))
            
            # 收集结果
            results = {}
            for asset_file, future in futures:
                try:
                    result = future.result(timeout=300)  # 5分钟超时
                    results[str(asset_file)] = result
                    print(f"✅ 完成: {asset_file.name}")
                except Exception as e:
                    print(f"❌ 失败: {asset_file.name} - {e}")
                    results[str(asset_file)] = {'error': str(e)}
        
        # 生成批处理报告
        self._generate_batch_report(results, output_path)
        return results
    
    def _process_single_asset(self, asset_file: Path, output_dir: Path):
        """处理单个资产"""
        asset_output_dir = output_dir / asset_file.stem
        asset_output_dir.mkdir(exist_ok=True)
        
        return self.pipeline.process(
            input_path=str(asset_file),
            target_formats=["mjcf", "usd"]
        )

# 使用示例
batch_processor = BatchProcessor(max_workers=8)
results = batch_processor.process_directory(
    input_dir="./robot_assets",
    output_dir="./processed_assets"
)
```

### 配置驱动的工作流

```python
import yaml
from typing import Dict, Any

class ConfigurableWorkflow:
    def __init__(self, config_path: str):
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        self._initialize_processors()
    
    def _initialize_processors(self):
        """根据配置初始化处理器"""
        self.asset_loader = Asset
        self.validator = PhysicsValidator()
        self.converter = FormatConverter()
        self.mesh_processor = MeshProcessor()
        self.meta_manager = MetaManager()
        
        if self.config.get('preview', {}).get('enabled', False):
            backend = self.config['preview'].get('backend', 'auto')
            self.previewer = Previewer(backend=backend)
    
    def execute(self, input_path: str) -> Dict[str, Any]:
        """执行配置定义的工作流"""
        
        workflow_steps = self.config.get('workflow', [])
        context = {'input_path': input_path}
        
        for step in workflow_steps:
            step_name = step['name']
            step_config = step.get('config', {})
            
            print(f"执行步骤: {step_name}")
            
            if step_name == 'load':
                context['asset'] = self._step_load(context, step_config)
            elif step_name == 'validate':
                context['validation'] = self._step_validate(context, step_config)
            elif step_name == 'optimize_meshes':
                context['optimized_meshes'] = self._step_optimize_meshes(context, step_config)
            elif step_name == 'convert':
                context['conversions'] = self._step_convert(context, step_config)
            elif step_name == 'metadata':
                context['metadata'] = self._step_metadata(context, step_config)
            elif step_name == 'preview':
                self._step_preview(context, step_config)
            else:
                print(f"未知步骤: {step_name}")
        
        return context
    
    def _step_load(self, context: Dict, config: Dict):
        """加载资产步骤"""
        asset = self.asset_loader(context['input_path'])
        asset.load()
        return asset
    
    def _step_validate(self, context: Dict, config: Dict):
        """验证步骤"""
        asset = context['asset']
        validation = self.validator.validate_asset(asset)
        
        if not validation.is_valid and config.get('strict', False):
            raise ValueError(f"资产验证失败: {validation.errors}")
        
        return validation
    
    def _step_optimize_meshes(self, context: Dict, config: Dict):
        """网格优化步骤"""
        asset = context['asset']
        optimization_config = config.get('optimization', {})
        
        optimized = {}
        for link in asset.links:
            if hasattr(link, 'visual_mesh') and link.visual_mesh:
                optimized_mesh = self.mesh_processor.process_mesh(
                    link.visual_mesh,
                    **optimization_config
                )
                optimized[link.name] = optimized_mesh
        
        return optimized
    
    def _step_convert(self, context: Dict, config: Dict):
        """格式转换步骤"""
        input_path = context['input_path']
        target_formats = config.get('target_formats', [])
        output_dir = config.get('output_dir', './output')
        
        Path(output_dir).mkdir(exist_ok=True)
        
        conversions = {}
        for target_format in target_formats:
            output_path = self._get_conversion_output_path(
                input_path, target_format, output_dir
            )
            
            if self.converter.convert(input_path, target_format, output_path):
                conversions[target_format] = output_path
        
        return conversions

# 工作流配置文件示例 (workflow.yaml)
workflow_config = """
workflow:
  - name: load
  
  - name: validate
    config:
      strict: false
  
  - name: optimize_meshes
    config:
      optimization:
        scale: 0.001
        units: "meters"
        simplify: true
        target_faces: 1000
  
  - name: convert
    config:
      target_formats: ["mjcf", "usd"]
      output_dir: "./converted"
  
  - name: metadata
    config:
      category: "mobile_robot"
      version: "1.0.0"
  
  - name: preview
    config:
      enabled: true
      backend: "open3d"

preview:
  enabled: true
  backend: "open3d"
"""

# 使用示例
with open('workflow.yaml', 'w') as f:
    f.write(workflow_config)

workflow = ConfigurableWorkflow('workflow.yaml')
result = workflow.execute('robot.urdf')
```

---

## 高级网格处理

### 自定义网格处理管道

```python
class AdvancedMeshProcessor(MeshProcessor):
    def __init__(self):
        super().__init__()
        self.processing_history = []
    
    def create_processing_pipeline(self, operations: list):
        """创建自定义处理管道"""
        def pipeline(mesh_path: str):
            current_mesh = mesh_path
            
            for operation in operations:
                op_name = operation['name']
                op_params = operation.get('params', {})
                
                if op_name == 'scale':
                    current_mesh = self.scale_mesh(current_mesh, **op_params)
                elif op_name == 'simplify':
                    current_mesh = self.simplify_mesh(current_mesh, **op_params)
                elif op_name == 'smooth':
                    current_mesh = self.smooth_mesh(current_mesh, **op_params)
                elif op_name == 'repair':
                    current_mesh = self.repair_mesh(current_mesh, **op_params)
                elif op_name == 'center':
                    current_mesh = self.center_mesh(current_mesh)
                
                self.processing_history.append({
                    'operation': op_name,
                    'input': mesh_path if len(self.processing_history) == 0 else self.processing_history[-1]['output'],
                    'output': current_mesh,
                    'params': op_params
                })
            
            return current_mesh
        
        return pipeline
    
    def smooth_mesh(self, mesh_path: str, iterations: int = 1, lambda_factor: float = 0.5):
        """网格平滑处理"""
        try:
            import trimesh
        except ImportError:
            print("警告: trimesh 未安装，跳过平滑处理")
            return mesh_path
        
        mesh = trimesh.load(mesh_path)
        
        # Laplacian 平滑
        for _ in range(iterations):
            mesh = mesh.smoothed(lambda_factor=lambda_factor)
        
        output_path = self._get_output_path(mesh_path, "smoothed")
        mesh.export(output_path)
        
        return output_path
    
    def repair_mesh(self, mesh_path: str, fill_holes: bool = True, remove_duplicates: bool = True):
        """网格修复"""
        try:
            import trimesh
        except ImportError:
            print("警告: trimesh 未安装，跳过修复处理")
            return mesh_path
        
        mesh = trimesh.load(mesh_path)
        
        if remove_duplicates:
            mesh.remove_duplicate_faces()
            mesh.remove_unreferenced_vertices()
        
        if fill_holes:
            mesh.fill_holes()
        
        output_path = self._get_output_path(mesh_path, "repaired")
        mesh.export(output_path)
        
        return output_path

# 使用示例
processor = AdvancedMeshProcessor()

# 定义处理管道
processing_operations = [
    {'name': 'repair', 'params': {'fill_holes': True, 'remove_duplicates': True}},
    {'name': 'scale', 'params': {'scale_factor': 0.001}},
    {'name': 'smooth', 'params': {'iterations': 2, 'lambda_factor': 0.3}},
    {'name': 'simplify', 'params': {'target_faces': 1000}},
    {'name': 'center'}
]

pipeline = processor.create_processing_pipeline(processing_operations)
final_mesh = pipeline("complex_mesh.obj")

print("处理历史:")
for step in processor.processing_history:
    print(f"  {step['operation']}: {step['input']} -> {step['output']}")
```

### 质量评估和优化

```python
class MeshQualityAssessor:
    def __init__(self):
        self.quality_metrics = {}
    
    def assess_mesh_quality(self, mesh_path: str) -> Dict[str, float]:
        """评估网格质量"""
        try:
            import trimesh
            import numpy as np
        except ImportError:
            return {"error": "依赖库未安装"}
        
        mesh = trimesh.load(mesh_path)
        
        metrics = {
            'vertex_count': len(mesh.vertices),
            'face_count': len(mesh.faces),
            'edge_count': len(mesh.edges),
            'volume': float(mesh.volume) if mesh.is_volume else 0.0,
            'surface_area': float(mesh.area),
            'is_watertight': mesh.is_watertight,
            'is_volume': mesh.is_volume,
            'euler_number': mesh.euler_number,
        }
        
        # 计算质量指标
        if mesh.is_volume:
            metrics['compactness'] = self._calculate_compactness(mesh)
        
        metrics['aspect_ratio'] = self._calculate_aspect_ratio(mesh)
        metrics['triangle_quality'] = self._calculate_triangle_quality(mesh)
        
        self.quality_metrics[mesh_path] = metrics
        return metrics
    
    def _calculate_compactness(self, mesh) -> float:
        """计算紧凑度 (球形度)"""
        import numpy as np
        
        if mesh.volume <= 0:
            return 0.0
        
        # 理想球体的表面积
        ideal_area = (36 * np.pi * mesh.volume**2)**(1/3)
        compactness = ideal_area / mesh.area
        
        return float(compactness)
    
    def _calculate_aspect_ratio(self, mesh) -> float:
        """计算长宽比"""
        bbox = mesh.bounding_box.extents
        max_extent = max(bbox)
        min_extent = min(bbox)
        
        return float(max_extent / min_extent) if min_extent > 0 else float('inf')
    
    def _calculate_triangle_quality(self, mesh) -> float:
        """计算三角形质量 (基于角度分布)"""
        import numpy as np
        
        if not hasattr(mesh, 'face_angles'):
            return 0.0
        
        angles = mesh.face_angles
        
        # 计算角度偏差 (理想三角形每个角60度)
        ideal_angle = np.pi / 3
        angle_deviations = np.abs(angles - ideal_angle)
        mean_deviation = np.mean(angle_deviations)
        
        # 质量评分 (0-1, 1为最好)
        quality = max(0.0, 1.0 - (mean_deviation / ideal_angle))
        
        return float(quality)
    
    def recommend_optimizations(self, mesh_path: str) -> List[str]:
        """推荐优化建议"""
        metrics = self.quality_metrics.get(mesh_path)
        if not metrics:
            metrics = self.assess_mesh_quality(mesh_path)
        
        recommendations = []
        
        # 面数过多
        if metrics['face_count'] > 10000:
            recommendations.append(f"建议简化网格: 当前 {metrics['face_count']} 面，推荐减少到 5000 面以下")
        
        # 非水密
        if not metrics['is_watertight']:
            recommendations.append("建议修复网格: 检测到非水密结构")
        
        # 长宽比过大
        if metrics['aspect_ratio'] > 10:
            recommendations.append(f"建议调整比例: 长宽比 {metrics['aspect_ratio']:.2f} 过大")
        
        # 三角形质量差
        if metrics['triangle_quality'] < 0.5:
            recommendations.append(f"建议重新网格化: 三角形质量 {metrics['triangle_quality']:.2f} 较差")
        
        # 紧凑度低 (仅对体积网格)
        if metrics.get('compactness', 1.0) < 0.3:
            recommendations.append(f"建议优化形状: 紧凑度 {metrics['compactness']:.2f} 较低")
        
        return recommendations

# 使用示例
assessor = MeshQualityAssessor()

# 评估网格质量
metrics = assessor.assess_mesh_quality("robot_link.obj")
print("网格质量指标:")
for key, value in metrics.items():
    print(f"  {key}: {value}")

# 获取优化建议
recommendations = assessor.recommend_optimizations("robot_link.obj")
print("\n优化建议:")
for rec in recommendations:
    print(f"  - {rec}")
```

---

## 自定义验证规则

### 扩展物理验证器

```python
class CustomPhysicsValidator(PhysicsValidator):
    def __init__(self):
        super().__init__()
        self.custom_rules = []
    
    def add_custom_rule(self, rule_func, rule_name: str, severity: str = "error"):
        """添加自定义验证规则"""
        self.custom_rules.append({
            'function': rule_func,
            'name': rule_name,
            'severity': severity
        })
    
    def validate_asset(self, asset: Asset) -> ValidationResult:
        """扩展的验证方法"""
        # 调用基础验证
        result = super().validate_asset(asset)
        
        # 执行自定义规则
        for rule in self.custom_rules:
            try:
                rule_result = rule['function'](asset)
                
                if rule_result is not True:
                    message = f"{rule['name']}: {rule_result}"
                    
                    if rule['severity'] == 'error':
                        result.errors.append(message)
                        result.is_valid = False
                    elif rule['severity'] == 'warning':
                        result.warnings.append(message)
                        
            except Exception as e:
                error_msg = f"{rule['name']} 验证失败: {e}"
                result.errors.append(error_msg)
                result.is_valid = False
        
        return result

# 自定义验证规则示例
def validate_joint_naming_convention(asset: Asset):
    """验证关节命名约定"""
    invalid_joints = []
    
    for joint in asset.joints:
        # 检查命名约定: joint_[type]_[number]
        if not joint.name.startswith('joint_'):
            invalid_joints.append(joint.name)
    
    if invalid_joints:
        return f"关节命名不符合约定: {', '.join(invalid_joints)}"
    
    return True

def validate_mass_distribution(asset: Asset):
    """验证质量分布合理性"""
    total_mass = sum(link.mass for link in asset.links if hasattr(link, 'mass'))
    
    if total_mass < 0.1:
        return "总质量过小，可能存在单位问题"
    
    if total_mass > 1000:
        return "总质量过大，请检查是否合理"
    
    # 检查质量分布
    masses = [link.mass for link in asset.links if hasattr(link, 'mass') and link.mass > 0]
    if masses:
        max_mass = max(masses)
        min_mass = min(masses)
        
        if max_mass / min_mass > 1000:
            return f"质量分布不均匀，最大最小质量比为 {max_mass/min_mass:.1f}"
    
    return True

def validate_link_hierarchy(asset: Asset):
    """验证链接层次结构"""
    # 检查是否有根链接
    root_links = []
    child_links = set()
    
    for joint in asset.joints:
        child_links.add(joint.child_link)
    
    for link in asset.links:
        if link.name not in child_links:
            root_links.append(link.name)
    
    if len(root_links) == 0:
        return "未找到根链接，可能存在循环依赖"
    
    if len(root_links) > 1:
        return f"发现多个根链接: {', '.join(root_links)}"
    
    return True

# 使用示例
validator = CustomPhysicsValidator()

# 添加自定义规则
validator.add_custom_rule(
    rule_func=validate_joint_naming_convention,
    rule_name="关节命名约定检查",
    severity="warning"
)

validator.add_custom_rule(
    rule_func=validate_mass_distribution,
    rule_name="质量分布检查",
    severity="error"
)

validator.add_custom_rule(
    rule_func=validate_link_hierarchy,
    rule_name="链接层次结构检查",
    severity="error"
)

# 验证资产
asset = Asset("robot.urdf")
asset.load()

result = validator.validate_asset(asset)
print(f"验证结果: {'通过' if result.is_valid else '失败'}")

if result.errors:
    print("错误:")
    for error in result.errors:
        print(f"  ❌ {error}")

if result.warnings:
    print("警告:")
    for warning in result.warnings:
        print(f"  ⚠️ {warning}")
```

---

## 插件系统

### 创建自定义插件

```python
from abc import ABC, abstractmethod

class AssetXPlugin(ABC):
    """AssetX 插件基类"""
    
    @abstractmethod
    def get_name(self) -> str:
        """返回插件名称"""
        pass
    
    @abstractmethod
    def get_version(self) -> str:
        """返回插件版本"""
        pass
    
    @abstractmethod
    def get_supported_formats(self) -> List[str]:
        """返回支持的格式列表"""
        pass
    
    @abstractmethod
    def process(self, asset: Asset, **kwargs) -> Any:
        """执行插件功能"""
        pass

class ROSBagExportPlugin(AssetXPlugin):
    """导出到 ROS Bag 的插件"""
    
    def get_name(self) -> str:
        return "ROS Bag Exporter"
    
    def get_version(self) -> str:
        return "1.0.0"
    
    def get_supported_formats(self) -> List[str]:
        return ["urdf"]
    
    def process(self, asset: Asset, bag_path: str = "robot_description.bag", **kwargs) -> str:
        """将 URDF 资产导出为 ROS Bag"""
        try:
            import rosbag
            import rospy
            from std_msgs.msg import String
        except ImportError:
            raise ImportError("需要安装 ROS 才能使用此插件")
        
        bag = rosbag.Bag(bag_path, 'w')
        
        try:
            # 读取 URDF 内容
            with open(asset.file_path, 'r') as f:
                urdf_content = f.read()
            
            # 创建 ROS 消息
            urdf_msg = String()
            urdf_msg.data = urdf_content
            
            # 写入 bag 文件
            bag.write('/robot_description', urdf_msg)
            
            # 添加元数据
            metadata_msg = String()
            metadata_msg.data = f"Generated by AssetX v{asset.__version__}"
            bag.write('/assetx_metadata', metadata_msg)
            
        finally:
            bag.close()
        
        return bag_path

class PluginManager:
    """插件管理器"""
    
    def __init__(self):
        self.plugins = {}
    
    def register_plugin(self, plugin: AssetXPlugin):
        """注册插件"""
        plugin_name = plugin.get_name()
        self.plugins[plugin_name] = plugin
        print(f"注册插件: {plugin_name} v{plugin.get_version()}")
    
    def list_plugins(self) -> List[str]:
        """列出所有插件"""
        return list(self.plugins.keys())
    
    def get_plugin(self, name: str) -> AssetXPlugin:
        """获取插件"""
        return self.plugins.get(name)
    
    def execute_plugin(self, plugin_name: str, asset: Asset, **kwargs):
        """执行插件"""
        plugin = self.get_plugin(plugin_name)
        if not plugin:
            raise ValueError(f"插件不存在: {plugin_name}")
        
        asset_format = asset.get_format()
        supported_formats = plugin.get_supported_formats()
        
        if asset_format not in supported_formats:
            raise ValueError(
                f"插件 {plugin_name} 不支持格式 {asset_format}。"
                f"支持的格式: {', '.join(supported_formats)}"
            )
        
        return plugin.process(asset, **kwargs)

# 使用示例
plugin_manager = PluginManager()

# 注册插件
ros_bag_plugin = ROSBagExportPlugin()
plugin_manager.register_plugin(ros_bag_plugin)

# 使用插件
asset = Asset("robot.urdf")
asset.load()

bag_path = plugin_manager.execute_plugin(
    plugin_name="ROS Bag Exporter",
    asset=asset,
    bag_path="my_robot.bag"
)

print(f"ROS Bag 导出完成: {bag_path}")
```

这个高级使用指南涵盖了：

1. **自定义工作流** - 管道式处理、批处理模式、配置驱动的工作流
2. **高级网格处理** - 自定义处理管道、质量评估和优化
3. **自定义验证规则** - 扩展物理验证器、添加业务逻辑验证
4. **插件系统** - 创建可扩展的插件架构

这些高级功能让 AssetX 能够适应复杂的生产环境和定制化需求。
