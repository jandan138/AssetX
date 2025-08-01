#!/usr/bin/env python3
"""
USD 到 URDF 转换器

实现将 USD 格式的机器人文件转换为 URDF 格式。
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import math

from ..asset import Asset
from ..primitives.stage import AssetStage
from ..primitives.prim import AssetPrim


class UsdToUrdfConverter:
    """USD 到 URDF 转换器"""
    
    def __init__(self):
        """初始化转换器"""
        self.robot_name = "converted_robot"
        self.package_name = "robot_description"
        
    def convert(self, usd_path: str, urdf_path: str, robot_name: Optional[str] = None) -> bool:
        """
        转换 USD 文件到 URDF
        
        Args:
            usd_path: USD 文件路径
            urdf_path: 输出 URDF 文件路径
            robot_name: 机器人名称
            
        Returns:
            转换是否成功
        """
        try:
            print(f"🔄 开始转换: {usd_path} → {urdf_path}")
            
            # 设置机器人名称
            if robot_name:
                self.robot_name = robot_name
            
            # 加载 USD 文件
            asset = Asset(usd_path)
            asset.load()
            stage = asset.get_stage()
            
            # 分析 USD 结构
            structure = self._analyze_usd_structure(stage)
            
            # 生成 URDF
            urdf_root = self._create_urdf(structure)
            
            # 保存 URDF 文件
            self._save_urdf(urdf_root, urdf_path)
            
            print(f"✅ 转换成功！URDF 文件已保存到: {urdf_path}")
            return True
            
        except Exception as e:
            print(f"❌ 转换失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _analyze_usd_structure(self, stage: AssetStage) -> Dict:
        """分析 USD 结构"""
        print(f"🔍 分析 USD 结构...")
        
        structure = {
            'root_prim': None,
            'links': [],
            'joints': [],
            'robot_name': self.robot_name
        }
        
        # 获取根 Prim
        root_prim = stage.get_default_prim()
        if root_prim:
            structure['root_prim'] = root_prim
            structure['robot_name'] = root_prim.name
        
        # 遍历所有 Prim，分类收集
        for prim in stage.traverse():
            path_str = str(prim.path)
            
            # 识别关节
            if "joint" in path_str.lower() and prim.type_name == "Joint":
                joint_info = self._extract_joint_info(prim)
                if joint_info:
                    structure['joints'].append(joint_info)
                    
            # 识别链接 (排除 visuals/collisions 子组件)
            elif ("link" in path_str.lower() and 
                  prim.type_name == "Prim" and
                  "visuals" not in path_str and 
                  "collisions" not in path_str):
                link_info = self._extract_link_info(prim, stage)
                if link_info:
                    structure['links'].append(link_info)
        
        print(f"   发现 {len(structure['links'])} 个链接")
        print(f"   发现 {len(structure['joints'])} 个关节")
        
        # 验证和修复结构完整性
        structure = self._validate_and_fix_structure(structure)
        
        return structure
    
    def _validate_and_fix_structure(self, structure: Dict) -> Dict:
        """验证和修复结构完整性"""
        print(f"🔧 验证和修复结构完整性...")
        
        # 收集所有被关节引用的链接名称
        referenced_links = set()
        for joint in structure['joints']:
            if joint['parent']:
                referenced_links.add(joint['parent'])
            if joint['child']:
                referenced_links.add(joint['child'])
        
        # 收集现有的链接名称
        existing_links = {link['name'] for link in structure['links']}
        
        # 找到缺失的链接
        missing_links = referenced_links - existing_links
        
        if missing_links:
            print(f"   发现 {len(missing_links)} 个缺失的链接: {list(missing_links)}")
            
            # 为缺失的链接创建基础定义
            for link_name in missing_links:
                if link_name and link_name != 'world':  # 不为 world 创建链接
                    missing_link = {
                        'name': link_name,
                        'path': f"/{structure['robot_name']}/{link_name}",
                        'mass': 0.1,  # 小质量
                        'inertia': [0.001, 0.0, 0.0, 0.001, 0.0, 0.001],  # 小惯性
                        'origin_xyz': [0.0, 0.0, 0.0],
                        'origin_rpy': [0.0, 0.0, 0.0],
                        'visual_meshes': [],
                        'collision_meshes': []
                    }
                    structure['links'].append(missing_link)
                    print(f"   ✅ 添加缺失链接: {link_name}")
        
        # 修复 rootJoint 问题
        for joint in structure['joints']:
            if 'root' in joint['name'].lower():
                joint['type'] = 'fixed'
                if not joint['parent'] or joint['parent'] == structure['robot_name']:
                    joint['parent'] = 'world'
                if not joint['child']:
                    # 将第一个链接作为子链接
                    if structure['links']:
                        joint['child'] = structure['links'][0]['name']
                print(f"   🔧 修复根关节: {joint['name']}")
        
        # 移除无效的关节（没有有效父子关系的）
        valid_joints = []
        for joint in structure['joints']:
            if joint['parent'] and joint['child'] and joint['parent'] != joint['child']:
                valid_joints.append(joint)
            else:
                print(f"   ❌ 移除无效关节: {joint['name']} (parent: {joint['parent']}, child: {joint['child']})")
        
        structure['joints'] = valid_joints
        
        print(f"   ✅ 最终结构: {len(structure['links'])} 个链接, {len(structure['joints'])} 个关节")
        
        return structure
    
    def _extract_link_info(self, prim: AssetPrim, stage: AssetStage) -> Optional[Dict]:
        """提取链接信息"""
        link_info = {
            'name': prim.name,
            'path': str(prim.path),
            'mass': 1.0,  # 默认质量
            'inertia': [1.0, 0.0, 0.0, 1.0, 0.0, 1.0],  # 默认惯性
            'origin_xyz': [0.0, 0.0, 0.0],
            'origin_rpy': [0.0, 0.0, 0.0],
            'visual_meshes': [],
            'collision_meshes': []
        }
        
        # 提取物理属性
        if hasattr(prim, '_properties'):
            props = prim._properties
            
            # 查找质量
            if 'physics:mass' in props:
                mass_prop = props['physics:mass']
                if hasattr(mass_prop, 'get_value'):
                    link_info['mass'] = float(mass_prop.get_value())
        
        # 查找子组件 (visuals, collisions)
        for child_prim in stage.traverse():
            child_path = str(child_prim.path)
            if child_path.startswith(str(prim.path) + "/"):
                if "visuals" in child_path:
                    # 处理可视化几何体
                    mesh_info = self._extract_mesh_info(child_prim)
                    if mesh_info:
                        link_info['visual_meshes'].append(mesh_info)
                elif "collisions" in child_path:
                    # 处理碰撞几何体
                    mesh_info = self._extract_mesh_info(child_prim)
                    if mesh_info:
                        link_info['collision_meshes'].append(mesh_info)
        
        return link_info
    
    def _extract_joint_info(self, prim: AssetPrim) -> Optional[Dict]:
        """提取关节信息"""
        joint_info = {
            'name': prim.name,
            'path': str(prim.path),
            'type': 'revolute',  # 默认为旋转关节
            'parent': '',
            'child': '',
            'origin_xyz': [0.0, 0.0, 0.0],
            'origin_rpy': [0.0, 0.0, 0.0],
            'axis_xyz': [0.0, 0.0, 1.0],
            'limits': {
                'lower': -3.14159,
                'upper': 3.14159,
                'effort': 100.0,
                'velocity': 1.0
            },
            'dynamics': {
                'damping': 0.0,
                'friction': 0.0
            }
        }
        
        # 确定关节类型
        if "revolute" in prim.name.lower() or prim.type_name == "PhysicsRevoluteJoint":
            joint_info['type'] = 'revolute'
        elif "prismatic" in prim.name.lower() or prim.type_name == "PhysicsPrismaticJoint":
            joint_info['type'] = 'prismatic'
        elif "fixed" in prim.name.lower() or prim.type_name == "PhysicsFixedJoint":
            joint_info['type'] = 'fixed'
        
        # 提取关节属性
        if hasattr(prim, '_properties'):
            props = prim._properties
            
            # 关节位置 - 使用 physics:localPos0
            if 'physics:localPos0' in props:
                pos = props['physics:localPos0']
                if hasattr(pos, 'get_value'):
                    pos_value = pos.get_value()
                    if pos_value and len(pos_value) >= 3:
                        joint_info['origin_xyz'] = [float(pos_value[0]), float(pos_value[1]), float(pos_value[2])]
            
            # 关节旋转 - 使用 physics:localRot0 (四元数转欧拉角)
            if 'physics:localRot0' in props:
                rot = props['physics:localRot0']
                if hasattr(rot, 'get_value'):
                    rot_value = rot.get_value()
                    if rot_value and len(rot_value) >= 4:
                        # 四元数转欧拉角 (简化版本)
                        # 这里需要更完整的四元数到欧拉角转换
                        joint_info['origin_rpy'] = self._quaternion_to_euler(rot_value)
            
            # 关节轴向 - 使用 physics:axis
            if 'physics:axis' in props:
                axis = props['physics:axis']
                if hasattr(axis, 'get_value'):
                    axis_value = axis.get_value()
                    if axis_value:
                        # USD axis 可能是 "X", "Y", "Z" 字符串
                        if isinstance(axis_value, str):
                            if axis_value.upper() == "X":
                                joint_info['axis_xyz'] = [1.0, 0.0, 0.0]
                            elif axis_value.upper() == "Y":
                                joint_info['axis_xyz'] = [0.0, 1.0, 0.0]
                            elif axis_value.upper() == "Z":
                                joint_info['axis_xyz'] = [0.0, 0.0, 1.0]
                        elif len(axis_value) >= 3:
                            joint_info['axis_xyz'] = [float(axis_value[0]), float(axis_value[1]), float(axis_value[2])]
            
            # 关节限制 - 提取真实限制
            if 'physics:lowerLimit' in props:
                limit = props['physics:lowerLimit']
                if hasattr(limit, 'get_value'):
                    limit_value = limit.get_value()
                    if limit_value is not None:
                        # USD 中的角度可能是度数，需要转换为弧度
                        joint_info['limits']['lower'] = math.radians(float(limit_value))
                    
            if 'physics:upperLimit' in props:
                limit = props['physics:upperLimit']
                if hasattr(limit, 'get_value'):
                    limit_value = limit.get_value()
                    if limit_value is not None:
                        joint_info['limits']['upper'] = math.radians(float(limit_value))
            
            # 动力学参数 - 提取真实参数
            if 'drive:angular:physics:damping' in props:
                damping = props['drive:angular:physics:damping']
                if hasattr(damping, 'get_value'):
                    damping_value = damping.get_value()
                    if damping_value is not None:
                        joint_info['dynamics']['damping'] = float(damping_value)
                    
            if 'drive:angular:physics:maxForce' in props:
                force = props['drive:angular:physics:maxForce']
                if hasattr(force, 'get_value'):
                    force_value = force.get_value()
                    if force_value is not None:
                        joint_info['limits']['effort'] = float(force_value)
            
            # 提取父子关系 - 使用 physics:body0/body1
            if 'physics:body0' in props:
                body0 = props['physics:body0']
                if hasattr(body0, 'get_value'):
                    body0_value = body0.get_value()
                    if body0_value:
                        # 从路径中提取链接名称
                        parent_path = str(body0_value)
                        joint_info['parent'] = self._extract_link_name_from_path(parent_path)
            
            if 'physics:body1' in props:
                body1 = props['physics:body1']
                if hasattr(body1, 'get_value'):
                    body1_value = body1.get_value()
                    if body1_value:
                        child_path = str(body1_value)
                        joint_info['child'] = self._extract_link_name_from_path(child_path)
        
        # 如果没有通过属性找到父子关系，使用路径推断（备用方案）
        if not joint_info['parent'] or not joint_info['child']:
            self._infer_joint_relationships(prim, joint_info)
        
        return joint_info
    
    def _quaternion_to_euler(self, quat) -> List[float]:
        """四元数转欧拉角 (简化版本)"""
        try:
            if len(quat) >= 4:
                # quat = [w, x, y, z] 或 [x, y, z, w]
                # 这里假设是 [w, x, y, z] 格式
                w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                
                # 计算欧拉角 (roll, pitch, yaw)
                roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                pitch = math.asin(2 * (w * y - z * x))
                yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                
                return [roll, pitch, yaw]
        except:
            pass
        
        return [0.0, 0.0, 0.0]
    
    def _extract_link_name_from_path(self, path: str) -> str:
        """从 USD 路径中提取链接名称"""
        if not path:
            return ""
        
        # 移除路径前缀，获取最后的名称
        path_parts = path.strip('<>').split('/')
        if path_parts:
            link_name = path_parts[-1]
            # 清理名称中的特殊字符
            return link_name.replace('>', '').replace('<', '').strip()
        
        return ""
    
    def _infer_joint_relationships(self, prim: AssetPrim, joint_info: Dict) -> None:
        """推断关节的父子关系（备用方案）"""
        path_parts = str(prim.path).split('/')
        
        if len(path_parts) >= 3:
            # 关节通常在某个链接下
            parent_link = path_parts[-2]  # 父链接
            joint_info['parent'] = parent_link
            
            # 子链接名称推断
            joint_name = prim.name
            if "joint" in joint_name:
                # 例如 panda_joint1 -> panda_link1
                child_name = joint_name.replace("joint", "link")
                joint_info['child'] = child_name
            
            # 特殊处理 rootJoint
            if "root" in joint_name.lower():
                joint_info['type'] = 'fixed'
                joint_info['parent'] = 'world'  # 或者使用第一个链接作为子链接
                if len(path_parts) >= 2:
                    joint_info['child'] = path_parts[1]  # 机器人根节点
    
    def _extract_mesh_info(self, prim: AssetPrim) -> Optional[Dict]:
        """提取网格信息"""
        mesh_info = {
            'filename': f"package://{self.package_name}/meshes/{prim.name}.dae",
            'origin_xyz': [0.0, 0.0, 0.0],
            'origin_rpy': [0.0, 0.0, 0.0]
        }
        
        # 尝试从 USD references 中获取真实的网格文件路径
        if hasattr(prim, '_properties'):
            props = prim._properties
            
            # 查找几何体相关属性
            for prop_name, prop in props.items():
                if 'filename' in prop_name.lower() or 'file' in prop_name.lower() or 'path' in prop_name.lower():
                    if hasattr(prop, 'get_value'):
                        file_path = prop.get_value()
                        if file_path and isinstance(file_path, str):
                            # 处理 USD 几何文件路径
                            if file_path.endswith(('.dae', '.obj', '.fbx', '.stl')):
                                # 转换为 ROS package 格式
                                filename = file_path.split('/')[-1]  # 获取文件名
                                mesh_info['filename'] = f"package://{self.package_name}/meshes/{filename}"
                                break
        
        # 如果是 references 引用，尝试从路径中获取更具体的名称
        parent_path = str(prim.path)
        if "visuals" in parent_path:
            link_name = parent_path.split('/')[-2]  # 获取父链接名称
            mesh_info['filename'] = f"package://{self.package_name}/meshes/{link_name}_visual.dae"
        elif "collisions" in parent_path:
            link_name = parent_path.split('/')[-2]  # 获取父链接名称
            mesh_info['filename'] = f"package://{self.package_name}/meshes/{link_name}_collision.dae"
        
        return mesh_info
    
    def _create_urdf(self, structure: Dict) -> ET.Element:
        """创建 URDF XML 结构"""
        print(f"🏗️ 生成 URDF 结构...")
        
        # 创建根元素
        robot = ET.Element('robot')
        robot.set('name', structure['robot_name'])
        
        # 添加链接
        for link_info in structure['links']:
            link_elem = self._create_link_element(link_info)
            robot.append(link_elem)
        
        # 添加关节
        for joint_info in structure['joints']:
            joint_elem = self._create_joint_element(joint_info)
            robot.append(joint_elem)
        
        return robot
    
    def _create_link_element(self, link_info: Dict) -> ET.Element:
        """创建 link 元素"""
        link = ET.Element('link')
        link.set('name', link_info['name'])
        
        # 添加惯性信息
        inertial = ET.SubElement(link, 'inertial')
        
        origin = ET.SubElement(inertial, 'origin')
        origin.set('xyz', ' '.join(map(str, link_info['origin_xyz'])))
        origin.set('rpy', ' '.join(map(str, link_info['origin_rpy'])))
        
        mass = ET.SubElement(inertial, 'mass')
        mass.set('value', str(link_info['mass']))
        
        inertia = ET.SubElement(inertial, 'inertia')
        inertia_values = link_info['inertia']
        inertia.set('ixx', str(inertia_values[0]))
        inertia.set('ixy', str(inertia_values[1]))
        inertia.set('ixz', str(inertia_values[2]))
        inertia.set('iyy', str(inertia_values[3]))
        inertia.set('iyz', str(inertia_values[4]))
        inertia.set('izz', str(inertia_values[5]))
        
        # 添加可视化几何体
        for mesh_info in link_info['visual_meshes']:
            visual = ET.SubElement(link, 'visual')
            self._add_geometry_to_element(visual, mesh_info)
        
        # 添加碰撞几何体
        for mesh_info in link_info['collision_meshes']:
            collision = ET.SubElement(link, 'collision')
            self._add_geometry_to_element(collision, mesh_info)
        
        return link
    
    def _create_joint_element(self, joint_info: Dict) -> ET.Element:
        """创建 joint 元素"""
        joint = ET.Element('joint')
        joint.set('name', joint_info['name'])
        joint.set('type', joint_info['type'])
        
        # 原点
        origin = ET.SubElement(joint, 'origin')
        origin.set('xyz', ' '.join(map(str, joint_info['origin_xyz'])))
        origin.set('rpy', ' '.join(map(str, joint_info['origin_rpy'])))
        
        # 父子关节
        parent = ET.SubElement(joint, 'parent')
        parent.set('link', joint_info['parent'])
        
        child = ET.SubElement(joint, 'child')
        child.set('link', joint_info['child'])
        
        # 关节轴 (对于旋转和移动关节)
        if joint_info['type'] in ['revolute', 'prismatic', 'continuous']:
            axis = ET.SubElement(joint, 'axis')
            axis.set('xyz', ' '.join(map(str, joint_info['axis_xyz'])))
        
        # 关节限制
        if joint_info['type'] in ['revolute', 'prismatic']:
            limit = ET.SubElement(joint, 'limit')
            limits = joint_info['limits']
            limit.set('lower', str(limits['lower']))
            limit.set('upper', str(limits['upper']))
            limit.set('effort', str(limits['effort']))
            limit.set('velocity', str(limits['velocity']))
        
        # 动力学参数
        dynamics = ET.SubElement(joint, 'dynamics')
        dyn = joint_info['dynamics']
        dynamics.set('damping', str(dyn['damping']))
        dynamics.set('friction', str(dyn['friction']))
        
        return joint
    
    def _add_geometry_to_element(self, parent: ET.Element, mesh_info: Dict) -> None:
        """添加几何体到元素"""
        origin = ET.SubElement(parent, 'origin')
        origin.set('xyz', ' '.join(map(str, mesh_info['origin_xyz'])))
        origin.set('rpy', ' '.join(map(str, mesh_info['origin_rpy'])))
        
        geometry = ET.SubElement(parent, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        mesh.set('filename', mesh_info['filename'])
    
    def _save_urdf(self, urdf_root: ET.Element, output_path: str) -> None:
        """保存 URDF 文件"""
        # 格式化 XML
        self._indent_xml(urdf_root)
        
        # 创建 XML 树
        tree = ET.ElementTree(urdf_root)
        
        # 保存文件，使用正确的格式
        tree.write(output_path, encoding='utf-8', xml_declaration=True)
    
    def _indent_xml(self, elem: ET.Element, level: int = 0) -> None:
        """格式化 XML 缩进"""
        indent = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = indent + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = indent
            for child in elem:
                self._indent_xml(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = indent
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = indent
