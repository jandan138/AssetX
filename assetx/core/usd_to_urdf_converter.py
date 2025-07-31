#!/usr/bin/env python3
"""
USD 到 URDF 转换器

实现将 USD 格式的机器人文件转换为 URDF 格式。
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import math

from .asset import Asset
from .stage import AssetStage
from .prim import AssetPrim


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
            
            # 关节位置
            if 'physics:localPos0' in props:
                pos = props['physics:localPos0']
                if hasattr(pos, 'get_value'):
                    joint_info['origin_xyz'] = list(pos.get_value())
            
            # 关节限制
            if 'physics:lowerLimit' in props:
                limit = props['physics:lowerLimit']
                if hasattr(limit, 'get_value'):
                    joint_info['limits']['lower'] = float(limit.get_value())
                    
            if 'physics:upperLimit' in props:
                limit = props['physics:upperLimit']
                if hasattr(limit, 'get_value'):
                    joint_info['limits']['upper'] = float(limit.get_value())
            
            # 动力学参数
            if 'drive:angular:physics:damping' in props:
                damping = props['drive:angular:physics:damping']
                if hasattr(damping, 'get_value'):
                    joint_info['dynamics']['damping'] = float(damping.get_value())
                    
            if 'drive:angular:physics:maxForce' in props:
                force = props['drive:angular:physics:maxForce']
                if hasattr(force, 'get_value'):
                    joint_info['limits']['effort'] = float(force.get_value())
        
        # 推断父子关节关系 (基于路径)
        path_parts = str(prim.path).split('/')
        if len(path_parts) >= 3:
            # 关节通常在某个链接下，其父链接是当前链接，子链接通过关节名推断
            parent_link = path_parts[-2]  # 父链接
            joint_info['parent'] = parent_link
            
            # 子链接名称推断
            joint_name = prim.name
            if "joint" in joint_name:
                # 例如 panda_joint1 -> panda_link1
                child_name = joint_name.replace("joint", "link")
                joint_info['child'] = child_name
        
        return joint_info
    
    def _extract_mesh_info(self, prim: AssetPrim) -> Optional[Dict]:
        """提取网格信息"""
        return {
            'filename': f"package://{self.package_name}/meshes/{prim.name}.dae",
            'origin_xyz': [0.0, 0.0, 0.0],
            'origin_rpy': [0.0, 0.0, 0.0]
        }
    
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
