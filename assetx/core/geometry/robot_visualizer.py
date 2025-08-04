#!/usr/bin/env python3
"""
机器人可视化器 - 通用机器人整体渲染逻辑
支持任意类型机器人的可视化，包括：
- 工业机械臂
- 人形机器人
- 四足机器人
- 轮式机器人
- 其他自定义机器人
"""

from typing import Dict, List, Optional, Tuple, Set, TYPE_CHECKING
import logging
import numpy as np

if TYPE_CHECKING:
    from ...asset import Asset
    from ..primitives import AssetPrim

from .plotting import MatplotlibPlotter

logger = logging.getLogger(__name__)


class RobotVisualizer:
    """通用机器人可视化器
    
    采用基于USD数据的通用算法，能够可视化任意类型的机器人：
    - 自动解析机器人树状结构
    - 从USD数据中提取真实的位置和几何信息
    - 智能推断连接关系
    - 支持多种几何形状和材质
    """

    def __init__(self, asset: "Asset"):
        """初始化机器人可视化器
        
        Args:
            asset: 所属的Asset对象
        """
        self.asset = asset
        self.plotter = MatplotlibPlotter()
        self._link_tree = {}  # 存储链接的树状结构
        self._joint_connections = {}  # 存储关节连接关系
        
    def render_complete_robot(self, show_joints: bool = True, show_frames: bool = True, 
                            auto_scale: bool = True) -> bool:
        """渲染完整机器人
        
        Args:
            show_joints: 是否显示关节连接
            show_frames: 是否显示坐标系
            auto_scale: 是否自动缩放场景以适应机器人尺寸
            
        Returns:
            是否渲染成功
        """
        try:
            # 创建3D图形
            fig, ax = self.plotter.create_3d_figure("AssetX Robot Visualization", (12, 10))
            if fig is None or ax is None:
                logger.error("无法创建3D图形")
                return False
            
            # 获取机器人结构
            links = self.asset.query.get_links()
            joints = self.asset.query.get_joints()
            
            if not links:
                logger.error("No links found in robot")
                return False
            
            logger.info(f"Rendering robot with {len(links)} links and {len(joints)} joints")
            
            # 构建机器人树状结构
            self._build_robot_structure(links, joints)
            
            # 计算链接的真实空间位置
            link_positions = self._calculate_real_link_positions(links, joints)
            
            # 渲染每个链接（使用真实几何数据）
            self._render_all_links_with_geometry(ax, links, link_positions)
            
            # 显示关节连接线（基于真实连接关系）
            if show_joints and joints:
                self._render_real_joint_connections(ax, link_positions, joints)
            
            # 显示坐标系
            if show_frames:
                self._render_coordinate_frames(ax, link_positions)
            
            # 自动缩放场景
            if auto_scale:
                max_range = self._calculate_scene_bounds(link_positions)
                self.plotter.setup_3d_scene(ax, max_range=max_range)
            else:
                self.plotter.setup_3d_scene(ax, max_range=1.0)
                
            self.plotter.finalize_plot(ax, show_legend=True)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to render complete robot: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _build_robot_structure(self, links: List["AssetPrim"], joints: List["AssetPrim"]):
        """构建机器人的树状结构
        
        Args:
            links: 链接列表
            joints: 关节列表
        """
        self._link_tree = {}
        self._joint_connections = {}
        
        # 为每个链接初始化节点
        for link in links:
            self._link_tree[link.name] = {
                'prim': link,
                'parent': None,
                'children': [],
                'joints': []
            }
        
        # 调试：打印关节信息
        logger.info("=== 调试关节信息 ===")
        for joint in joints:
            logger.info(f"关节: {joint.name}")
            self._debug_joint_attributes(joint)
        
        # 解析关节连接关系
        for joint in joints:
            try:
                # 尝试获取关节的父子链接
                parent_link = self._get_joint_parent_link(joint)
                child_link = self._get_joint_child_link(joint)
                
                logger.info(f"关节 {joint.name}: 父链接={parent_link}, 子链接={child_link}")
                
                if parent_link and child_link:
                    # 建立父子关系
                    if parent_link in self._link_tree and child_link in self._link_tree:
                        self._link_tree[child_link]['parent'] = parent_link
                        self._link_tree[parent_link]['children'].append(child_link)
                        self._link_tree[parent_link]['joints'].append(joint)
                        
                        self._joint_connections[joint.name] = {
                            'parent_link': parent_link,
                            'child_link': child_link,
                            'joint': joint
                        }
                        logger.info(f"✓ 成功建立连接: {parent_link} -> {child_link}")
                        
            except Exception as e:
                logger.warning(f"Failed to parse joint {joint.name}: {e}")
                continue
    
    def _debug_joint_attributes(self, joint: "AssetPrim"):
        """调试关节属性"""
        try:
            logger.info(f"  关节类型: {getattr(joint, 'type_name', 'Unknown')}")
            
            # 列出所有属性
            if hasattr(joint, 'get_property_names'):
                properties = joint.get_property_names()
                logger.info(f"  属性列表: {properties}")
                
            # 尝试常见的关节属性
            common_attrs = [
                'parent', 'child', 'parent_link', 'child_link',
                'physics:body0', 'physics:body1', 'body0', 'body1',
                'parentLink', 'childLink', 'rel:parent', 'rel:child'
            ]
            
            for attr_name in common_attrs:
                if hasattr(joint, 'get_attribute'):
                    attr = joint.get_attribute(attr_name)
                    if attr:
                        value = attr.get() if hasattr(attr, 'get') else attr
                        logger.info(f"  {attr_name}: {value}")
                        
        except Exception as e:
            logger.debug(f"  调试属性失败: {e}")
    
    def _get_joint_parent_link(self, joint: "AssetPrim") -> Optional[str]:
        """获取关节的父链接 - 针对USD PhysicsJoint"""
        try:
            # 首先检查关系(Relationship) - USD Physics关节的标准方式
            rel_names = ['physics:body0', 'body0', 'parentBody', 'parent']
            
            for rel_name in rel_names:
                if hasattr(joint, 'get_relationship'):
                    try:
                        rel = joint.get_relationship(rel_name)
                        if rel:
                            targets = rel.get_targets()
                            if targets:
                                # 获取第一个目标路径
                                target_path = str(targets[0])
                                # 提取链接名（去掉路径前缀）
                                link_name = target_path.split('/')[-1]
                                logger.debug(f"从关系 {rel_name} 获取父链接: {link_name}")
                                return link_name
                    except Exception as e:
                        logger.debug(f"解析关系 {rel_name} 失败: {e}")
                        continue
            
            # 备选：尝试属性方法（兼容其他格式）
            attr_names = [
                'physics:body0', 'parent_link', 'parentLink', 'body0',
                'parent', 'rel:parent', 'target0', 'link0',
                'urdf:parent_link', 'urdf:parent', 'parentBody'
            ]
            
            for attr_name in attr_names:
                if hasattr(joint, 'get_attribute'):
                    attr = joint.get_attribute(attr_name)
                    if attr:
                        try:
                            value = attr.get()
                            if value:
                                # 提取链接名（去掉路径前缀）
                                link_name = str(value).split('/')[-1]
                                if link_name:
                                    logger.debug(f"从属性 {attr_name} 获取父链接: {link_name}")
                                    return link_name
                        except Exception as e:
                            logger.debug(f"解析属性 {attr_name} 失败: {e}")
                            continue
            
            # 最后：从关节名称推断
            joint_name = joint.name.lower()
            if 'joint' in joint_name:
                # 对于 panda_joint1，父链接通常是 panda_link0
                joint_num = ''.join(filter(str.isdigit, joint_name))
                if joint_num:
                    parent_num = max(0, int(joint_num) - 1)
                    parent_link = f"panda_link{parent_num}"
                    logger.debug(f"从关节名推断父链接: {parent_link}")
                    return parent_link
                    
        except Exception as e:
            logger.debug(f"获取父链接失败: {e}")
        return None
    
    def _get_joint_child_link(self, joint: "AssetPrim") -> Optional[str]:
        """获取关节的子链接 - 针对USD PhysicsJoint"""
        try:
            # 首先检查关系(Relationship) - USD Physics关节的标准方式
            rel_names = ['physics:body1', 'body1', 'childBody', 'child']
            
            for rel_name in rel_names:
                if hasattr(joint, 'get_relationship'):
                    try:
                        rel = joint.get_relationship(rel_name)
                        if rel:
                            targets = rel.get_targets()
                            if targets:
                                # 获取第一个目标路径
                                target_path = str(targets[0])
                                # 提取链接名（去掉路径前缀）
                                link_name = target_path.split('/')[-1]
                                logger.debug(f"从关系 {rel_name} 获取子链接: {link_name}")
                                return link_name
                    except Exception as e:
                        logger.debug(f"解析关系 {rel_name} 失败: {e}")
                        continue
            
            # 备选：尝试属性方法（兼容其他格式）
            attr_names = [
                'physics:body1', 'child_link', 'childLink', 'body1',
                'child', 'rel:child', 'target1', 'link1',
                'urdf:child_link', 'urdf:child', 'childBody'
            ]
            
            for attr_name in attr_names:
                if hasattr(joint, 'get_attribute'):
                    attr = joint.get_attribute(attr_name)
                    if attr:
                        try:
                            value = attr.get()
                            if value:
                                # 提取链接名（去掉路径前缀）
                                link_name = str(value).split('/')[-1]
                                if link_name:
                                    logger.debug(f"从属性 {attr_name} 获取子链接: {link_name}")
                                    return link_name
                        except Exception as e:
                            logger.debug(f"解析属性 {attr_name} 失败: {e}")
                            continue
            
            # 最后：从关节名称推断
            joint_name = joint.name.lower()
            if 'joint' in joint_name:
                # 对于 panda_joint1，子链接通常是 panda_link1
                joint_num = ''.join(filter(str.isdigit, joint_name))
                if joint_num:
                    child_link = f"panda_link{joint_num}"
                    logger.debug(f"从关节名推断子链接: {child_link}")
                    return child_link
                    
        except Exception as e:
            logger.debug(f"获取子链接失败: {e}")
        return None
    
    def _find_base_link(self, links: List["AssetPrim"], joints: List["AssetPrim"]) -> Optional["AssetPrim"]:
        """找到机器人的基座链接（根链接）"""
        # 方法1：通过名称识别
        for link in links:
            name_lower = link.name.lower()
            if any(keyword in name_lower for keyword in ['base', 'root', 'world']):
                logger.info(f"Found base link by name: {link.name}")
                return link
        
        # 方法2：找到没有被任何关节作为子链接的链接
        child_links = set()
        for joint in joints:
            child_link = self._get_joint_child_link(joint)
            if child_link:
                child_links.add(child_link)
        
        for link in links:
            if link.name not in child_links:
                logger.info(f"Found base link by exclusion: {link.name}")
                return link
        
        # 方法3：如果都不行，使用第一个链接
        if links:
            logger.warning(f"Using first link as base: {links[0].name}")
            return links[0]
            
        return None
    
    def _compute_urdf_kinematic_chain(self, current_link: "AssetPrim", 
                                    positions: Dict[str, List[float]], 
                                    joints: List["AssetPrim"]):
        """基于URDF关节origin信息递归计算运动学链"""
        current_pos = positions[current_link.name]
        
        # 找到以当前链接为父链接的所有关节
        for joint in joints:
            parent_link = self._get_joint_parent_link(joint)
            child_link = self._get_joint_child_link(joint)
            
            if parent_link == current_link.name and child_link:
                # 避免重复计算同一个链接
                if child_link in positions:
                    logger.debug(f"跳过已计算的链接: {child_link}")
                    continue
                
                # 获取关节的origin信息
                joint_origin = self._get_joint_origin(joint)
                if joint_origin:
                    # 计算子链接的位置
                    child_pos = [
                        current_pos[0] + joint_origin[0],
                        current_pos[1] + joint_origin[1],
                        current_pos[2] + joint_origin[2]
                    ]
                    positions[child_link] = child_pos
                    logger.info(f"Computed position for {child_link}: {child_pos} (from joint {joint.name})")
                else:
                    # 如果没有origin信息，使用默认偏移
                    child_pos = [
                        current_pos[0] + 0.0,
                        current_pos[1] + 0.0,
                        current_pos[2] + 0.2  # 默认向上偏移20cm
                    ]
                    positions[child_link] = child_pos
                    logger.info(f"Using default offset for {child_link}: {child_pos}")
                
                # 递归计算子链接的子链接 - 直接使用链接名查找
                child_link_obj = None
                # 从原始链接列表中查找对应的prim对象
                for link_info in self._link_tree.values():
                    if link_info['prim'].name == child_link:
                        child_link_obj = link_info['prim']
                        break
                
                if child_link_obj:
                    logger.debug(f"递归计算子链接: {child_link}")
                    self._compute_urdf_kinematic_chain(child_link_obj, positions, joints)
                else:
                    logger.warning(f"未找到子链接对象: {child_link}")
    
    def _get_joint_origin(self, joint: "AssetPrim") -> Optional[List[float]]:
        """从关节中提取origin信息（xyz位移）"""
        try:
            # 尝试多种可能的origin属性
            origin_attrs = [
                'origin', 'xyz', 'translation', 'offset',
                'physics:localPos0', 'physics:localPos1'
            ]
            
            for attr_name in origin_attrs:
                if hasattr(joint, 'get_attribute'):
                    attr = joint.get_attribute(attr_name)
                    if attr and attr.get():
                        value = attr.get()
                        if hasattr(value, '__len__') and len(value) >= 3:
                            return [float(value[0]), float(value[1]), float(value[2])]
                        elif isinstance(value, str):
                            # 解析字符串格式，如 "0 0 0.15"
                            parts = value.strip().split()
                            if len(parts) >= 3:
                                try:
                                    return [float(parts[0]), float(parts[1]), float(parts[2])]
                                except ValueError:
                                    pass
            
            # 如果没有找到明确的origin，尝试从子元素中查找
            if hasattr(joint, 'get_children'):
                for child in joint.get_children():
                    if hasattr(child, 'name') and 'origin' in child.name.lower():
                        xyz_attr = child.get_attribute('xyz')
                        if xyz_attr and xyz_attr.get():
                            value = xyz_attr.get()
                            if hasattr(value, '__len__') and len(value) >= 3:
                                return [float(value[0]), float(value[1]), float(value[2])]
                                
        except Exception as e:
            logger.debug(f"Failed to extract origin from joint {joint.name}: {e}")
            
        return None

    def _calculate_real_link_positions(self, links: List["AssetPrim"], joints: List["AssetPrim"]) -> Dict[str, List[float]]:
        """基于URDF关节信息计算链接的真实空间位置
        
        Args:
            links: 链接列表
            joints: 关节列表
            
        Returns:
            链接位置字典 {link_name: [x, y, z]}
        """
        positions = {}
        
        # 首先尝试直接从USD数据提取所有链接的位置
        logger.info("尝试从USD数据提取链接位置...")
        usd_positions = {}
        for link in links:
            extracted_pos = self._extract_real_position(link)
            if extracted_pos:
                usd_positions[link.name] = extracted_pos
                logger.info(f"从USD提取位置 {link.name}: {extracted_pos}")
        
        # 如果成功提取了所有链接的位置，直接使用USD位置
        if len(usd_positions) == len(links):
            logger.info("✅ 成功从USD提取所有链接位置，使用USD数据")
            return usd_positions
        
        # 如果USD位置不完整，尝试使用关节运动学计算
        logger.info(f"USD只提取了 {len(usd_positions)}/{len(links)} 个位置，尝试运动学计算...")
        
        # 第一步：找到基座链接（根链接）
        base_link = self._find_base_link(links, joints)
        if base_link:
            positions[base_link.name] = [0.0, 0.0, 0.0]  # 基座在原点
            
            # 第二步：从基座开始，使用关节origin信息递归计算所有链接位置
            self._compute_urdf_kinematic_chain(base_link, positions, joints)
        
        # 第三步：处理孤立的链接（没有连接到主链的）
        orphan_links = [link for link in links if link.name not in positions]
        if orphan_links:
            logger.warning(f"Found {len(orphan_links)} orphan links: {[l.name for l in orphan_links]}")
            
            # 如果运动学计算失败，回退到USD位置（如果有的话）
            if usd_positions:
                logger.info("运动学计算不完整，使用USD位置作为备用")
                for name, pos in usd_positions.items():
                    positions[name] = pos
                return positions
            
            # 最后备用方案：使用硬编码布局
            logger.warning("使用硬编码布局作为最后备用")
            positions = self._create_hardcoded_urdf_layout(links, joints)
            
        return positions
    
    def _create_hardcoded_urdf_layout(self, links: List["AssetPrim"], joints: List["AssetPrim"]) -> Dict[str, List[float]]:
        """创建基于USD数据的位置布局，优先使用USD位置信息"""
        positions = {}
        
        logger.info("使用硬编码布局:")
        
        # 直接使用USD数据中的位置 - 这是最准确的
        for link in links:
            extracted_pos = self._extract_real_position(link)
            if extracted_pos:
                positions[link.name] = extracted_pos
                logger.info(f"从USD提取位置 {link.name}: {extracted_pos}")
        
        # 如果USD提取成功，直接返回（这是最准确的位置）
        if len(positions) == len(links):
            logger.info("✅ 成功从USD提取所有链接位置")
            return positions
        
        # 如果部分失败，用硬编码补充缺失的
        logger.warning(f"只从USD提取了 {len(positions)}/{len(links)} 个位置，使用备用方案")
        
        # 备用方案：根据链接名称类型使用硬编码位置
        link_map = {link.name: link for link in links}
        
        # 检查是否是Franka机器人
        franka_links = [name for name in link_map.keys() if 'panda_link' in name]
        if franka_links:
            logger.info("检测到Franka机器人，使用Franka硬编码布局")
            hardcoded_positions = {
                'panda_link0': [0.0, 0.0, 0.0],
                'panda_link1': [0.0, 0.0, 0.333],
                'panda_link2': [0.0, 0.0, 0.333],
                'panda_link3': [0.0, 0.0, 0.649],
                'panda_link4': [0.0825, 0.0, 0.649],
                'panda_link5': [0.0825, 0.0, 1.033],
                'panda_link6': [0.0825, 0.0, 1.033],
                'panda_link7': [0.0825, 0.0, 1.12]
            }
            
            # 只为缺失的链接使用硬编码位置
            for name, pos in hardcoded_positions.items():
                if name not in positions and name in link_map:
                    positions[name] = pos
                    logger.info(f"硬编码位置 {name}: {pos}")
                    
            return positions
        
        # 原来的URDF硬编码逻辑
        if 'base_link' in link_map:
            positions['base_link'] = [0.0, 0.0, 0.0]
            
        if 'arm1' in link_map:
            positions['arm1'] = [0.0, 0.0, 0.15]  # base_to_arm1: xyz="0 0 0.15"
            
        if 'arm2' in link_map:
            positions['arm2'] = [0.0, 0.0, 0.35]  # arm1 + arm1_to_arm2: 0.15 + 0.2
            
        if 'end_effector' in link_map:
            positions['end_effector'] = [0.0, 0.0, 0.50]  # arm2 + arm2_to_end: 0.35 + 0.15
            
        logger.info("使用硬编码布局:")
        for name, pos in positions.items():
            logger.info(f"  {name}: {pos}")
            
        return positions
    
    def _extract_real_position(self, link: "AssetPrim") -> Optional[List[float]]:
        """从USD数据中提取链接的真实位置，考虑层次变换"""
        try:
            # 先尝试从properties字典直接获取（这是我们调试中发现有效的方法）
            if hasattr(link, 'properties') and 'xformOp:translate' in link.properties:
                translate_attr = link.properties['xformOp:translate']
                if hasattr(translate_attr, '__iter__') and len(translate_attr) >= 3:
                    local_pos = [float(translate_attr[0]), float(translate_attr[1]), float(translate_attr[2])]
                    logger.info(f"🔍 {link.name} USD原始位置: {local_pos}")
                    
                    # 修正接近零的浮点数（处理科学计数法的小数）
                    corrected_pos = []
                    for coord in local_pos:
                        if abs(coord) < 1e-6:  # 如果值接近0，设为0
                            corrected_pos.append(0.0)
                        else:
                            corrected_pos.append(coord)
                    
                    logger.info(f"✅ {link.name} 修正后位置: {corrected_pos}")
                    return corrected_pos
            
            # 方法1：获取世界变换矩阵
            if hasattr(link, 'get_world_transformation'):
                transform = link.get_world_transformation()
                if transform is not None:
                    return [float(transform[0, 3]), float(transform[1, 3]), float(transform[2, 3])]
            
            # 方法2：获取本地变换矩阵
            if hasattr(link, 'get_local_transformation'):
                transform = link.get_local_transformation()
                if transform is not None:
                    return [float(transform[0, 3]), float(transform[1, 3]), float(transform[2, 3])]
            
            # 方法3：直接从USD属性中获取变换信息
            position_attrs = [
                'xformOp:translate',  # USD标准变换属性
                'translate', 
                'position', 
                'xformOp:transform', 
                'transform'
            ]
            
            for attr_name in position_attrs:
                if hasattr(link, 'get_attribute'):
                    attr = link.get_attribute(attr_name)
                    if attr and attr.get():
                        value = attr.get()
                        # 处理不同的数据格式
                        if hasattr(value, '__len__') and len(value) >= 3:
                            position = [float(value[0]), float(value[1]), float(value[2])]
                            logger.debug(f"从 {attr_name} 提取位置 {link.name}: {position}")
                            return position
                        elif hasattr(value, 'ExtractTranslation'):
                            # USD Gf.Matrix4d类型
                            translation = value.ExtractTranslation()
                            position = [float(translation[0]), float(translation[1]), float(translation[2])]
                            logger.debug(f"从矩阵提取位置 {link.name}: {position}")
                            return position
            
            logger.debug(f"未找到 {link.name} 的位置信息")
            
        except Exception as e:
            logger.debug(f"Failed to extract position for {link.name}: {e}")
            
        return None
    
    def _apply_franka_kinematics(self, link_name: str, local_pos: List[float]) -> List[float]:
        """为Franka机器人应用运动学变换"""
        try:
            x, y, z = local_pos
            logger.info(f"🎯 {link_name} 处理前位置: [{x}, {y}, {z}]")
            
            # 直接使用USD中的位置数据，但修正明显的错误值
            # 如果Y坐标接近0（例如 -6.984919e-8），将其设置为0
            if abs(y) < 1e-6:
                y = 0.0
                logger.info(f"🔧 {link_name} Y坐标修正: {local_pos[1]} -> {y}")
            
            # 返回修正后的位置
            result = [x, y, z]
            logger.info(f"✅ {link_name} 最终位置: {result}")
            return result
            
        except Exception as e:
            logger.warning(f"应用Franka运动学变换失败: {e}")
            return local_pos
    
    def _compute_positions_from_kinematic_chain(self, missing_links: List["AssetPrim"], 
                                              known_positions: Dict[str, List[float]]) -> Dict[str, List[float]]:
        """基于运动学链计算缺失链接的位置"""
        computed = {}
        
        # 找到根链接（没有父链接的链接）
        root_links = []
        for link in missing_links:
            if link.name in self._link_tree:
                parent = self._link_tree[link.name]['parent']
                if parent is None or parent in known_positions:
                    root_links.append(link)
        
        # 从根链接开始递归计算位置
        for root_link in root_links:
            self._recursive_compute_position(root_link, known_positions, computed)
            
        return computed
    
    def _recursive_compute_position(self, link: "AssetPrim", known_positions: Dict[str, List[float]], 
                                  computed: Dict[str, List[float]]):
        """递归计算链接位置"""
        if link.name in computed or link.name in known_positions:
            return
            
        # 获取父链接位置
        parent_name = self._link_tree.get(link.name, {}).get('parent')
        if parent_name:
            if parent_name in known_positions:
                parent_pos = known_positions[parent_name]
            elif parent_name in computed:
                parent_pos = computed[parent_name]
            else:
                return  # 父链接位置未知，无法计算
                
            # 尝试从关节信息计算相对位置
            relative_pos = self._get_joint_relative_position(link, parent_name)
            if relative_pos:
                computed[link.name] = [
                    parent_pos[0] + relative_pos[0],
                    parent_pos[1] + relative_pos[1], 
                    parent_pos[2] + relative_pos[2]
                ]
            else:
                # 使用默认偏移
                computed[link.name] = [
                    parent_pos[0] + 0.1,
                    parent_pos[1],
                    parent_pos[2] + 0.1
                ]
        else:
            # 没有父链接，放在原点
            computed[link.name] = [0.0, 0.0, 0.0]
            
        # 递归计算子链接
        children = self._link_tree.get(link.name, {}).get('children', [])
        for child_name in children:
            child_link = next((l for l in self._link_tree.values() if l['prim'].name == child_name), None)
            if child_link:
                self._recursive_compute_position(child_link['prim'], known_positions, computed)
    
    def _get_joint_relative_position(self, child_link: "AssetPrim", parent_name: str) -> Optional[List[float]]:
        """获取关节的相对位置"""
        try:
            # 查找连接这两个链接的关节
            for joint_info in self._joint_connections.values():
                if (joint_info['parent_link'] == parent_name and 
                    joint_info['child_link'] == child_link.name):
                    
                    joint = joint_info['joint']
                    
                    # 尝试获取关节的位置信息
                    for attr_name in ['physics:localPos0', 'physics:localPos1', 'anchor', 'origin']:
                        if hasattr(joint, 'get_attribute'):
                            attr = joint.get_attribute(attr_name)
                            if attr and attr.get():
                                pos = attr.get()
                                if hasattr(pos, '__len__') and len(pos) >= 3:
                                    return [float(pos[0]), float(pos[1]), float(pos[2])]
                    break
                    
        except Exception:
            pass
            
        return None

    def _generate_improved_layout(self, missing_links: List["AssetPrim"], 
                                known_positions: Dict[str, List[float]],
                                joints: List["AssetPrim"]) -> Dict[str, List[float]]:
        """为缺失位置的链接生成改进的智能布局"""
        positions = {}
        
        if not known_positions:
            # 如果没有任何已知位置，创建基于关节层次的布局
            base_links = []
            other_links = []
            
            # 分类链接
            for link in missing_links:
                name_lower = link.name.lower()
                if any(keyword in name_lower for keyword in ['base', 'root', 'world']):
                    base_links.append(link)
                else:
                    other_links.append(link)
            
            # 基座放在原点
            for i, base_link in enumerate(base_links):
                positions[base_link.name] = [i * 0.3, 0, 0]
            
            # 其他链接按照URDF层次布局
            if base_links:
                base_pos = positions[base_links[0].name]
                for i, link in enumerate(other_links):
                    # 根据URDF中的关节顺序垂直排列
                    positions[link.name] = [
                        base_pos[0],
                        base_pos[1],
                        base_pos[2] + 0.2 * (i + 1)
                    ]
            else:
                # 没有基座，水平排列
                for i, link in enumerate(other_links):
                    positions[link.name] = [i * 0.2, 0, 0.1]
        else:
            # 基于已知位置分布缺失的链接
            existing_positions = list(known_positions.values())
            center = np.mean(existing_positions, axis=0)
            max_distance = max([np.linalg.norm(np.array(pos) - center) for pos in existing_positions])
            
            for i, link in enumerate(missing_links):
                # 在已知位置周围分布
                angle = 2 * np.pi * i / len(missing_links)
                radius = max_distance * 1.2
                positions[link.name] = [
                    center[0] + radius * np.cos(angle),
                    center[1] + radius * np.sin(angle),
                    center[2]
                ]
                
        return positions
    
    def _render_all_links_with_geometry(self, ax, links: List["AssetPrim"], 
                                      link_positions: Dict[str, List[float]]):
        """使用真实几何数据渲染所有链接"""
        colors = ['blue', 'red', 'orange', 'yellow', 'green', 'cyan', 'purple', 'magenta', 'gray', 'brown']
        
        for i, link in enumerate(links):
            link_name = link.name
            position = link_positions.get(link_name, [0, 0, 0])
            color = colors[i % len(colors)]
            
            logger.info(f"Rendering {link_name} at position {position}")
            
            # 尝试获取真实几何信息
            geometry_info = self._extract_geometry_info(link)
            
            if geometry_info:
                logger.info(f"📐 找到真实几何信息: {link_name} -> {geometry_info}")
                self._render_geometry(ax, position, geometry_info, color, link_name)
            else:
                logger.info(f"📦 使用智能默认几何: {link_name}")
                # 使用智能默认几何形状（基于链接名称）
                self._render_smart_default_geometry(ax, position, color, link_name)
    
    def _extract_geometry_info(self, link: "AssetPrim") -> Optional[Dict]:
        """从USD数据中提取几何信息 - 增强版支持visuals"""
        try:
            # 1. 优先：查找 visuals 子节点（真实视觉几何）
            visual_geom = self._extract_visual_geometry(link)
            if visual_geom:
                logger.info(f"🎨 找到visuals几何: {link.name} -> {visual_geom['type']}")
                return visual_geom
            
            # 2. 备选：查找 collisions 子节点（碰撞几何）
            collision_geom = self._extract_collision_geometry(link)
            if collision_geom:
                logger.info(f"🛡️ 使用collision几何: {link.name} -> {collision_geom['type']}")
                return collision_geom
            
            # 3. 最后：查找直接几何子节点
            direct_geom = self._extract_direct_geometry(link)
            if direct_geom:
                logger.info(f"📐 找到直接几何: {link.name} -> {direct_geom['type']}")
                return direct_geom
            
            logger.debug(f"未找到几何信息: {link.name}")
            return None
            
        except Exception as e:
            logger.debug(f"几何提取失败 {link.name}: {e}")
            return None
    
    def _extract_visual_geometry(self, link: "AssetPrim") -> Optional[Dict]:
        """提取USD visuals几何信息"""
        try:
            # 查找 visuals 子节点
            if hasattr(link, 'get_child'):
                visuals_prim = link.get_child("visuals")
                if visuals_prim:
                    logger.debug(f"找到visuals节点: {link.name}/visuals")
                    
                    # 检查是否有外部引用
                    if hasattr(visuals_prim, 'get_references'):
                        references = visuals_prim.get_references()
                        if references:
                            # 有外部网格引用
                            geometry_info = {
                                'type': 'mesh',
                                'source': 'external_reference',
                                'reference_path': str(references[0]),
                                'size': [0.1, 0.1, 0.1],  # 默认尺寸
                                'material': {
                                    'color': [0.7, 0.7, 0.7, 1.0],  # 默认灰色
                                    'roughness': 0.5
                                }
                            }
                            logger.info(f"外部引用: {references[0]}")
                            return geometry_info
                    
                    # 检查内部几何定义
                    return self._extract_geometry_from_prim(visuals_prim, 'visual')
            
            return None
            
        except Exception as e:
            logger.debug(f"visuals提取失败: {e}")
            return None
    
    def _extract_collision_geometry(self, link: "AssetPrim") -> Optional[Dict]:
        """提取collision几何信息"""
        try:
            if hasattr(link, 'get_child'):
                collisions_prim = link.get_child("collisions")
                if collisions_prim:
                    logger.debug(f"找到collisions节点: {link.name}/collisions")
                    return self._extract_geometry_from_prim(collisions_prim, 'collision')
            
            return None
            
        except Exception as e:
            logger.debug(f"collision提取失败: {e}")
            return None
    
    def _extract_direct_geometry(self, link: "AssetPrim") -> Optional[Dict]:
        """提取直接几何子节点"""
        try:
            found_real_geometry = False
            geometry_info = {
                'type': 'box',  # 默认类型
                'size': [0.1, 0.1, 0.1],  # 默认尺寸
                'source': 'direct_geometry',
                'material': {
                    'color': [0.7, 0.7, 0.7, 1.0],
                    'roughness': 0.5
                }
            }
            
            # 查找几何属性
            if hasattr(link, 'get_children'):
                children = link.get_children()
                for child in children:
                    if hasattr(child, 'get_type_name'):
                        type_name = child.get_type_name()
                        
                        if type_name == 'Cube':
                            geometry_info['type'] = 'box'
                            size_attr = child.get_attribute('size')
                            if size_attr and size_attr.get():
                                size = size_attr.get()
                                geometry_info['size'] = [float(size), float(size), float(size)]
                                found_real_geometry = True
                                
                        elif type_name == 'Cylinder':
                            geometry_info['type'] = 'cylinder'
                            radius_attr = child.get_attribute('radius')
                            height_attr = child.get_attribute('height')
                            if radius_attr and radius_attr.get():
                                geometry_info['radius'] = float(radius_attr.get())
                                found_real_geometry = True
                            if height_attr and height_attr.get():
                                geometry_info['height'] = float(height_attr.get())
                                found_real_geometry = True
                                
                        elif type_name == 'Sphere':
                            geometry_info['type'] = 'sphere'
                            radius_attr = child.get_attribute('radius')
                            if radius_attr and radius_attr.get():
                                geometry_info['radius'] = float(radius_attr.get())
                                found_real_geometry = True
            
            return geometry_info if found_real_geometry else None
            
        except Exception as e:
            logger.debug(f"直接几何提取失败: {e}")
            return None
    
    def _extract_geometry_from_prim(self, prim: "AssetPrim", source_type: str) -> Optional[Dict]:
        """从指定的prim中提取几何信息"""
        try:
            geometry_info = {
                'type': 'box',
                'size': [0.1, 0.1, 0.1],
                'source': source_type,
                'material': {
                    'color': [0.7, 0.7, 0.7, 1.0] if source_type == 'visual' else [0.5, 0.5, 0.5, 0.5],
                    'roughness': 0.5
                }
            }
            
            # 查找几何子节点
            if hasattr(prim, 'get_children'):
                children = prim.get_children()
                for child in children:
                    if hasattr(child, 'get_type_name'):
                        type_name = child.get_type_name()
                        
                        # 处理基础几何类型
                        if type_name in ['Cube', 'Box']:
                            geometry_info['type'] = 'box'
                            self._extract_box_parameters(child, geometry_info)
                            return geometry_info
                            
                        elif type_name == 'Cylinder':
                            geometry_info['type'] = 'cylinder'
                            self._extract_cylinder_parameters(child, geometry_info)
                            return geometry_info
                            
                        elif type_name == 'Sphere':
                            geometry_info['type'] = 'sphere'
                            self._extract_sphere_parameters(child, geometry_info)
                            return geometry_info
                        
                        elif type_name == 'Mesh':
                            geometry_info['type'] = 'mesh'
                            self._extract_mesh_parameters(child, geometry_info)
                            return geometry_info
            
            return None
            
        except Exception as e:
            logger.debug(f"prim几何提取失败: {e}")
            return None
    
    def _extract_box_parameters(self, geom_prim: "AssetPrim", geometry_info: Dict):
        """提取立方体参数"""
        try:
            size_attr = geom_prim.get_attribute('size')
            if size_attr and size_attr.get():
                size = size_attr.get()
                if hasattr(size, '__iter__') and len(size) >= 3:
                    geometry_info['size'] = [float(size[0]), float(size[1]), float(size[2])]
                else:
                    geometry_info['size'] = [float(size), float(size), float(size)]
        except Exception as e:
            logger.debug(f"立方体参数提取失败: {e}")
    
    def _extract_cylinder_parameters(self, geom_prim: "AssetPrim", geometry_info: Dict):
        """提取圆柱体参数"""
        try:
            radius_attr = geom_prim.get_attribute('radius')
            height_attr = geom_prim.get_attribute('height')
            
            if radius_attr and radius_attr.get():
                geometry_info['radius'] = float(radius_attr.get())
            else:
                geometry_info['radius'] = 0.05  # 默认半径
                
            if height_attr and height_attr.get():
                geometry_info['height'] = float(height_attr.get())
            else:
                geometry_info['height'] = 0.1  # 默认高度
        except Exception as e:
            logger.debug(f"圆柱体参数提取失败: {e}")
    
    def _extract_sphere_parameters(self, geom_prim: "AssetPrim", geometry_info: Dict):
        """提取球体参数"""
        try:
            radius_attr = geom_prim.get_attribute('radius')
            if radius_attr and radius_attr.get():
                geometry_info['radius'] = float(radius_attr.get())
            else:
                geometry_info['radius'] = 0.05  # 默认半径
        except Exception as e:
            logger.debug(f"球体参数提取失败: {e}")
    
    def _extract_mesh_parameters(self, geom_prim: "AssetPrim", geometry_info: Dict):
        """提取网格参数"""
        try:
            # 查找网格文件路径
            file_attr = geom_prim.get_attribute('file')
            if file_attr and file_attr.get():
                geometry_info['mesh_file'] = str(file_attr.get())
            
            # 查找点和面数据
            points_attr = geom_prim.get_attribute('points')
            if points_attr and points_attr.get():
                geometry_info['points'] = points_attr.get()
            
            faces_attr = geom_prim.get_attribute('faceVertexIndices')
            if faces_attr and faces_attr.get():
                geometry_info['faces'] = faces_attr.get()
                
        except Exception as e:
            logger.debug(f"网格参数提取失败: {e}")
    
    def _render_geometry(self, ax, position: List[float], geometry_info: Dict, color: str, label: str):
        """根据几何信息渲染链接"""
        geom_type = geometry_info.get('type', 'box')
        
        if geom_type == 'box':
            size = geometry_info.get('size', [0.1, 0.1, 0.1])
            self.plotter.plot_box(ax, position, size, color, label)
            
        elif geom_type == 'cylinder':
            radius = geometry_info.get('radius', 0.05)
            height = geometry_info.get('height', 0.1)
            self.plotter.plot_cylinder(ax, position, radius, height, color, label)
            
        elif geom_type == 'sphere':
            radius = geometry_info.get('radius', 0.05)
            # 用立方体近似球体（如果没有sphere方法）
            size = [radius * 2, radius * 2, radius * 2]
            self.plotter.plot_box(ax, position, size, color, label)
            
        else:
            # 默认渲染
            self._render_default_geometry(ax, position, color, label)
    
    def _render_default_geometry(self, ax, position: List[float], color: str, label: str):
        """渲染默认几何形状"""
        # 使用适中的默认尺寸
        self.plotter.plot_box(ax, position, [0.08, 0.08, 0.08], color, label)
    
    def _render_smart_default_geometry(self, ax, position: List[float], color: str, label: str):
        """基于链接名称智能选择几何形状"""
        link_name_lower = label.lower()
        
        logger.info(f"🔧 使用智能几何形状渲染: {label}")
        
        if 'base' in link_name_lower:
            # 基座：较大的立方体
            logger.info(f"  -> 基座形状: [0.15, 0.15, 0.05]")
            self.plotter.plot_box(ax, position, [0.15, 0.15, 0.05], color, label)
            
        elif 'arm' in link_name_lower or 'link' in link_name_lower:
            # 手臂：长方形，模拟臂段
            if 'arm1' in link_name_lower:
                # 第一臂段：较粗，垂直方向
                logger.info(f"  -> 第一臂段形状: [0.06, 0.06, 0.12]")
                self.plotter.plot_box(ax, position, [0.06, 0.06, 0.12], color, label)
            elif 'arm2' in link_name_lower:
                # 第二臂段：稍细，垂直方向
                logger.info(f"  -> 第二臂段形状: [0.05, 0.05, 0.10]")
                self.plotter.plot_box(ax, position, [0.05, 0.05, 0.10], color, label)
            else:
                # 其他臂段
                logger.info(f"  -> 其他臂段形状: [0.05, 0.05, 0.08]")
                self.plotter.plot_box(ax, position, [0.05, 0.05, 0.08], color, label)
                
        elif 'end' in link_name_lower or 'effector' in link_name_lower or 'gripper' in link_name_lower:
            # 末端执行器：较小的立方体
            logger.info(f"  -> 末端执行器形状: [0.04, 0.04, 0.06]")
            self.plotter.plot_box(ax, position, [0.04, 0.04, 0.06], color, label)
            
        elif 'wheel' in link_name_lower:
            # 轮子：圆柱体（如果支持的话）
            if hasattr(self.plotter, 'plot_cylinder'):
                logger.info(f"  -> 轮子形状: 圆柱体 radius=0.05, height=0.02")
                self.plotter.plot_cylinder(ax, position, 0.05, 0.02, color, label)
            else:
                logger.info(f"  -> 轮子形状（立方体替代）: [0.1, 0.02, 0.1]")
                self.plotter.plot_box(ax, position, [0.1, 0.02, 0.1], color, label)
                
        elif 'leg' in link_name_lower or 'foot' in link_name_lower:
            # 腿部：细长的立方体
            logger.info(f"  -> 腿部形状: [0.03, 0.03, 0.15]")
            self.plotter.plot_box(ax, position, [0.03, 0.03, 0.15], color, label)
            
        else:
            # 默认：中等尺寸的立方体
            logger.info(f"  -> 默认形状: [0.08, 0.08, 0.08]")
            self.plotter.plot_box(ax, position, [0.08, 0.08, 0.08], color, label)
    
    def _render_real_joint_connections(self, ax, link_positions: Dict[str, List[float]], 
                                     joints: List["AssetPrim"]):
        """基于真实关节连接关系渲染连接线"""
        connection_count = 0
        
        # 首先尝试使用解析的关节连接
        for joint_info in self._joint_connections.values():
            parent_link = joint_info['parent_link']
            child_link = joint_info['child_link']
            
            if parent_link in link_positions and child_link in link_positions:
                pos1 = link_positions[parent_link]
                pos2 = link_positions[child_link]
                label = 'Joint Connections' if connection_count == 0 else ""
                self.plotter.draw_connection_line(ax, pos1, pos2, color='black', style='--', label=label)
                connection_count += 1
        
        # 如果没有找到连接关系，检查是否是Franka机器人并使用硬编码连接
        if connection_count == 0:
            franka_connections = self._render_franka_connections(ax, link_positions)
            if franka_connections == 0:
                # 如果不是Franka机器人，使用树状结构连接
                self._render_tree_connections(ax, link_positions)
    
    def _render_franka_connections(self, ax, link_positions: Dict[str, List[float]]) -> int:
        """为Franka机器人渲染硬编码的运动学链连接"""
        # Franka机器人的标准运动学链：panda_link0 -> panda_link1 -> ... -> panda_link7
        franka_chain = [
            ('panda_link0', 'panda_link1'),
            ('panda_link1', 'panda_link2'),
            ('panda_link2', 'panda_link3'),
            ('panda_link3', 'panda_link4'),
            ('panda_link4', 'panda_link5'),
            ('panda_link5', 'panda_link6'),
            ('panda_link6', 'panda_link7')
        ]
        
        connection_count = 0
        for i, (parent_link, child_link) in enumerate(franka_chain):
            if parent_link in link_positions and child_link in link_positions:
                pos1 = link_positions[parent_link]
                pos2 = link_positions[child_link]
                label = 'Franka Kinematic Chain' if connection_count == 0 else ""
                # 使用红色虚线表示Franka运动学链
                self.plotter.draw_connection_line(ax, pos1, pos2, color='red', style='--', 
                                                linewidth=2, label=label)
                connection_count += 1
                logger.debug(f"绘制Franka连接: {parent_link} -> {child_link}")
        
        if connection_count > 0:
            logger.info(f"成功绘制 {connection_count} 条Franka运动学链连接")
        
        return connection_count
    
    def _render_tree_connections(self, ax, link_positions: Dict[str, List[float]]):
        """基于树状结构渲染连接"""
        connection_count = 0
        
        for link_name, link_info in self._link_tree.items():
            parent_name = link_info.get('parent')
            if parent_name and parent_name in link_positions and link_name in link_positions:
                pos1 = link_positions[parent_name]
                pos2 = link_positions[link_name]
                label = 'Structural Connections' if connection_count == 0 else ""
                self.plotter.draw_connection_line(ax, pos1, pos2, color='gray', style='-', label=label)
                connection_count += 1
    
    def _calculate_scene_bounds(self, link_positions: Dict[str, List[float]]) -> float:
        """计算场景边界以进行自动缩放"""
        if not link_positions:
            return 1.0
            
        positions = np.array(list(link_positions.values()))
        min_coords = np.min(positions, axis=0)
        max_coords = np.max(positions, axis=0)
        
        # 计算最大范围
        ranges = max_coords - min_coords
        max_range = np.max(ranges)
        
        # 添加一些边距
        return max(max_range * 1.2, 0.5)
    
    def _render_coordinate_frames(self, ax, link_positions: Dict[str, List[float]]):
        """渲染坐标系"""
        for name, pos in link_positions.items():
            self.plotter.draw_coordinate_frame(ax, pos, size=0.1)


__all__ = ['RobotVisualizer']
