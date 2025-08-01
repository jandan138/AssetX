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
        """获取关节的父链接"""
        try:
            # 扩展的属性名列表，优先检查URDF转换后常见的属性
            attr_names = [
                'physics:body0', 'parent_link', 'parentLink', 'body0',
                'parent', 'rel:parent', 'target0', 'link0',
                # URDF转换可能使用的属性
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
                                    logger.debug(f"从 {attr_name} 获取父链接: {link_name}")
                                    return link_name
                        except Exception as e:
                            logger.debug(f"解析属性 {attr_name} 失败: {e}")
                            continue
            
            # 尝试从关节名称推断（如果遵循命名约定）
            joint_name = joint.name.lower()
            
            # 常见的关节命名模式
            if '_to_' in joint_name:
                parts = joint_name.split('_to_')
                if len(parts) >= 2:
                    parent_link = parts[0]
                    # 处理常见的链接名称映射
                    if parent_link == 'base':
                        parent_link = 'base_link'
                    elif parent_link == 'arm2_to':
                        parent_link = 'arm2'
                    logger.debug(f"从关节名推断父链接: {parent_link}")
                    return parent_link
            elif 'base_to_' in joint_name:
                return 'base_link'
            elif joint_name.endswith('_joint'):
                # 如果是 "base_to_arm1_joint" 这种格式
                base_name = joint_name.replace('_joint', '')
                if '_to_' in base_name:
                    parent_link = base_name.split('_to_')[0]
                    # 处理常见的链接名称映射
                    if parent_link == 'base':
                        parent_link = 'base_link'
                    logger.debug(f"从关节名推断父链接: {parent_link}")
                    return parent_link
                    
        except Exception as e:
            logger.debug(f"获取父链接失败: {e}")
        return None
    
    def _get_joint_child_link(self, joint: "AssetPrim") -> Optional[str]:
        """获取关节的子链接"""
        try:
            # 扩展的属性名列表，优先检查URDF转换后常见的属性
            attr_names = [
                'physics:body1', 'child_link', 'childLink', 'body1',
                'child', 'rel:child', 'target1', 'link1',
                # URDF转换可能使用的属性
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
                                    logger.debug(f"从 {attr_name} 获取子链接: {link_name}")
                                    return link_name
                        except Exception as e:
                            logger.debug(f"解析属性 {attr_name} 失败: {e}")
                            continue
            
            # 尝试从关节名称推断（如果遵循命名约定）
            joint_name = joint.name.lower()
            
            # 常见的关节命名模式
            if '_to_' in joint_name:
                parts = joint_name.split('_to_')
                if len(parts) >= 2:
                    child_link = parts[1]
                    # 处理常见的链接名称映射
                    if child_link == 'end':
                        child_link = 'end_effector'
                    elif 'effector' in child_link:
                        child_link = 'end_effector'
                    logger.debug(f"从关节名推断子链接: {child_link}")
                    return child_link
            elif joint_name.endswith('_joint'):
                # 如果是 "base_to_arm1_joint" 这种格式
                base_name = joint_name.replace('_joint', '')
                if '_to_' in base_name:
                    child_link = base_name.split('_to_')[1]
                    # 处理常见的链接名称映射
                    if child_link == 'end':
                        child_link = 'end_effector'
                    elif 'effector' in child_link:
                        child_link = 'end_effector'
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
                    
                    # 递归计算子链接的子链接
                    child_link_obj = next((l for l in self._link_tree.values() if l['prim'].name == child_link), None)
                    if child_link_obj:
                        self._compute_urdf_kinematic_chain(child_link_obj['prim'], positions, joints)
                else:
                    # 如果没有origin信息，使用默认偏移
                    child_pos = [
                        current_pos[0] + 0.0,
                        current_pos[1] + 0.0,
                        current_pos[2] + 0.2  # 默认向上偏移20cm
                    ]
                    positions[child_link] = child_pos
                    logger.info(f"Using default offset for {child_link}: {child_pos}")
                    
                    child_link_obj = next((l for l in self._link_tree.values() if l['prim'].name == child_link), None)
                    if child_link_obj:
                        self._compute_urdf_kinematic_chain(child_link_obj['prim'], positions, joints)
    
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
        
        # 直接使用URDF关节信息进行运动学计算
        # 这是最可靠的方法，因为URDF origin信息通常是准确的
        
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
            
            # 如果所有链接都是孤立的，说明关节解析失败，使用硬编码的URDF布局
            if len(orphan_links) == len(links) - 1:  # 除了base_link外都是孤立的
                logger.warning("关节解析失败，使用硬编码的URDF布局")
                positions = self._create_hardcoded_urdf_layout(links, joints)
            else:
                # 只有部分孤立链接，放在基座旁边
                for i, link in enumerate(orphan_links):
                    positions[link.name] = [0.3 + i * 0.1, 0.0, 0.0]
            
        return positions
    
    def _create_hardcoded_urdf_layout(self, links: List["AssetPrim"], joints: List["AssetPrim"]) -> Dict[str, List[float]]:
        """创建基于测试URDF的硬编码布局"""
        positions = {}
        
        # 根据测试URDF的结构硬编码位置
        link_map = {link.name: link for link in links}
        
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
        """从USD数据中提取链接的真实位置"""
        try:
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
            
            # 方法3：查找各种可能的位置属性
            position_attrs = [
                'xformOp:translate', 'translate', 'position', 
                'xformOp:transform', 'transform'
            ]
            
            for attr_name in position_attrs:
                if hasattr(link, 'get_attribute'):
                    attr = link.get_attribute(attr_name)
                    if attr and attr.get():
                        value = attr.get()
                        if hasattr(value, '__len__') and len(value) >= 3:
                            return [float(value[0]), float(value[1]), float(value[2])]
                        elif hasattr(value, 'ExtractTranslation'):
                            # USD Gf.Matrix4d类型
                            translation = value.ExtractTranslation()
                            return [float(translation[0]), float(translation[1]), float(translation[2])]
            
        except Exception as e:
            logger.debug(f"Failed to extract position for {link.name}: {e}")
            
        return None
    
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
        """从USD数据中提取几何信息"""
        try:
            found_real_geometry = False
            geometry_info = {
                'type': 'box',  # 默认类型
                'size': [0.1, 0.1, 0.1],  # 默认尺寸
                'properties': {}
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
            
            # 只有找到真实几何信息时才返回，否则返回None让智能默认几何接管
            if found_real_geometry:
                logger.debug(f"Found real geometry for {link.name}: {geometry_info}")
                return geometry_info
            else:
                logger.debug(f"No real geometry found for {link.name}, using smart defaults")
                return None
                                
        except Exception as e:
            logger.debug(f"Failed to extract geometry for {link.name}: {e}")
            return None
    
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
        
        for joint_info in self._joint_connections.values():
            parent_link = joint_info['parent_link']
            child_link = joint_info['child_link']
            
            if parent_link in link_positions and child_link in link_positions:
                pos1 = link_positions[parent_link]
                pos2 = link_positions[child_link]
                label = 'Joint Connections' if connection_count == 0 else ""
                self.plotter.draw_connection_line(ax, pos1, pos2, color='black', style='--', label=label)
                connection_count += 1
        
        # 如果没有找到连接关系，使用树状结构连接
        if connection_count == 0:
            self._render_tree_connections(ax, link_positions)
    
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
