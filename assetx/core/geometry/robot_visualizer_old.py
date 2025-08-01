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
        
        # 解析关节连接关系
        for joint in joints:
            try:
                # 尝试获取关节的父子链接
                parent_link = self._get_joint_parent_link(joint)
                child_link = self._get_joint_child_link(joint)
                
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
                        
            except Exception as e:
                logger.warning(f"Failed to parse joint {joint.name}: {e}")
                continue
    
    def _get_joint_parent_link(self, joint: "AssetPrim") -> Optional[str]:
        """获取关节的父链接"""
        try:
            # 尝试多种可能的属性名
            for attr_name in ['physics:body0', 'parent_link', 'parentLink', 'body0']:
                if hasattr(joint, 'get_attribute'):
                    attr = joint.get_attribute(attr_name)
                    if attr and attr.get():
                        return str(attr.get()).split('/')[-1]  # 提取链接名
            
            # 如果没有找到，尝试从USD层次结构推断
            if hasattr(joint, 'get_parent'):
                parent = joint.get_parent()
                if parent:
                    return parent.name
                    
        except Exception:
            pass
        return None
    
    def _get_joint_child_link(self, joint: "AssetPrim") -> Optional[str]:
        """获取关节的子链接"""
        try:
            # 尝试多种可能的属性名
            for attr_name in ['physics:body1', 'child_link', 'childLink', 'body1']:
                if hasattr(joint, 'get_attribute'):
                    attr = joint.get_attribute(attr_name)
                    if attr and attr.get():
                        return str(attr.get()).split('/')[-1]  # 提取链接名
            
            # 如果没有找到，尝试从USD层次结构推断
            if hasattr(joint, 'get_children'):
                children = joint.get_children()
                if children:
                    return children[0].name
                    
        except Exception:
            pass
        return None
    
    def _calculate_real_link_positions(self, links: List["AssetPrim"], joints: List["AssetPrim"]) -> Dict[str, List[float]]:
        """基于USD数据计算链接的真实空间位置
        
        Args:
            links: 链接列表
            joints: 关节列表
            
        Returns:
            链接位置字典 {link_name: [x, y, z]}
        """
        positions = {}
        
        # 第一步：尝试从USD变换数据获取绝对位置
        for link in links:
            real_position = self._extract_real_position(link)
            if real_position is not None:
                positions[link.name] = real_position
        
        # 第二步：如果某些链接没有明确位置，使用运动学链推算
        missing_links = [link for link in links if link.name not in positions]
        if missing_links:
            computed_positions = self._compute_positions_from_kinematic_chain(missing_links, positions)
            positions.update(computed_positions)
        
        # 第三步：如果仍有缺失，使用智能布局算法
        still_missing = [link for link in links if link.name not in positions]
        if still_missing:
            fallback_positions = self._generate_intelligent_layout(still_missing, positions)
            positions.update(fallback_positions)
            
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
    
    def _generate_intelligent_layout(self, missing_links: List["AssetPrim"], 
                                   known_positions: Dict[str, List[float]]) -> Dict[str, List[float]]:
        """为缺失位置的链接生成智能布局"""
        positions = {}
        
        if not known_positions:
            # 如果没有任何已知位置，创建默认布局
            for i, link in enumerate(missing_links):
                positions[link.name] = [i * 0.2, 0, 0]
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
                self._render_geometry(ax, position, geometry_info, color, link_name)
            else:
                # 使用默认几何形状
                self._render_default_geometry(ax, position, color, link_name)
    
    def _extract_geometry_info(self, link: "AssetPrim") -> Optional[Dict]:
        """从USD数据中提取几何信息"""
        try:
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
                                
                        elif type_name == 'Cylinder':
                            geometry_info['type'] = 'cylinder'
                            radius_attr = child.get_attribute('radius')
                            height_attr = child.get_attribute('height')
                            if radius_attr and radius_attr.get():
                                geometry_info['radius'] = float(radius_attr.get())
                            if height_attr and height_attr.get():
                                geometry_info['height'] = float(height_attr.get())
                                
                        elif type_name == 'Sphere':
                            geometry_info['type'] = 'sphere'
                            radius_attr = child.get_attribute('radius')
                            if radius_attr and radius_attr.get():
                                geometry_info['radius'] = float(radius_attr.get())
                                
            return geometry_info
            
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
