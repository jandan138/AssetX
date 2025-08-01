#!/usr/bin/env python3
"""
机器人可视化器 - 专门负责机器人整体渲染逻辑
"""

from typing import Dict, List, Optional
import logging

from .plotting import MatplotlibPlotter

logger = logging.getLogger(__name__)


class RobotVisualizer:
    """机器人可视化器
    
    专门负责机器人的整体可视化：
    - 链接位置计算
    - 机器人结构渲染
    - 关节连接显示
    - 坐标系显示
    """

    def __init__(self, asset: 'Asset'):
        """初始化机器人可视化器
        
        Args:
            asset: 所属的Asset对象
        """
        self.asset = asset
        self.plotter = MatplotlibPlotter()
        
    def render_complete_robot(self, show_joints: bool = True, show_frames: bool = True) -> bool:
        """渲染完整机器人
        
        Args:
            show_joints: 是否显示关节连接
            show_frames: 是否显示坐标系
            
        Returns:
            是否渲染成功
        """
        try:
            # 创建3D图形
            fig, ax = self.plotter.create_3d_figure("AssetX Complete Robot", (12, 10))
            if fig is None or ax is None:
                logger.error("无法创建3D图形")
                return False
            
            # 获取机器人结构
            links = self.asset.query.get_links()
            joints = self.asset.query.get_joints()
            
            if not links:
                logger.error("No links found in robot")
                return False
            
            logger.info(f"Rendering complete robot with {len(links)} links and {len(joints)} joints")
            
            # 计算链接的空间布局
            link_positions = self._calculate_link_positions(links, joints)
            
            # 渲染每个链接
            self._render_all_links(ax, links, link_positions)
            
            # 显示关节连接线
            if show_joints and joints:
                self._render_joint_connections(ax, link_positions, joints)
            
            # 显示坐标系
            if show_frames:
                self._render_coordinate_frames(ax, link_positions)
            
            # 设置场景和完成绘图
            self.plotter.setup_3d_scene(ax, max_range=0.6)
            self.plotter.finalize_plot(ax, show_legend=True)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to render complete robot: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _calculate_link_positions(self, links: List["AssetPrim"], joints: List["AssetPrim"]) -> Dict[str, List[float]]:
        """计算各链接的空间位置（简化运动学）
        
        Args:
            links: 链接列表
            joints: 关节列表
            
        Returns:
            链接位置字典 {link_name: [x, y, z]}
        """
        positions = {}
        
        # 基座固定在原点
        for link in links:
            if 'base' in link.name.lower():
                positions[link.name] = [0, 0, 0.05]  # 稍微抬高基座
                break
        
        # 简化的链式布局
        for link in links:
            name = link.name
            if 'arm1' in name.lower():
                positions[name] = [0, 0, 0.35]  # 基座上方
            elif 'arm2' in name.lower():
                positions[name] = [0, 0, 0.75]  # arm1上方
            elif 'end' in name.lower() or 'effector' in name.lower():
                positions[name] = [0, 0, 1.05]  # arm2上方
        
        return positions
    
    def _render_all_links(self, ax, links: List["AssetPrim"], link_positions: Dict[str, List[float]]):
        """渲染所有链接"""
        for link in links:
            link_name = link.name
            position = link_positions.get(link_name, [0, 0, 0])
            
            logger.info(f"Rendering {link_name} at position {position}")
            
            # 根据链接名称确定几何类型和参数
            if 'base' in link_name.lower():
                self.plotter.plot_box(ax, position, [0.5, 0.3, 0.1], 'blue', link_name)
            elif 'arm1' in link_name.lower():
                self.plotter.plot_cylinder(ax, [position[0], position[1], position[2] + 0.2], 
                                         0.05, 0.4, 'red', link_name)
            elif 'arm2' in link_name.lower():
                self.plotter.plot_cylinder(ax, [position[0], position[1], position[2] + 0.15], 
                                         0.04, 0.3, 'green', link_name)
            elif 'end' in link_name.lower() or 'effector' in link_name.lower():
                self.plotter.plot_sphere(ax, [position[0], position[1], position[2] + 0.15], 
                                       0.06, 'yellow', link_name)
            else:
                # 默认小立方体
                self.plotter.plot_box(ax, position, [0.1, 0.1, 0.1], 'gray', link_name)
    
    def _render_joint_connections(self, ax, link_positions: Dict[str, List[float]], joints: List["AssetPrim"]):
        """渲染关节连接线"""
        # 简化的连接：按顺序连接链接
        link_names = list(link_positions.keys())
        for i in range(len(link_names) - 1):
            pos1 = link_positions[link_names[i]]
            pos2 = link_positions[link_names[i + 1]]
            label = 'Joints' if i == 0 else ""
            self.plotter.draw_connection_line(ax, pos1, pos2, color='black', style='--', label=label)
    
    def _render_coordinate_frames(self, ax, link_positions: Dict[str, List[float]]):
        """渲染坐标系"""
        for name, pos in link_positions.items():
            self.plotter.draw_coordinate_frame(ax, pos, size=0.1)


__all__ = ['RobotVisualizer']
