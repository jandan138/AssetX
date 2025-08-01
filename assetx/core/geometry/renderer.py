#!/usr/bin/env python3
"""
几何体渲染器 - 负责可视化相关功能 (重构版)

作为渲染功能的统一入口，委托具体功能给专门的子模块：
- RenderBackend: 后端管理
- MatplotlibPlotter: 基础绘图
- RobotVisualizer: 机器人可视化
"""

from typing import Dict, List, Optional, Any
import logging

from .backend import RenderBackend
from .plotting import MatplotlibPlotter  
from .robot_visualizer import RobotVisualizer

logger = logging.getLogger(__name__)


class GeometryRenderer:
    """几何体渲染器
    
    作为渲染功能的统一入口，委托具体功能给专门的子模块：
    - 后端管理：RenderBackend
    - 基础绘图：MatplotlibPlotter
    - 机器人可视化：RobotVisualizer
    """

    def __init__(self, asset: 'Asset'):
        """初始化渲染器
        
        Args:
            asset: 所属的Asset对象
        """
        self.asset = asset
        
        # 初始化子模块
        self.backend = RenderBackend()
        self.plotter = MatplotlibPlotter()
        self.robot_visualizer = RobotVisualizer(asset)
        
    def has_render_support(self) -> bool:
        """检查是否有渲染支持"""
        return self.backend.has_any_backend
        
    def render_mesh(self, mesh_prim: 'AssetMesh', 
                   show_axes: bool = True, 
                   show_wireframe: bool = False) -> bool:
        """渲染单个网格
        
        Args:
            mesh_prim: 要渲染的AssetMesh
            show_axes: 是否显示坐标轴
            show_wireframe: 是否显示线框
            
        Returns:
            是否渲染成功
        """
        # 优先使用Previewer
        if self.backend.has_previewer:
            mesh_file = mesh_prim.get_mesh_file()
            if mesh_file and mesh_file.exists():
                if self.backend.render_with_previewer(mesh_file, show_axes, show_wireframe):
                    return True
        
        # 备选：使用matplotlib
        if self.backend.has_matplotlib:
            return self._render_mesh_matplotlib(mesh_prim, show_axes)
        
        logger.error("无可用的渲染后端")
        return False
        
    def _render_mesh_matplotlib(self, mesh_prim: 'AssetMesh', show_axes: bool) -> bool:
        """使用matplotlib渲染网格"""
        try:
            # 创建3D图形
            fig, ax = self.plotter.create_3d_figure(f"AssetX Mesh: {mesh_prim.name}")
            if fig is None or ax is None:
                return False
            
            # 获取几何类型和属性
            geom_type = mesh_prim.get_geometry_type()
            if geom_type:
                if geom_type.value == "box":
                    size_attr = mesh_prim.get_attribute("geom:size")
                    size = size_attr.get() if size_attr and size_attr.get() else [1, 1, 1]
                    self.plotter.plot_box(ax, [0, 0, 0], size)
                    
                elif geom_type.value == "sphere":
                    size_attr = mesh_prim.get_attribute("geom:size")
                    radius = size_attr.get()[0] if size_attr and size_attr.get() else 1.0
                    self.plotter.plot_sphere(ax, [0, 0, 0], radius)
                    
                elif geom_type.value == "cylinder":
                    size_attr = mesh_prim.get_attribute("geom:size")
                    if size_attr and size_attr.get():
                        size = size_attr.get()
                        radius, height = size[0], size[2]
                    else:
                        radius, height = 1.0, 2.0
                    self.plotter.plot_cylinder(ax, [0, 0, 0], radius, height)
            
            # 设置场景和完成绘图
            self.plotter.setup_3d_scene(ax, show_axes=show_axes)
            self.plotter.finalize_plot(ax, show_legend=False)
            
            return True
            
        except Exception as e:
            logger.error(f"matplotlib渲染失败: {e}")
            return False
            
    def render_link(self, link_name: str) -> bool:
        """渲染机器人链接"""
        # 查找链接Prim
        link_prim = self.asset.query.get_link_by_name(link_name)
        if not link_prim:
            logger.error(f"Link '{link_name}' not found")
            return False
            
        # 首先查找已存在的网格
        mesh_prims = self.asset.query.get_children_by_type(link_prim, "Mesh")
        
        if mesh_prims:
            # 渲染已存在的网格
            success_count = 0
            for mesh_prim in mesh_prims:
                if hasattr(mesh_prim, 'get_geometry_type'):  # 检查是否为AssetMesh
                    if self.render_mesh(mesh_prim):
                        success_count += 1
            return success_count > 0
        
        # 如果没有现成的网格，创建临时演示几何体
        try:
            logger.info(f"Creating temporary geometry for link '{link_name}' demonstration")
            
            # 通过几何处理器创建网格
            from .manager import GeometryManager
            manager = GeometryManager(self.asset)
            mesh_prim = manager.create_mesh_prim(f'/temp/{link_name}_demo', f'{link_name}_demo')
            
            if 'base' in link_name.lower():
                mesh_prim.set_box_size([0.5, 0.3, 0.1])
            elif 'arm' in link_name.lower():
                if '1' in link_name:
                    mesh_prim.set_cylinder_size(0.05, 0.4)
                else:
                    mesh_prim.set_cylinder_size(0.04, 0.3)
            elif 'end' in link_name.lower() or 'effector' in link_name.lower():
                mesh_prim.set_sphere_size(0.06)
            else:
                mesh_prim.set_box_size([0.1, 0.1, 0.1])
                
            return self.render_mesh(mesh_prim)
            
        except Exception as e:
            logger.error(f"Failed to render link '{link_name}': {e}")
            return False
            
    def render_complete_robot(self, show_joints: bool = True, show_frames: bool = True) -> bool:
        """渲染完整机器人（委托给RobotVisualizer）"""
        return self.robot_visualizer.render_complete_robot(show_joints, show_frames)
        
    def get_backend_info(self) -> dict:
        """获取渲染后端信息"""
        return self.backend.get_backend_info()


__all__ = ['GeometryRenderer']
