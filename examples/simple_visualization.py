#!/usr/bin/env python3
"""
临时的几何体可视化功能

在没有trimesh/open3d的情况下，使用matplotlib提供基本的2D/3D可视化
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from pathlib import Path
from typing import List, Optional, Tuple

from assetx.core.geometry.mesh import AssetMesh
from assetx.core.geometry.xform import AssetXform


class SimpleGeometryVisualizer:
    """简单的几何体可视化器（使用matplotlib）"""
    
    def __init__(self):
        self.fig = None
        self.ax = None
        
    def create_3d_plot(self, title: str = "AssetX Geometry") -> None:
        """创建3D绘图窗口"""
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title(title)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y') 
        self.ax.set_zlabel('Z')
        
    def plot_box(self, center: List[float] = [0, 0, 0], 
                 size: List[float] = [1, 1, 1], 
                 color: str = 'blue', alpha: float = 0.3) -> None:
        """绘制立方体"""
        if not self.ax:
            self.create_3d_plot()
            
        x, y, z = center
        dx, dy, dz = size
        
        # 定义立方体的8个顶点
        vertices = np.array([
            [x-dx/2, y-dy/2, z-dz/2],  # 0
            [x+dx/2, y-dy/2, z-dz/2],  # 1
            [x+dx/2, y+dy/2, z-dz/2],  # 2
            [x-dx/2, y+dy/2, z-dz/2],  # 3
            [x-dx/2, y-dy/2, z+dz/2],  # 4
            [x+dx/2, y-dy/2, z+dz/2],  # 5
            [x+dx/2, y+dy/2, z+dz/2],  # 6
            [x-dx/2, y+dy/2, z+dz/2],  # 7
        ])
        
        # 定义立方体的12条边
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # 底面
            [4, 5], [5, 6], [6, 7], [7, 4],  # 顶面
            [0, 4], [1, 5], [2, 6], [3, 7],  # 竖直边
        ]
        
        # 绘制边框
        for edge in edges:
            points = vertices[edge]
            self.ax.plot3D(*points.T, color=color, alpha=alpha*2, linewidth=2)
            
        # 绘制面（简化版）
        self.ax.scatter(*vertices.T, color=color, alpha=alpha, s=20)
        
    def plot_sphere(self, center: List[float] = [0, 0, 0], 
                   radius: float = 1.0, 
                   color: str = 'red', alpha: float = 0.3) -> None:
        """绘制球体"""
        if not self.ax:
            self.create_3d_plot()
            
        x, y, z = center
        
        # 创建球体表面
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        xs = x + radius * np.outer(np.cos(u), np.sin(v))
        ys = y + radius * np.outer(np.sin(u), np.sin(v))
        zs = z + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        self.ax.plot_surface(xs, ys, zs, color=color, alpha=alpha)
        
    def plot_cylinder(self, center: List[float] = [0, 0, 0], 
                     radius: float = 1.0, height: float = 2.0,
                     color: str = 'green', alpha: float = 0.3) -> None:
        """绘制圆柱体"""
        if not self.ax:
            self.create_3d_plot()
            
        x, y, z = center
        
        # 创建圆柱体表面
        theta = np.linspace(0, 2*np.pi, 20)
        z_cyl = np.linspace(z - height/2, z + height/2, 20)
        theta_mesh, z_mesh = np.meshgrid(theta, z_cyl)
        
        xs = x + radius * np.cos(theta_mesh)
        ys = y + radius * np.sin(theta_mesh)
        zs = z_mesh
        
        self.ax.plot_surface(xs, ys, zs, color=color, alpha=alpha)
        
    def plot_coordinate_frame(self, origin: List[float] = [0, 0, 0], 
                            scale: float = 1.0) -> None:
        """绘制坐标系"""
        if not self.ax:
            self.create_3d_plot()
            
        x, y, z = origin
        
        # X轴 - 红色
        self.ax.quiver(x, y, z, scale, 0, 0, color='red', arrow_length_ratio=0.1)
        # Y轴 - 绿色  
        self.ax.quiver(x, y, z, 0, scale, 0, color='green', arrow_length_ratio=0.1)
        # Z轴 - 蓝色
        self.ax.quiver(x, y, z, 0, 0, scale, color='blue', arrow_length_ratio=0.1)
        
    def plot_asset_mesh(self, mesh: AssetMesh, transform: Optional[AssetXform] = None) -> None:
        """绘制AssetMesh对象"""
        if not self.ax:
            self.create_3d_plot()
            
        # 获取几何体类型
        geom_type = mesh.get_geometry_type()
        if not geom_type:
            print(f"跳过无几何类型的mesh: {mesh.path}")
            return
            
        # 获取变换
        center = [0, 0, 0]
        if transform:
            center = transform.get_translation()
            
        # 根据类型绘制
        if geom_type.value == "box":
            size_attr = mesh.get_attribute("geom:size")
            size = size_attr.get() if size_attr and size_attr.get() else [1, 1, 1]
            self.plot_box(center, size, color='skyblue')
            
        elif geom_type.value == "sphere":
            size_attr = mesh.get_attribute("geom:size")
            radius = size_attr.get()[0] if size_attr and size_attr.get() else 1.0
            self.plot_sphere(center, radius, color='lightcoral')
            
        elif geom_type.value == "cylinder":
            size_attr = mesh.get_attribute("geom:size")
            if size_attr and size_attr.get():
                size = size_attr.get()
                radius = size[0]
                height = size[2]
            else:
                radius, height = 1.0, 2.0
            self.plot_cylinder(center, radius, height, color='lightgreen')
            
        # 添加坐标系
        self.plot_coordinate_frame(center, scale=0.5)
        
    def show(self) -> None:
        """显示图形"""
        if self.fig:
            self.ax.set_box_aspect([1,1,1])  # 等比例
            plt.tight_layout()
            plt.show()
        else:
            print("没有可显示的图形")
            
    def save(self, filename: str) -> None:
        """保存图形"""
        if self.fig:
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"图形已保存到: {filename}")
        else:
            print("没有可保存的图形")


def demo_simple_visualization():
    """演示简单可视化功能"""
    print("=== AssetX 简单几何体可视化演示 ===\n")
    
    visualizer = SimpleGeometryVisualizer()
    visualizer.create_3d_plot("AssetX 几何体演示")
    
    # 绘制不同几何体
    visualizer.plot_box([0, 0, 0], [2, 1, 0.5], 'blue', 0.3)
    visualizer.plot_sphere([3, 0, 0], 1.0, 'red', 0.3)
    visualizer.plot_cylinder([0, 3, 0], 0.5, 2.0, 'green', 0.3)
    
    # 添加坐标系
    visualizer.plot_coordinate_frame([0, 0, 0], 1.5)
    
    print("显示可视化窗口...")
    visualizer.show()


if __name__ == "__main__":
    demo_simple_visualization()
