#!/usr/bin/env python3
"""
matplotlib绘图工具 - 提供基础几何形状绘制功能
"""

from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)


class MatplotlibPlotter:
    """matplotlib绘图工具
    
    提供各种基础几何形状的绘制功能：
    - 立方体、球体、圆柱体
    - 坐标系、连接线
    - 3D场景设置
    """

    def __init__(self):
        """初始化绘图工具"""
        self._check_matplotlib()
        
    def _check_matplotlib(self) -> bool:
        """检查matplotlib可用性"""
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            import numpy as np
            return True
        except ImportError:
            logger.error("matplotlib不可用")
            return False
            
    def create_3d_figure(self, title: str = "AssetX 3D Plot", figsize: tuple = (8, 6)):
        """创建3D图形"""
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            
            fig = plt.figure(figsize=figsize)
            ax = fig.add_subplot(111, projection='3d')
            ax.set_title(title)
            return fig, ax
        except ImportError:
            logger.error("无法创建3D图形：matplotlib不可用")
            return None, None
            
    def plot_box(self, ax, center: List[float], size: List[float], color: str = 'blue', label: str = ""):
        """绘制立方体"""
        try:
            import numpy as np
            x, y, z = center
            dx, dy, dz = size
            
            # 立方体的8个顶点
            vertices = np.array([
                [x-dx/2, y-dy/2, z-dz/2], [x+dx/2, y-dy/2, z-dz/2],
                [x+dx/2, y+dy/2, z-dz/2], [x-dx/2, y+dy/2, z-dz/2],
                [x-dx/2, y-dy/2, z+dz/2], [x+dx/2, y-dy/2, z+dz/2],
                [x+dx/2, y+dy/2, z+dz/2], [x-dx/2, y+dy/2, z+dz/2],
            ])
            
            # 立方体的12条边
            edges = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],
                    [0,4],[1,5],[2,6],[3,7]]
            
            for i, edge in enumerate(edges):
                points = vertices[edge]
                label_text = label if i == 0 else ""  # 只在第一条边显示标签
                ax.plot3D(*points.T, color=color, linewidth=3, label=label_text)
                
        except Exception as e:
            logger.error(f"绘制立方体失败: {e}")
            
    def plot_sphere(self, ax, center: List[float], radius: float, color: str = 'red', label: str = ""):
        """绘制球体"""
        try:
            import numpy as np
            x, y, z = center
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            xs = x + radius * np.outer(np.cos(u), np.sin(v))
            ys = y + radius * np.outer(np.sin(u), np.sin(v))
            zs = z + radius * np.outer(np.ones(np.size(u)), np.cos(v))
            ax.plot_surface(xs, ys, zs, color=color, alpha=0.7, label=label)
            
        except Exception as e:
            logger.error(f"绘制球体失败: {e}")
            
    def plot_cylinder(self, ax, center: List[float], radius: float, height: float, color: str = 'green', label: str = ""):
        """绘制圆柱体"""
        try:
            import numpy as np
            x, y, z = center
            theta = np.linspace(0, 2*np.pi, 20)
            z_cyl = np.linspace(z - height/2, z + height/2, 20)
            theta_mesh, z_mesh = np.meshgrid(theta, z_cyl)
            xs = x + radius * np.cos(theta_mesh)
            ys = y + radius * np.sin(theta_mesh)
            zs = z_mesh
            ax.plot_surface(xs, ys, zs, color=color, alpha=0.7, label=label)
            
        except Exception as e:
            logger.error(f"绘制圆柱体失败: {e}")
            
    def draw_coordinate_frame(self, ax, position: List[float], size: float = 0.1):
        """绘制坐标系"""
        try:
            x, y, z = position
            # X轴 - 红色
            ax.quiver(x, y, z, size, 0, 0, color='red', alpha=0.8, arrow_length_ratio=0.1)
            # Y轴 - 绿色  
            ax.quiver(x, y, z, 0, size, 0, color='green', alpha=0.8, arrow_length_ratio=0.1)
            # Z轴 - 蓝色
            ax.quiver(x, y, z, 0, 0, size, color='blue', alpha=0.8, arrow_length_ratio=0.1)
            
        except Exception as e:
            logger.error(f"绘制坐标系失败: {e}")
            
    def draw_connection_line(self, ax, pos1: List[float], pos2: List[float], 
                           color: str = 'black', style: str = '--', linewidth: int = 2, label: str = ""):
        """绘制连接线"""
        try:
            ax.plot3D([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]], 
                     color=color, linestyle=style, linewidth=linewidth, alpha=0.6, label=label)
        except Exception as e:
            logger.error(f"绘制连接线失败: {e}")
            
    def setup_3d_scene(self, ax, max_range: float = 0.6, show_axes: bool = True, show_grid: bool = True):
        """设置3D场景"""
        try:
            if show_axes:
                ax.set_xlabel('X (m)', fontsize=12)
                ax.set_ylabel('Y (m)', fontsize=12) 
                ax.set_zlabel('Z (m)', fontsize=12)
                
            # 设置等比例和合适的视角
            ax.set_xlim([-max_range, max_range])
            ax.set_ylim([-max_range, max_range])
            ax.set_zlim([0, max_range * 2])
            ax.set_box_aspect([1,1,1])
            
            if show_grid:
                ax.grid(True, alpha=0.3)
                
        except Exception as e:
            logger.error(f"设置3D场景失败: {e}")
            
    def finalize_plot(self, ax, show_legend: bool = True):
        """完成绘图"""
        try:
            import matplotlib.pyplot as plt
            
            if show_legend:
                ax.legend(loc='upper left', bbox_to_anchor=(0, 1))
                
            plt.tight_layout()
            plt.show()
            
        except Exception as e:
            logger.error(f"完成绘图失败: {e}")


__all__ = ['MatplotlibPlotter']
