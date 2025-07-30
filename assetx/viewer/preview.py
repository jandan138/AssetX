"""
Asset preview and visualization utilities
"""

from pathlib import Path
from typing import Optional, Union
import logging

logger = logging.getLogger(__name__)


class Previewer:
    """资产预览器"""
    
    def __init__(self, backend: str = 'trimesh'):
        """
        初始化预览器
        
        Args:
            backend: 可视化后端 ('trimesh', 'open3d')
        """
        self.backend = backend
        self.has_trimesh = False
        self.has_open3d = False
        self._check_dependencies()
        self._validate_backend()
    
    def _check_dependencies(self):
        """检查可用的可视化后端"""
        try:
            import trimesh
            self.has_trimesh = True
        except ImportError:
            pass
        
        try:
            import open3d as o3d
            self.has_open3d = True
        except ImportError:
            pass
    
    def _validate_backend(self) -> None:
        """验证可视化后端"""
        if self.backend == 'trimesh' and not self.has_trimesh:
            if self.has_open3d:
                logger.warning("trimesh not available, switching to open3d backend")
                self.backend = 'open3d'
            else:
                raise ImportError(
                    "No visualization backend available. "
                    "Install with: pip install 'assetx[viewer]' or pip install trimesh"
                )
        elif self.backend == 'open3d' and not self.has_open3d:
            if self.has_trimesh:
                logger.warning("open3d not available, switching to trimesh backend") 
                self.backend = 'trimesh'
            else:
                raise ImportError(
                    "open3d not available. "
                    "Install with: pip install 'assetx[viewer-open3d]' or pip install open3d"
                )
    
    def preview_mesh(self, mesh_path: Union[str, Path], 
                    show_axes: bool = True,
                    show_wireframe: bool = False) -> None:
        """预览网格文件"""
        mesh_path = Path(mesh_path)
        
        if not mesh_path.exists():
            raise FileNotFoundError(f"Mesh file not found: {mesh_path}")
        
        if self.backend == 'trimesh':
            self._preview_with_trimesh(mesh_path, show_axes, show_wireframe)
        elif self.backend == 'open3d':
            self._preview_with_open3d(mesh_path, show_axes, show_wireframe)
    
    def _preview_with_trimesh(self, mesh_path: Path, 
                             show_axes: bool, 
                             show_wireframe: bool) -> None:
        """使用trimesh预览"""
        try:
            import trimesh
            
            # 加载网格
            mesh = trimesh.load(str(mesh_path))
            
            # 创建场景
            scene = trimesh.Scene()
            
            # 添加网格
            if hasattr(mesh, 'visual'):
                scene.add_geometry(mesh)
            else:
                # 如果没有视觉信息，添加基本材质
                mesh.visual.face_colors = [200, 200, 250, 100]
                scene.add_geometry(mesh)
            
            # 添加坐标轴
            if show_axes:
                axes = trimesh.creation.axis(axis_length=0.1)
                scene.add_geometry(axes)
            
            # 显示
            scene.show()
            
            logger.info(f"Previewing mesh: {mesh_path}")
            logger.info(f"Vertices: {len(mesh.vertices) if hasattr(mesh, 'vertices') else 'N/A'}")
            logger.info(f"Faces: {len(mesh.faces) if hasattr(mesh, 'faces') else 'N/A'}")
            
        except Exception as e:
            logger.error(f"Failed to preview mesh with trimesh: {e}")
            raise
    
    def _preview_with_open3d(self, mesh_path: Path,
                           show_axes: bool,
                           show_wireframe: bool) -> None:
        """使用Open3D预览"""
        try:
            import open3d as o3d
            
            # 加载网格
            mesh = o3d.io.read_triangle_mesh(str(mesh_path))
            
            if len(mesh.vertices) == 0:
                raise ValueError("Empty mesh or unsupported format")
            
            # 计算法线
            mesh.compute_vertex_normals()
            
            # 设置颜色
            if not mesh.has_vertex_colors():
                mesh.paint_uniform_color([0.7, 0.7, 0.9])
            
            # 创建可视化对象列表
            geometries = [mesh]
            
            # 添加坐标轴
            if show_axes:
                coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
                geometries.append(coordinate_frame)
            
            # 线框模式
            if show_wireframe:
                wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
                wireframe.paint_uniform_color([0.0, 0.0, 0.0])
                geometries.append(wireframe)
            
            # 显示
            o3d.visualization.draw_geometries(
                geometries,
                window_name=f"AssetX Preview: {mesh_path.name}",
                width=800,
                height=600
            )
            
            logger.info(f"Previewing mesh: {mesh_path}")
            logger.info(f"Vertices: {len(mesh.vertices)}")
            logger.info(f"Triangles: {len(mesh.triangles)}")
            
        except Exception as e:
            logger.error(f"Failed to preview mesh with Open3D: {e}")
            raise
    
    def preview_urdf(self, urdf_path: Union[str, Path]) -> None:
        """预览URDF模型"""
        try:
            import urdfpy
            import trimesh
            
            urdf_path = Path(urdf_path)
            robot = urdfpy.URDF.load(str(urdf_path))
            
            # 创建场景
            scene = trimesh.Scene()
            
            # 添加所有视觉元素
            for link in robot.links:
                if link.visuals:
                    for visual in link.visuals:
                        if visual.geometry.mesh:
                            try:
                                # 获取网格路径
                                mesh_path = Path(urdf_path.parent) / visual.geometry.mesh.filename
                                if mesh_path.exists():
                                    mesh = trimesh.load(str(mesh_path))
                                    
                                    # 应用变换
                                    if visual.origin is not None:
                                        mesh.apply_transform(visual.origin)
                                    
                                    # 应用缩放
                                    if visual.geometry.mesh.scale is not None:
                                        mesh.apply_scale(visual.geometry.mesh.scale)
                                    
                                    scene.add_geometry(mesh, node_name=f"{link.name}_{visual.name}")
                                    
                            except Exception as e:
                                logger.warning(f"Failed to load mesh for {link.name}: {e}")
            
            # 添加坐标轴
            axes = trimesh.creation.axis(axis_length=0.1)
            scene.add_geometry(axes)
            
            # 显示
            scene.show()
            
            logger.info(f"Previewing URDF: {urdf_path}")
            logger.info(f"Links: {len(robot.links)}")
            logger.info(f"Joints: {len(robot.joints)}")
            
        except ImportError:
            raise ImportError("urdfpy is required for URDF preview. Install with: pip install urdfpy")
        except Exception as e:
            logger.error(f"Failed to preview URDF: {e}")
            raise
    
    def capture_screenshot(self, mesh_path: Union[str, Path],
                          output_path: Optional[Union[str, Path]] = None,
                          resolution: tuple = (800, 600)) -> Path:
        """捕获网格截图"""
        mesh_path = Path(mesh_path)
        
        if output_path is None:
            output_path = mesh_path.with_suffix('.png')
        else:
            output_path = Path(output_path)
        
        if self.backend == 'open3d':
            return self._capture_with_open3d(mesh_path, output_path, resolution)
        else:
            # trimesh截图需要额外设置
            logger.warning("Screenshot capture not implemented for trimesh backend")
            raise NotImplementedError("Screenshot capture not available for trimesh")
    
    def _capture_with_open3d(self, mesh_path: Path, 
                           output_path: Path,
                           resolution: tuple) -> Path:
        """使用Open3D捕获截图"""
        try:
            import open3d as o3d
            
            # 加载网格
            mesh = o3d.io.read_triangle_mesh(str(mesh_path))
            mesh.compute_vertex_normals()
            mesh.paint_uniform_color([0.7, 0.7, 0.9])
            
            # 创建可视化器
            vis = o3d.visualization.Visualizer()
            vis.create_window(width=resolution[0], height=resolution[1], visible=False)
            vis.add_geometry(mesh)
            
            # 渲染
            vis.poll_events()
            vis.update_renderer()
            
            # 捕获截图
            vis.capture_screen_image(str(output_path))
            vis.destroy_window()
            
            logger.info(f"Screenshot saved to: {output_path}")
            return output_path
            
        except Exception as e:
            logger.error(f"Failed to capture screenshot: {e}")
            raise
    
    def get_mesh_statistics(self, mesh_path: Union[str, Path]) -> dict:
        """获取网格统计信息"""
        mesh_path = Path(mesh_path)
        
        try:
            if self.backend == 'trimesh':
                import trimesh
                mesh = trimesh.load(str(mesh_path))
                
                stats = {
                    'path': str(mesh_path),
                    'vertices': len(mesh.vertices) if hasattr(mesh, 'vertices') else 0,
                    'faces': len(mesh.faces) if hasattr(mesh, 'faces') else 0,
                    'volume': float(mesh.volume) if hasattr(mesh, 'volume') else 0.0,
                    'surface_area': float(mesh.area) if hasattr(mesh, 'area') else 0.0,
                    'is_watertight': mesh.is_watertight if hasattr(mesh, 'is_watertight') else False,
                    'bounds': mesh.bounds.tolist() if hasattr(mesh, 'bounds') else None,
                    'center_mass': mesh.center_mass.tolist() if hasattr(mesh, 'center_mass') else None
                }
                
            elif self.backend == 'open3d':
                import open3d as o3d
                mesh = o3d.io.read_triangle_mesh(str(mesh_path))
                
                stats = {
                    'path': str(mesh_path),
                    'vertices': len(mesh.vertices),
                    'faces': len(mesh.triangles),
                    'volume': mesh.get_volume() if mesh.is_watertight() else 0.0,
                    'surface_area': mesh.get_surface_area(),
                    'is_watertight': mesh.is_watertight(),
                    'bounds': [mesh.get_min_bound().tolist(), mesh.get_max_bound().tolist()],
                    'center': mesh.get_center().tolist()
                }
            
            return stats
            
        except Exception as e:
            logger.error(f"Failed to get mesh statistics: {e}")
            return {'path': str(mesh_path), 'error': str(e)}
