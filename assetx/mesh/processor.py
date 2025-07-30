"""
Mesh processing utilities
"""

from pathlib import Path
from typing import List, Optional, Tuple, Union
import logging
import numpy as np

logger = logging.getLogger(__name__)


class MeshProcessor:
    """网格处理器"""
    
    def __init__(self):
        self.supported_formats = ['.obj', '.stl', '.ply', '.dae', '.glb', '.gltf']
        self._check_dependencies()
    
    def _check_dependencies(self):
        """检查可选依赖"""
        self.has_trimesh = False
        try:
            import trimesh
            self.has_trimesh = True
        except ImportError:
            pass
    
    def _require_trimesh(self):
        """检查trimesh是否可用"""
        if not self.has_trimesh:
            raise ImportError(
                "trimesh is required for mesh processing. "
                "Install with: pip install 'assetx[mesh]' or pip install trimesh"
            )
    
    def simplify_mesh(self, 
                     mesh_path: Union[str, Path], 
                     target_faces: int,
                     output_path: Optional[Union[str, Path]] = None) -> Path:
        """简化网格，减少面数"""
        self._require_trimesh()
        
        try:
            import trimesh
            
            mesh_path = Path(mesh_path)
            mesh = trimesh.load(str(mesh_path))
            
            if not hasattr(mesh, 'faces'):
                raise ValueError("Loaded object is not a mesh")
            
            original_faces = len(mesh.faces)
            if original_faces <= target_faces:
                logger.info(f"Mesh already has {original_faces} faces, less than target {target_faces}")
                return mesh_path
            
            # 使用trimesh的简化功能
            simplified = mesh.simplify_quadric_decimation(target_faces)
            
            # 生成输出路径
            if output_path is None:
                output_path = mesh_path.with_suffix(f'_simplified{mesh_path.suffix}')
            else:
                output_path = Path(output_path)
            
            # 保存简化后的网格
            simplified.export(str(output_path))
            
            logger.info(f"Simplified mesh from {original_faces} to {len(simplified.faces)} faces")
            return output_path
            
        except Exception as e:
            raise RuntimeError(f"Mesh simplification failed: {e}")
    
    def center_mesh(self, 
                   mesh_path: Union[str, Path],
                   output_path: Optional[Union[str, Path]] = None) -> Path:
        """将网格中心化到原点"""
        self._require_trimesh()
        
        try:
            import trimesh
            
            mesh_path = Path(mesh_path)
            mesh = trimesh.load(str(mesh_path))
            
            # 计算包围盒中心
            center = mesh.bounds.mean(axis=0)
            
            # 平移网格到原点
            mesh.vertices -= center
            
            # 生成输出路径
            if output_path is None:
                output_path = mesh_path.with_suffix(f'_centered{mesh_path.suffix}')
            else:
                output_path = Path(output_path)
            
            # 保存中心化后的网格
            mesh.export(str(output_path))
            
            logger.info(f"Centered mesh, translation: {center}")
            return output_path
            
        except Exception as e:
            raise RuntimeError(f"Mesh centering failed: {e}")
    
    def scale_mesh(self, 
                  mesh_path: Union[str, Path],
                  scale_factor: Union[float, List[float]],
                  output_path: Optional[Union[str, Path]] = None) -> Path:
        """缩放网格"""
        try:
            import trimesh
            
            mesh_path = Path(mesh_path)
            mesh = trimesh.load(str(mesh_path))
            
            # 应用缩放
            if isinstance(scale_factor, (int, float)):
                mesh.apply_scale(scale_factor)
            else:
                # 非均匀缩放
                scale_matrix = np.diag(list(scale_factor) + [1.0])
                mesh.apply_transform(scale_matrix)
            
            # 生成输出路径
            if output_path is None:
                scale_str = f"{scale_factor}" if isinstance(scale_factor, (int, float)) else "scaled"
                output_path = mesh_path.with_suffix(f'_{scale_str}{mesh_path.suffix}')
            else:
                output_path = Path(output_path)
            
            # 保存缩放后的网格
            mesh.export(str(output_path))
            
            logger.info(f"Scaled mesh by factor: {scale_factor}")
            return output_path
            
        except ImportError:
            raise ImportError("trimesh is required for mesh processing")
        except Exception as e:
            raise RuntimeError(f"Mesh scaling failed: {e}")
    
    def generate_collision_mesh(self,
                              mesh_path: Union[str, Path],
                              method: str = 'convex_hull',
                              output_path: Optional[Union[str, Path]] = None) -> Path:
        """生成碰撞网格"""
        try:
            import trimesh
            
            mesh_path = Path(mesh_path)
            mesh = trimesh.load(str(mesh_path))
            
            if method == 'convex_hull':
                collision_mesh = mesh.convex_hull
            elif method == 'bounding_box':
                collision_mesh = mesh.bounding_box_oriented
            elif method == 'simplified':
                # 大幅简化网格作为碰撞体
                target_faces = max(100, len(mesh.faces) // 10)
                collision_mesh = mesh.simplify_quadric_decimation(target_faces)
            else:
                raise ValueError(f"Unsupported collision generation method: {method}")
            
            # 生成输出路径
            if output_path is None:
                output_path = mesh_path.with_suffix(f'_collision{mesh_path.suffix}')
            else:
                output_path = Path(output_path)
            
            # 保存碰撞网格
            collision_mesh.export(str(output_path))
            
            logger.info(f"Generated collision mesh using {method} method")
            return output_path
            
        except ImportError:
            raise ImportError("trimesh is required for mesh processing")
        except Exception as e:
            raise RuntimeError(f"Collision mesh generation failed: {e}")
    
    def normalize_units(self,
                       mesh_path: Union[str, Path],
                       source_unit: str = 'mm',
                       target_unit: str = 'm',
                       output_path: Optional[Union[str, Path]] = None) -> Path:
        """单位标准化"""
        
        # 单位转换表（转换为米）
        unit_scales = {
            'mm': 0.001,
            'cm': 0.01,
            'm': 1.0,
            'km': 1000.0,
            'in': 0.0254,
            'ft': 0.3048
        }
        
        if source_unit not in unit_scales or target_unit not in unit_scales:
            raise ValueError(f"Unsupported units. Supported: {list(unit_scales.keys())}")
        
        # 计算缩放因子
        scale_factor = unit_scales[source_unit] / unit_scales[target_unit]
        
        if abs(scale_factor - 1.0) < 1e-10:
            logger.info("No scaling needed, units are the same")
            return Path(mesh_path)
        
        return self.scale_mesh(mesh_path, scale_factor, output_path)
    
    def get_mesh_info(self, mesh_path: Union[str, Path]) -> dict:
        """获取网格信息"""
        if not self.has_trimesh:
            return {
                'path': str(mesh_path),
                'error': 'trimesh not available. Install with: pip install trimesh'
            }
        
        try:
            import trimesh
            
            mesh = trimesh.load(str(mesh_path))
            
            info = {
                'path': str(mesh_path),
                'vertices': len(mesh.vertices) if hasattr(mesh, 'vertices') else 0,
                'faces': len(mesh.faces) if hasattr(mesh, 'faces') else 0,
                'is_watertight': mesh.is_watertight if hasattr(mesh, 'is_watertight') else False,
                'volume': float(mesh.volume) if hasattr(mesh, 'volume') else 0.0,
                'surface_area': float(mesh.area) if hasattr(mesh, 'area') else 0.0,
                'bounds': mesh.bounds.tolist() if hasattr(mesh, 'bounds') else None,
                'center_mass': mesh.center_mass.tolist() if hasattr(mesh, 'center_mass') else None
            }
            
            return info
            
        except Exception as e:
            logger.error(f"Failed to analyze mesh: {e}")
            return {'path': str(mesh_path), 'error': str(e)}
