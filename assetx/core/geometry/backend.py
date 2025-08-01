#!/usr/bin/env python3
"""
渲染后端管理器 - 负责渲染引擎的检测和选择
"""

import logging

logger = logging.getLogger(__name__)


class RenderBackend:
    """渲染后端管理器
    
    负责检测和管理不同的渲染后端：
    - Previewer (高级渲染器)
    - matplotlib (备用渲染器)
    """

    def __init__(self):
        """初始化渲染后端"""
        self._previewer = None
        self._matplotlib_available = False
        self._check_backends()
        
    def _check_backends(self) -> None:
        """检查可用的渲染后端"""
        # 检查Previewer
        try:
            from .....viewer.preview import Previewer
            self._previewer = Previewer()
            logger.info("Previewer后端可用")
        except ImportError:
            logger.warning("Previewer not available")
            
        # 检查matplotlib
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            self._matplotlib_available = True
            logger.info("matplotlib后端可用")
        except ImportError:
            logger.warning("matplotlib不可用，无可视化支持")
            
    @property
    def previewer(self):
        """获取Previewer实例"""
        return self._previewer
        
    @property
    def has_previewer(self) -> bool:
        """是否有Previewer后端"""
        return self._previewer is not None
        
    @property
    def has_matplotlib(self) -> bool:
        """是否有matplotlib后端"""
        return self._matplotlib_available
        
    @property
    def has_any_backend(self) -> bool:
        """是否有任何可用的渲染后端"""
        return self.has_previewer or self.has_matplotlib
        
    def get_backend_info(self) -> dict:
        """获取后端信息"""
        return {
            'previewer': '✓' if self.has_previewer else '✗',
            'matplotlib': '✓' if self.has_matplotlib else '✗',
            'total_backends': sum([self.has_previewer, self.has_matplotlib])
        }
        
    def render_with_previewer(self, mesh_file, show_axes: bool = True, show_wireframe: bool = False) -> bool:
        """使用Previewer渲染"""
        if not self.has_previewer:
            return False
            
        try:
            self._previewer.preview_mesh(
                mesh_file, 
                show_axes=show_axes,
                show_wireframe=show_wireframe
            )
            return True
        except Exception as e:
            logger.error(f"Previewer渲染失败: {e}")
            return False


__all__ = ['RenderBackend']
