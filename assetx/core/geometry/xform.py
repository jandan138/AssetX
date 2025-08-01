#!/usr/bin/env python3
"""
AssetXform - USD风格的变换节点

对应USD的UsdGeomXform，提供空间变换功能。
"""

import numpy as np
from typing import List, Optional, Tuple

from ..primitives import AssetPrim, SdfPath


class AssetXform(AssetPrim):
    """USD风格的变换节点类
    
    对应USD的UsdGeomXform，提供：
    - 平移、旋转、缩放变换
    - 变换矩阵管理
    - 层次化变换组合
    """

    def __init__(self, stage: 'AssetStage', path: SdfPath, name: str = ""):
        """初始化AssetXform
        
        Args:
            stage: 所属Stage
            path: SDF路径
            name: 变换节点名称（仅用于显示，实际名称来自路径）
        """
        super().__init__(stage, path)
        self.set_type_name("Xform")
        self._setup_transform_attributes()
    
    def _setup_transform_attributes(self) -> None:
        """设置USD风格的变换属性"""
        # 对应UsdGeomXform的变换属性
        self.create_attribute("xformOpOrder", "token[]")  # 变换操作顺序
        
        # 基本变换操作
        self.create_attribute("xformOp:translate", "float3")  # 平移
        self.create_attribute("xformOp:rotateXYZ", "float3")  # 欧拉角旋转
        self.create_attribute("xformOp:scale", "float3")  # 缩放
        
        # 变换矩阵
        self.create_attribute("xformOp:transform", "matrix4d")  # 4x4变换矩阵
        
        # 设置默认变换顺序
        self.get_attribute("xformOpOrder").set([
            "xformOp:translate",
            "xformOp:rotateXYZ", 
            "xformOp:scale"
        ])
        
        # 设置默认值
        self.set_translation([0.0, 0.0, 0.0])
        self.set_rotation([0.0, 0.0, 0.0])
        self.set_scale([1.0, 1.0, 1.0])
        
    def set_translation(self, translation: List[float]) -> None:
        """设置平移
        
        Args:
            translation: [x, y, z] 平移向量
        """
        if len(translation) != 3:
            raise ValueError("Translation must have 3 components")
        self.get_attribute("xformOp:translate").set(translation)
        
    def get_translation(self) -> List[float]:
        """获取平移"""
        attr = self.get_attribute("xformOp:translate")
        return attr.get() if attr.get() else [0.0, 0.0, 0.0]
        
    def set_rotation(self, rotation: List[float]) -> None:
        """设置旋转（欧拉角，度数）
        
        Args:
            rotation: [rx, ry, rz] 欧拉角（度数）
        """
        if len(rotation) != 3:
            raise ValueError("Rotation must have 3 components")
        self.get_attribute("xformOp:rotateXYZ").set(rotation)
        
    def get_rotation(self) -> List[float]:
        """获取旋转（欧拉角，度数）"""
        attr = self.get_attribute("xformOp:rotateXYZ")
        return attr.get() if attr.get() else [0.0, 0.0, 0.0]
        
    def set_scale(self, scale: List[float]) -> None:
        """设置缩放
        
        Args:
            scale: [sx, sy, sz] 缩放因子
        """
        if len(scale) != 3:
            raise ValueError("Scale must have 3 components")
        self.get_attribute("xformOp:scale").set(scale)
        
    def get_scale(self) -> List[float]:
        """获取缩放"""
        attr = self.get_attribute("xformOp:scale")
        return attr.get() if attr.get() else [1.0, 1.0, 1.0]
        
    def set_transform_matrix(self, matrix: np.ndarray) -> None:
        """设置4x4变换矩阵
        
        Args:
            matrix: 4x4变换矩阵
        """
        if matrix.shape != (4, 4):
            raise ValueError("Transform matrix must be 4x4")
        
        # 存储为扁平列表（USD惯例）
        matrix_flat = matrix.flatten().tolist()
        self.get_attribute("xformOp:transform").set(matrix_flat)
        
    def get_transform_matrix(self) -> np.ndarray:
        """获取4x4变换矩阵"""
        attr = self.get_attribute("xformOp:transform")
        if attr.get():
            matrix_flat = attr.get()
            return np.array(matrix_flat).reshape(4, 4)
        else:
            # 从基本变换计算矩阵
            return self.compute_transform_matrix()
            
    def compute_transform_matrix(self) -> np.ndarray:
        """从基本变换计算4x4矩阵"""
        # 获取基本变换
        translation = np.array(self.get_translation())
        rotation = np.array(self.get_rotation()) * np.pi / 180.0  # 转换为弧度
        scale = np.array(self.get_scale())
        
        # 构建变换矩阵
        # 旋转矩阵（XYZ欧拉角）
        rx, ry, rz = rotation
        
        # X轴旋转
        Rx = np.array([
            [1, 0, 0, 0],
            [0, np.cos(rx), -np.sin(rx), 0],
            [0, np.sin(rx), np.cos(rx), 0],
            [0, 0, 0, 1]
        ])
        
        # Y轴旋转
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry), 0],
            [0, 1, 0, 0],
            [-np.sin(ry), 0, np.cos(ry), 0],
            [0, 0, 0, 1]
        ])
        
        # Z轴旋转
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0, 0],
            [np.sin(rz), np.cos(rz), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # 缩放矩阵
        S = np.array([
            [scale[0], 0, 0, 0],
            [0, scale[1], 0, 0],
            [0, 0, scale[2], 0],
            [0, 0, 0, 1]
        ])
        
        # 平移矩阵
        T = np.array([
            [1, 0, 0, translation[0]],
            [0, 1, 0, translation[1]],
            [0, 0, 1, translation[2]],
            [0, 0, 0, 1]
        ])
        
        # 组合变换：T * R * S
        R = Rz @ Ry @ Rx
        transform = T @ R @ S
        
        return transform
        
    def transform_point(self, point: np.ndarray) -> np.ndarray:
        """变换一个3D点
        
        Args:
            point: 3D点 [x, y, z]
            
        Returns:
            变换后的3D点
        """
        if len(point) != 3:
            raise ValueError("Point must have 3 components")
            
        # 转换为齐次坐标
        point_homo = np.array([point[0], point[1], point[2], 1.0])
        
        # 应用变换
        matrix = self.get_transform_matrix()
        transformed = matrix @ point_homo
        
        # 返回3D坐标
        return transformed[:3]
        
    def transform_points(self, points: np.ndarray) -> np.ndarray:
        """批量变换3D点
        
        Args:
            points: 3D点数组 (N, 3)
            
        Returns:
            变换后的3D点数组 (N, 3)
        """
        if points.shape[1] != 3:
            raise ValueError("Points must have shape (N, 3)")
            
        # 转换为齐次坐标
        n_points = points.shape[0]
        points_homo = np.ones((n_points, 4))
        points_homo[:, :3] = points
        
        # 应用变换
        matrix = self.get_transform_matrix()
        transformed = (matrix @ points_homo.T).T
        
        # 返回3D坐标
        return transformed[:, :3]
        
    def reset_transform(self) -> None:
        """重置变换为单位变换"""
        self.set_translation([0.0, 0.0, 0.0])
        self.set_rotation([0.0, 0.0, 0.0])
        self.set_scale([1.0, 1.0, 1.0])
        
    def __repr__(self) -> str:
        """字符串表示"""
        t = self.get_translation()
        r = self.get_rotation()
        s = self.get_scale()
        
        return (f"AssetXform(path='{self.path}', "
                f"t=[{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}], "
                f"r=[{r[0]:.1f}°, {r[1]:.1f}°, {r[2]:.1f}°], "
                f"s=[{s[0]:.2f}, {s[1]:.2f}, {s[2]:.2f}])")
