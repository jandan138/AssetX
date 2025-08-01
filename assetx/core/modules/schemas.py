#!/usr/bin/env python3
"""
AssetX 机器人Schema应用功能

提供创建和操作机器人特定Schema的功能
"""

from typing import TYPE_CHECKING, List, Optional, Union

if TYPE_CHECKING:
    from ..asset import Asset
    from ..primitives import AssetPrim, SdfPath


class AssetSchema:
    """机器人Schema应用功能的集合"""
    
    def __init__(self, asset: "Asset"):
        """初始化Schema对象
        
        Args:
            asset: 目标Asset对象
        """
        self.asset = asset
    
    def create_robot_prim(
        self, path: Union[str, "SdfPath"], name: str = None
    ) -> "AssetPrim":
        """创建机器人Prim

        Args:
            path: Prim路径
            name: 机器人名称

        Returns:
            创建的AssetPrim对象
        """
        prim = self.asset.define_prim(path, "Robot")
        prim.apply_api_schema("RobotAPI")
        if name:
            prim.set_metadata("displayName", name)
        return prim

    def create_link_prim(
        self, path: Union[str, "SdfPath"], mass: float = None, inertia: List[float] = None
    ) -> "AssetPrim":
        """创建链接Prim

        Args:
            path: Prim路径
            mass: 质量
            inertia: 惯性矩阵

        Returns:
            创建的AssetPrim对象
        """
        prim = self.asset.define_prim(path, "Link")
        prim.apply_api_schema("LinkAPI")
        prim.apply_api_schema("PhysicsAPI")

        if mass is not None:
            mass_attr = prim.create_attribute("physics:mass", "float")
            mass_attr.set(mass)

        if inertia is not None:
            inertia_attr = prim.create_attribute("physics:inertia", "matrix3d")
            inertia_attr.set(inertia)

        return prim

    def create_joint_prim(
        self,
        path: Union[str, "SdfPath"],
        joint_type: str = "revolute",
        parent_link: str = None,
        child_link: str = None,
        axis: List[float] = None,
    ) -> "AssetPrim":
        """创建关节Prim

        Args:
            path: Prim路径
            joint_type: 关节类型
            parent_link: 父链接名称
            child_link: 子链接名称
            axis: 关节轴向

        Returns:
            创建的AssetPrim对象
        """
        from ..primitives import SdfPath
        
        prim = self.asset.define_prim(path, "Joint")
        prim.apply_api_schema("JointAPI")

        # 设置关节类型
        type_attr = prim.create_attribute("joint:type", "string")
        type_attr.set(joint_type)

        # 设置父子关系
        if parent_link:
            parent_rel = prim.create_relationship("joint:parentLink")
            parent_rel.add_target(SdfPath(f"/{parent_link}"))

        if child_link:
            child_rel = prim.create_relationship("joint:childLink")
            child_rel.add_target(SdfPath(f"/{child_link}"))

        # 设置轴向
        if axis:
            axis_attr = prim.create_attribute("joint:axis", "vector3")
            axis_attr.set(axis)

        return prim
    
    def create_mesh_prim(
        self, 
        path: Union[str, "SdfPath"], 
        mesh_file: str = None,
        scale: List[float] = None
    ) -> "AssetPrim":
        """创建网格Prim

        Args:
            path: Prim路径
            mesh_file: 网格文件路径
            scale: 缩放比例

        Returns:
            创建的AssetPrim对象
        """
        prim = self.asset.define_prim(path, "Mesh")
        prim.apply_api_schema("MeshAPI")

        if mesh_file:
            mesh_attr = prim.create_attribute("mesh:file", "string")
            mesh_attr.set(mesh_file)

        if scale:
            scale_attr = prim.create_attribute("mesh:scale", "vector3")
            scale_attr.set(scale)

        return prim
    
    def create_collision_prim(
        self, 
        path: Union[str, "SdfPath"],
        geometry_type: str = "mesh",
        geometry_data: dict = None
    ) -> "AssetPrim":
        """创建碰撞Prim

        Args:
            path: Prim路径
            geometry_type: 几何类型 (mesh, box, sphere, cylinder)
            geometry_data: 几何数据

        Returns:
            创建的AssetPrim对象
        """
        prim = self.asset.define_prim(path, "Collision")
        prim.apply_api_schema("CollisionAPI")

        # 设置几何类型
        type_attr = prim.create_attribute("collision:geometryType", "string")
        type_attr.set(geometry_type)

        # 设置几何数据
        if geometry_data:
            for key, value in geometry_data.items():
                attr = prim.create_attribute(f"collision:{key}", "float" if isinstance(value, (int, float)) else "string")
                attr.set(value)

        return prim
    
    def create_visual_prim(
        self, 
        path: Union[str, "SdfPath"],
        material: dict = None,
        geometry_ref: str = None
    ) -> "AssetPrim":
        """创建视觉Prim

        Args:
            path: Prim路径
            material: 材质信息
            geometry_ref: 几何引用

        Returns:
            创建的AssetPrim对象
        """
        prim = self.asset.define_prim(path, "Visual")
        prim.apply_api_schema("VisualAPI")

        if material:
            for key, value in material.items():
                attr = prim.create_attribute(f"material:{key}", "float" if isinstance(value, (int, float)) else "string")
                attr.set(value)

        if geometry_ref:
            geom_attr = prim.create_attribute("visual:geometryRef", "string")
            geom_attr.set(geometry_ref)

        return prim
