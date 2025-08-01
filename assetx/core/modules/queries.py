#!/usr/bin/env python3
"""
AssetX 查询和遍历功能

提供Asset的查询、遍历和搜索功能
"""

from typing import TYPE_CHECKING, Callable, List, Optional

if TYPE_CHECKING:
    from ..asset import Asset
    from ..primitives import AssetPrim


class AssetQuery:
    """Asset查询和遍历功能的集合"""
    
    def __init__(self, asset: "Asset"):
        """初始化查询对象
        
        Args:
            asset: 目标Asset对象
        """
        self.asset = asset
    
    def get_all_prims(self) -> List["AssetPrim"]:
        """获取所有Prim（除伪根外）

        Returns:
            所有AssetPrim列表
        """
        return self.traverse(lambda p: not p.path.path_string == "/")
    
    def get_root_prims(self) -> List["AssetPrim"]:
        """获取所有根Prim

        Returns:
            根AssetPrim列表
        """
        pseudo_root = self.asset.get_pseudo_root()
        return pseudo_root.get_children() if pseudo_root else []
    
    def find_prims_by_type(self, type_name: str) -> List["AssetPrim"]:
        """根据类型查找Prim

        Args:
            type_name: 类型名称

        Returns:
            匹配的AssetPrim列表
        """
        return self.traverse(lambda p: p.type_name == type_name)

    def find_prims_by_schema(self, schema_name: str) -> List["AssetPrim"]:
        """根据Schema查找Prim

        Args:
            schema_name: Schema名称

        Returns:
            匹配的AssetPrim列表
        """
        return self.traverse(lambda p: p.has_api_schema(schema_name))
    
    def find_prims_by_name(self, name: str) -> List["AssetPrim"]:
        """根据名称查找Prim

        Args:
            name: Prim名称

        Returns:
            匹配的AssetPrim列表
        """
        return self.traverse(lambda p: p.name == name)
    
    def find_prims_by_path_pattern(self, pattern: str) -> List["AssetPrim"]:
        """根据路径模式查找Prim

        Args:
            pattern: 路径模式（支持通配符*）

        Returns:
            匹配的AssetPrim列表
        """
        import fnmatch
        return self.traverse(lambda p: fnmatch.fnmatch(p.path.path_string, pattern))

    def traverse(
        self, predicate: Callable[["AssetPrim"], bool] = None
    ) -> List["AssetPrim"]:
        """遍历所有Prim

        Args:
            predicate: 过滤条件函数

        Returns:
            符合条件的AssetPrim列表
        """
        stage = self.asset.get_stage()
        return stage.traverse(predicate) if stage else []
    
    # ============ 机器人特定查询 ============
    
    def get_links(self) -> List["AssetPrim"]:
        """获取所有链接Prim
        
        Returns:
            链接Prim列表
        """
        return self.find_prims_by_type("Link")
    
    def get_joints(self) -> List["AssetPrim"]:
        """获取所有关节Prim
        
        Returns:
            关节Prim列表
        """
        return self.find_prims_by_type("Joint")
    
    def get_meshes(self) -> List["AssetPrim"]:
        """获取所有网格Prim
        
        Returns:
            网格Prim列表
        """
        return self.find_prims_by_type("Mesh")
    
    def get_joint_by_name(self, joint_name: str) -> Optional["AssetPrim"]:
        """根据名称获取关节
        
        Args:
            joint_name: 关节名称
            
        Returns:
            关节Prim或None
        """
        joints = self.get_joints()
        for joint in joints:
            if joint.name == joint_name:
                return joint
        return None
    
    def get_link_by_name(self, link_name: str) -> Optional["AssetPrim"]:
        """根据名称获取链接
        
        Args:
            link_name: 链接名称
            
        Returns:
            链接Prim或None
        """
        links = self.get_links()
        for link in links:
            if link.name == link_name:
                return link
        return None
