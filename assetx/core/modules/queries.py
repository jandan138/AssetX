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
    
    def get_children_by_type(self, parent_prim: "AssetPrim", type_name: str) -> List["AssetPrim"]:
        """获取指定Prim的特定类型子节点

        Args:
            parent_prim: 父Prim
            type_name: 子节点类型名称

        Returns:
            匹配的子Prim列表
        """
        children = parent_prim.get_children()
        return [child for child in children if child.type_name == type_name]
    
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
        # 首先尝试查找标准的Link类型
        links = self.find_prims_by_type("Link")
        
        # 如果没有找到标准Link，尝试智能识别USD中的链接
        if not links:
            # 在USD文件中，链接通常是:
            # 1. 类型为"Prim"或"Xform"
            # 2. 名称包含"link"
            # 3. 不是visuals/collisions子组件
            # 4. 通常在机器人根目录下的第二层
            
            potential_links = []
            
            # 方法1：查找名称包含link的Prim
            all_prims = self.get_all_prims()
            for prim in all_prims:
                path_str = str(prim.path)
                name_lower = prim.name.lower()
                
                # 检查是否是链接候选
                is_link_candidate = (
                    'link' in name_lower and  # 名称包含link
                    'visuals' not in path_str and  # 不是visuals子组件
                    'collisions' not in path_str and  # 不是collisions子组件
                    prim.type_name in ['Prim', 'Xform']  # 类型是Prim或Xform
                )
                
                if is_link_candidate:
                    potential_links.append(prim)
            
            # 方法2：如果还是没找到，查找所有非子组件的Prim
            if not potential_links:
                for prim in all_prims:
                    path_str = str(prim.path)
                    path_parts = path_str.strip('/').split('/')
                    
                    # 机器人链接通常在第二层（/robot_name/link_name）
                    is_potential_link = (
                        len(path_parts) == 2 and  # 第二层
                        prim.type_name in ['Prim', 'Xform'] and  # 合适的类型
                        'visuals' not in path_str and
                        'collisions' not in path_str and
                        not prim.name.lower().startswith('joint')  # 不是关节
                    )
                    
                    if is_potential_link:
                        potential_links.append(prim)
            
            links = potential_links
        
        return links
    
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
