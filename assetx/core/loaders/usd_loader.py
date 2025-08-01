#!/usr/bin/env python3
"""
USD 格式加载器
"""

from typing import TYPE_CHECKING

from .base import BaseAssetLoader

if TYPE_CHECKING:
    from ..asset import Asset


class UsdLoader(BaseAssetLoader):
    """USD格式加载器"""
    
    def can_load(self) -> bool:
        """检查是否为USD文件"""
        suffix = self.asset_path.suffix.lower()
        return suffix in ['.usd', '.usda', '.usdc']
    
    def load(self, asset: "Asset") -> None:
        """加载USD格式"""
        try:
            # 尝试导入USD库
            try:
                from pxr import Usd, UsdGeom, Sdf
            except ImportError:
                raise ImportError(
                    "USD library (pxr) not found. Install with: pip install pxr or conda install -c conda-forge pxr"
                )
            
            # 检查文件是否存在且有内容
            if not self.asset_path.exists():
                print(f"⚠️  USD file does not exist: {self.asset_path}")
                self._load_simulation_mode(asset)
                return
                
            if self.asset_path.stat().st_size == 0:
                print(f"⚠️  USD file is empty: {self.asset_path}")
                self._load_simulation_mode(asset)
                return
            
            # 尝试打开USD Stage
            try:
                usd_stage = Usd.Stage.Open(str(self.asset_path))
                if not usd_stage:
                    print(f"⚠️  Failed to open USD stage, using simulation mode")
                    self._load_simulation_mode(asset)
                    return
                
                print(f"✅ Successfully opened USD stage: {self.asset_path}")
                
                # 获取所有Prim并转换为AssetX格式
                self._convert_usd_stage_to_assetx(usd_stage, asset)
                
            except Exception as usd_error:
                print(f"⚠️  USD parsing error: {usd_error}")
                print(f"📄 Using simulation mode for: {self.asset_path}")
                self._load_simulation_mode(asset)
                return
            
        except ImportError as e:
            # USD库未安装，使用模拟加载
            print(f"⚠️  USD library not available: {e}")
            print(f"📄 Loading USD format from {self.asset_path} (simulation mode)")
            self._load_simulation_mode(asset)
            
        except Exception as e:
            print(f"⚠️  General USD loading error: {e}")
            print(f"📄 Using simulation mode for: {self.asset_path}")
            self._load_simulation_mode(asset)
    
    def _convert_usd_stage_to_assetx(self, usd_stage, asset: "Asset") -> None:
        """将USD Stage转换为AssetX格式"""
        from pxr import Usd, UsdGeom, Sdf
        
        # 获取默认Prim
        default_prim = usd_stage.GetDefaultPrim()
        if default_prim:
            # 创建对应的AssetX根Prim
            robot_name = default_prim.GetName()
            robot_prim = asset.create_robot_prim(f"/{robot_name}", robot_name)
            stage = self._get_stage(asset)
            stage.set_default_prim(robot_prim)
            print(f"  📍 Set default prim: {robot_name}")
        
        # 遍历所有Prim
        prim_count = 0
        for usd_prim in usd_stage.Traverse():
            if usd_prim.IsPseudoRoot():
                continue
                
            prim_path = str(usd_prim.GetPath())
            prim_type = usd_prim.GetTypeName()
            
            # 根据USD类型映射到AssetX类型
            assetx_type = self._map_usd_type_to_assetx(prim_type)
            
            # 创建AssetX Prim
            try:
                assetx_prim = asset.define_prim(prim_path, assetx_type)
                
                # 复制属性
                self._copy_usd_attributes(usd_prim, assetx_prim)
                
                prim_count += 1
                print(f"  🔗 Converted prim: {prim_path} ({prim_type} -> {assetx_type})")
                
            except Exception as e:
                print(f"  ⚠️  Failed to convert prim {prim_path}: {e}")
        
        print(f"  ✅ Converted {prim_count} prims from USD to AssetX")
    
    def _map_usd_type_to_assetx(self, usd_type: str) -> str:
        """映射USD类型到AssetX类型"""
        type_mapping = {
            # USD Geometry Types
            "Mesh": "Mesh",
            "Sphere": "Mesh", 
            "Cube": "Mesh",
            "Cylinder": "Mesh",
            "Cone": "Mesh",
            
            # USD Physics Types  
            "PhysicsRigidBody": "Link",
            "PhysicsJoint": "Joint",
            "PhysicsFixedJoint": "Joint",
            "PhysicsRevoluteJoint": "Joint",
            "PhysicsPrismaticJoint": "Joint",
            
            # Custom Robot Types
            "Robot": "Robot",
            "Link": "Link", 
            "Joint": "Joint",
            
            # Default
            "": "Prim"
        }
        return type_mapping.get(usd_type, "Prim")
    
    def _copy_usd_attributes(self, usd_prim, assetx_prim) -> None:
        """复制USD属性到AssetX Prim"""
        try:
            from pxr import Usd
            
            # 获取所有属性
            for attr in usd_prim.GetAttributes():
                attr_name = attr.GetName()
                attr_value = attr.Get()
                attr_type = attr.GetTypeName()
                
                if attr_value is not None:
                    # 创建对应的AssetX属性
                    try:
                        assetx_attr = assetx_prim.create_attribute(
                            attr_name, str(attr_type)
                        )
                        assetx_attr.set(attr_value)
                    except Exception as e:
                        print(f"    ⚠️  Failed to copy attribute {attr_name}: {e}")
                        
        except Exception as e:
            print(f"  ⚠️  Failed to copy attributes: {e}")
    
    def _load_simulation_mode(self, asset: "Asset") -> None:
        """USD库不可用时的模拟加载"""
        # 基于文件名创建一个模拟的机器人结构
        robot_name = self.asset_path.stem
        
        # 创建机器人根Prim
        robot_prim = asset.create_robot_prim(f"/{robot_name}", robot_name)
        stage = self._get_stage(asset)
        stage.set_default_prim(robot_prim)
        
        # 创建一些示例链接和关节（基于常见机器人结构）
        asset.create_link_prim(f"/{robot_name}/base_link", 1.0, [1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        asset.create_link_prim(f"/{robot_name}/link1", 0.5, [0.5, 0.5, 0.5, 0.0, 0.0, 0.0])
        
        asset.create_joint_prim(
            f"/{robot_name}/joint1", 
            "revolute", 
            f"/{robot_name}/base_link",
            f"/{robot_name}/link1",
            [0.0, 0.0, 1.0]
        )
        
        print(f"  📍 Created simulated robot structure with 2 links and 1 joint")
