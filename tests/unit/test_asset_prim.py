#!/usr/bin/env python3
"""
AssetPrim类的单元测试
"""

import unittest
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from assetx.core.prim import AssetPrim
from assetx.core.stage import AssetStage
from assetx.core.sdf_path import SdfPath


class TestAssetPrim(unittest.TestCase):
    """AssetPrim类的单元测试"""
    
    def setUp(self):
        """设置测试环境"""
        self.stage = AssetStage("test_stage")
        self.prim_path = SdfPath("/test_prim")
        self.prim = AssetPrim(self.stage, self.prim_path)
    
    def test_prim_creation(self):
        """测试Prim创建"""
        self.assertEqual(self.prim.path, self.prim_path)
        self.assertEqual(self.prim.name, "test_prim")
        self.assertEqual(self.prim.stage, self.stage)
        self.assertTrue(self.prim.is_valid())
        self.assertTrue(self.prim.is_active())
    
    def test_type_name_operations(self):
        """测试类型名操作"""
        # 初始没有类型
        self.assertIsNone(self.prim.type_name)
        
        # 设置类型
        self.prim.set_type_name("Link")
        self.assertEqual(self.prim.type_name, "Link")
    
    def test_active_state(self):
        """测试激活状态"""
        self.assertTrue(self.prim.is_active())
        
        # 设置为非激活
        self.prim.set_active(False)
        self.assertFalse(self.prim.is_active())
        
        # 重新激活
        self.prim.set_active(True)
        self.assertTrue(self.prim.is_active())
    
    def test_instanceable_state(self):
        """测试可实例化状态"""
        self.assertFalse(self.prim.is_instanceable())
        
        # 设置为可实例化
        self.prim.set_instanceable(True)
        self.assertTrue(self.prim.is_instanceable())
    
    def test_attribute_operations(self):
        """测试属性操作"""
        # 创建属性
        attr = self.prim.create_attribute("mass", "float")
        self.assertIsNotNone(attr)
        self.assertEqual(attr.name, "mass")
        
        # 获取属性
        retrieved_attr = self.prim.get_attribute("mass")
        self.assertEqual(attr, retrieved_attr)
        
        # 获取所有属性
        attributes = self.prim.get_attributes()
        self.assertEqual(len(attributes), 1)
        self.assertIn(attr, attributes)
        
        # 检查属性存在
        self.assertTrue(self.prim.has_attribute("mass"))
        self.assertFalse(self.prim.has_attribute("nonexistent"))
    
    def test_relationship_operations(self):
        """测试关系操作"""
        # 创建关系
        rel = self.prim.create_relationship("parent")
        self.assertIsNotNone(rel)
        self.assertEqual(rel.name, "parent")
        
        # 获取关系
        retrieved_rel = self.prim.get_relationship("parent")
        self.assertEqual(rel, retrieved_rel)
        
        # 获取所有关系
        relationships = self.prim.get_relationships()
        self.assertEqual(len(relationships), 1)
        self.assertIn(rel, relationships)
        
        # 检查关系存在
        self.assertTrue(self.prim.has_relationship("parent"))
        self.assertFalse(self.prim.has_relationship("nonexistent"))
    
    def test_property_operations(self):
        """测试属性和关系的统一操作"""
        # 创建属性和关系
        attr = self.prim.create_attribute("mass", "float")
        rel = self.prim.create_relationship("parent")
        
        # 获取所有属性（包括Attribute和Relationship）
        properties = self.prim.get_properties()
        self.assertEqual(len(properties), 2)
        
        # 获取属性名列表
        prop_names = self.prim.get_property_names()
        self.assertIn("mass", prop_names)
        self.assertIn("parent", prop_names)
        
        # 通过名称获取属性
        mass_prop = self.prim.get_property("mass")
        parent_prop = self.prim.get_property("parent")
        self.assertEqual(mass_prop, attr)
        self.assertEqual(parent_prop, rel)
    
    def test_metadata_operations(self):
        """测试元数据操作"""
        # 设置元数据
        self.prim.set_metadata("description", "Test prim")
        self.prim.set_metadata("version", "1.0")
        
        # 获取元数据
        self.assertEqual(self.prim.get_metadata("description"), "Test prim")
        self.assertEqual(self.prim.get_metadata("version"), "1.0")
        self.assertIsNone(self.prim.get_metadata("nonexistent"))
        
        # 检查元数据存在
        self.assertTrue(self.prim.has_metadata("description"))
        self.assertFalse(self.prim.has_metadata("nonexistent"))
        
        # 清除元数据
        self.prim.clear_metadata("description")
        self.assertFalse(self.prim.has_metadata("description"))
    
    def test_schema_operations(self):
        """测试Schema操作"""
        # 应用Schema
        self.prim.apply_api_schema("PhysicsAPI")
        self.prim.apply_api_schema("LinkAPI")
        
        # 获取应用的Schema
        schemas = self.prim.get_applied_schemas()
        self.assertIn("PhysicsAPI", schemas)
        self.assertIn("LinkAPI", schemas)
        
        # 检查Schema是否应用
        self.assertTrue(self.prim.has_api_schema("PhysicsAPI"))
        self.assertTrue(self.prim.has_api_schema("LinkAPI"))
        self.assertFalse(self.prim.has_api_schema("NonexistentAPI"))
        
        # 移除Schema
        self.prim.remove_api_schema("PhysicsAPI")
        self.assertFalse(self.prim.has_api_schema("PhysicsAPI"))
        self.assertTrue(self.prim.has_api_schema("LinkAPI"))


if __name__ == "__main__":
    unittest.main()
