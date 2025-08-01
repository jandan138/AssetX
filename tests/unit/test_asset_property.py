#!/usr/bin/env python3
"""
AssetProperty系统的单元测试
"""

import unittest
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from assetx.core.primitives.property import (
    AssetAttribute, AssetRelationship
)
from assetx.core.primitives.sdf_path import SdfPath
from assetx.core.primitives.stage import AssetStage
from assetx.core.enums import VariabilityType


class TestAssetProperty(unittest.TestCase):
    """AssetProperty系统的单元测试"""
    
    def setUp(self):
        """设置测试环境"""
        self.stage = AssetStage("test_stage")
        self.test_path = SdfPath("/test/prop")
    
    def test_attribute_creation(self):
        """测试Attribute创建"""
        attr = AssetAttribute(self.stage, self.test_path, "float")
        
        self.assertEqual(attr.name, "prop")
        self.assertEqual(attr.type_name, "float")
        self.assertEqual(attr.path, self.test_path)
        self.assertEqual(attr.variability, VariabilityType.VARYING)
    
    def test_attribute_value_operations(self):
        """测试Attribute值操作"""
        attr = AssetAttribute(self.stage, self.test_path, "float")
        
        # 设置和获取值
        attr.set(5.0)
        self.assertEqual(attr.get(), 5.0)
        
        # 测试时间采样值
        attr.set(10.0, 1.0)  # 在时间1.0设置值
        self.assertEqual(attr.get(1.0), 10.0)
    
    def test_attribute_time_samples(self):
        """测试Attribute时间采样"""
        attr = AssetAttribute(self.stage, self.test_path, "vector3")
        
        # 设置时间采样
        attr.set([0, 0, 0], 0.0)
        attr.set([1, 1, 1], 1.0)
        attr.set([2, 2, 2], 2.0)
        
        # 获取时间采样
        self.assertEqual(attr.get(0.0), [0, 0, 0])
        self.assertEqual(attr.get(1.0), [1, 1, 1])
        self.assertEqual(attr.get(2.0), [2, 2, 2])
        
        # 获取所有时间采样
        samples = attr.get_time_samples()
        self.assertEqual(len(samples), 3)
        self.assertIn(0.0, samples)
        self.assertIn(1.0, samples)
        self.assertIn(2.0, samples)
    
    def test_attribute_metadata(self):
        """测试Attribute元数据"""
        attr = AssetAttribute(self.stage, self.test_path, "float")
        
        # 设置元数据
        attr.set_metadata("description", "Object mass")
        attr.set_metadata("units", "kg")
        
        # 获取元数据
        self.assertEqual(attr.get_metadata("description"), "Object mass")
        self.assertEqual(attr.get_metadata("units"), "kg")
        self.assertIsNone(attr.get_metadata("nonexistent"))
        
        # 检查元数据存在
        self.assertTrue(attr.has_metadata("description"))
        self.assertFalse(attr.has_metadata("nonexistent"))
    
    def test_relationship_creation(self):
        """测试Relationship创建"""
        rel = AssetRelationship(self.stage, self.test_path)
        
        self.assertEqual(rel.name, "prop")
        self.assertEqual(rel.path, self.test_path)
        self.assertEqual(rel.variability, VariabilityType.VARYING)  # 基类默认是VARYING
    
    def test_relationship_targets(self):
        """测试Relationship目标操作"""
        rel = AssetRelationship(self.stage, self.test_path)
        
        target1 = SdfPath("/robot/link1")
        target2 = SdfPath("/robot/link2")
        
        # 添加目标
        rel.add_target(target1)
        rel.add_target(target2)
        
        targets = rel.get_targets()
        self.assertEqual(len(targets), 2)
        self.assertIn(target1, targets)
        self.assertIn(target2, targets)
        
        # 移除目标
        rel.remove_target(target1)
        targets = rel.get_targets()
        self.assertEqual(len(targets), 1)
        self.assertIn(target2, targets)
        self.assertNotIn(target1, targets)
        
        # 设置所有目标为空（相当于清除）
        rel.set_targets([])
        targets = rel.get_targets()
        self.assertEqual(len(targets), 0)


if __name__ == "__main__":
    unittest.main()
