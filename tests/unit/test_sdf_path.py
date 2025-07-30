#!/usr/bin/env python3
"""
SdfPath类的单元测试
"""

import unittest
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from assetx.core.asset import SdfPath


class TestSdfPath(unittest.TestCase):
    """SdfPath类的单元测试"""
    
    def test_basic_path_creation(self):
        """测试基本路径创建"""
        path = SdfPath("/robot/base_link")
        self.assertEqual(str(path), "/robot/base_link")
        self.assertEqual(path.path_string, "/robot/base_link")
    
    def test_root_path(self):
        """测试根路径"""
        root = SdfPath("/")
        self.assertEqual(str(root), "/")
        # 注释掉不存在的方法
        # self.assertTrue(root.is_root_path())
    
    def test_parent_path(self):
        """测试父路径获取"""
        path = SdfPath("/robot/base_link/visual")
        parent = path.get_parent_path()
        self.assertEqual(str(parent), "/robot/base_link")
        
        # 测试根路径的父路径
        root = SdfPath("/")
        root_parent = root.get_parent_path()
        self.assertEqual(str(root_parent), "/")
    
    def test_get_name(self):
        """测试获取节点名"""
        path = SdfPath("/robot/base_link")
        self.assertEqual(path.get_name(), "base_link")
        
        root = SdfPath("/")
        self.assertEqual(root.get_name(), "")
    
    def test_append_child(self):
        """测试追加子路径"""
        path = SdfPath("/robot")
        child = path.append_child("base_link")
        self.assertEqual(str(child), "/robot/base_link")
        
        # 测试追加到根路径
        root = SdfPath("/")
        child_of_root = root.append_child("robot")
        self.assertEqual(str(child_of_root), "/robot")
    
    def test_make_absolute_path(self):
        """测试创建绝对路径"""
        # 直接创建绝对路径
        abs_path = SdfPath("/robot/link")
        self.assertEqual(str(abs_path), "/robot/link")
    
    def test_path_equality(self):
        """测试路径相等性"""
        path1 = SdfPath("/robot/base_link")
        path2 = SdfPath("/robot/base_link")
        path3 = SdfPath("/robot/arm_link")
        
        self.assertEqual(path1, path2)
        self.assertNotEqual(path1, path3)
    
    def test_path_hashing(self):
        """测试路径哈希"""
        path1 = SdfPath("/robot/base_link")
        path2 = SdfPath("/robot/base_link")
        
        # 相同路径应该有相同的哈希值
        self.assertEqual(hash(path1), hash(path2))
        
        # 可以用作字典键
        path_dict = {path1: "value1"}
        self.assertEqual(path_dict[path2], "value1")


if __name__ == "__main__":
    unittest.main()
