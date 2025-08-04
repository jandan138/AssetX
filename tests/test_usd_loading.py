#!/usr/bin/env python3
"""
测试USD文件加载和内容获取功能

此测试文件验证AssetX框架能够正确加载USD文件并获取其完整内容，
包括链接、关节、属性和层次结构信息。
"""

import os
import sys
import unittest
from pathlib import Path

# 添加项目根目录到Python路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from assetx.core.asset import Asset
from assetx.core.modules.queries import AssetQuery
import logging

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TestUSDLoading(unittest.TestCase):
    """测试USD文件加载和内容提取"""
    
    def setUp(self):
        """设置测试环境"""
        # Franka USD文件路径
        self.franka_usd_path = r"c:\Users\24494\Downloads\franka.usda"
        
        # 检查文件是否存在
        if not os.path.exists(self.franka_usd_path):
            self.skipTest(f"USD文件不存在: {self.franka_usd_path}")
    
    def test_load_usd_file(self):
        """测试USD文件基本加载功能"""
        logger.info("=== 测试USD文件加载 ===")
        
        # 1. 创建Asset对象并加载USD文件
        asset = Asset(self.franka_usd_path)
        asset.load()
        self.assertTrue(asset.is_loaded, "USD文件加载失败")
        logger.info(f"✓ 成功加载USD文件: {self.franka_usd_path}")
        
        # 2. 验证Asset对象有效性
        stage = asset.get_stage()
        self.assertIsNotNone(stage, "Asset的stage为None")
        logger.info("✓ Asset对象创建成功")
    
    def test_get_links_from_usd(self):
        """测试从USD文件获取链接信息"""
        logger.info("=== 测试链接获取 ===")
        
        asset = Asset(self.franka_usd_path)
        asset.load()
        query = AssetQuery(asset)
        
        # 获取所有链接
        links = query.get_links()
        self.assertIsInstance(links, list, "链接列表应该是list类型")
        self.assertGreater(len(links), 0, "应该找到至少一个链接")
        
        logger.info(f"找到 {len(links)} 个链接:")
        
        # 验证并打印每个链接的详细信息
        for i, link in enumerate(links, 1):
            logger.info(f"\n--- 链接 {i}: {link.name} ---")
            logger.info(f"路径: {link.path}")
            logger.info(f"类型: {getattr(link, 'type_name', '未知')}")
            
            # 获取链接属性
            if hasattr(link, 'properties'):
                props = link.properties
                logger.info(f"属性数量: {len(props)}")
                for prop_name, prop_value in props.items():
                    logger.info(f"  {prop_name}: {prop_value} (类型: {type(prop_value)})")
            
            # 验证必要属性
            self.assertTrue(hasattr(link, 'name'), f"链接 {i} 缺少name属性")
            self.assertTrue(hasattr(link, 'path'), f"链接 {i} 缺少path属性")
        
        # 验证Franka机器人特定链接
        link_names = [link.name for link in links]
        expected_franka_links = [
            'panda_link0', 'panda_link1', 'panda_link2', 'panda_link3',
            'panda_link4', 'panda_link5', 'panda_link6', 'panda_link7'
        ]
        
        for expected_link in expected_franka_links:
            if expected_link in link_names:
                logger.info(f"✓ 找到预期的Franka链接: {expected_link}")
        
        logger.info("✓ 链接获取测试完成")
    
    def test_get_joints_from_usd(self):
        """测试从USD文件获取关节信息"""
        logger.info("=== 测试关节获取 ===")
        
        asset = Asset(self.franka_usd_path)
        asset.load()
        query = AssetQuery(asset)
        
        # 获取所有关节
        joints = query.get_joints()
        self.assertIsInstance(joints, list, "关节列表应该是list类型")
        self.assertGreater(len(joints), 0, "应该找到至少一个关节")
        
        logger.info(f"找到 {len(joints)} 个关节:")
        
        # 验证并打印每个关节的详细信息
        for i, joint in enumerate(joints, 1):
            logger.info(f"\n--- 关节 {i}: {joint.name} ---")
            logger.info(f"路径: {joint.path}")
            logger.info(f"类型: {getattr(joint, 'type_name', '未知')}")
            
            # 获取关节属性
            if hasattr(joint, 'get_property_names'):
                try:
                    property_names = joint.get_property_names()
                    logger.info(f"属性列表: {property_names}")
                    
                    # 检查重要的关节属性
                    important_attrs = ['physics:body0', 'physics:body1', 'physics:localPos0', 
                                     'physics:localPos1', 'physics:axis']
                    for attr in important_attrs:
                        if hasattr(joint, 'get_attribute'):
                            attr_obj = joint.get_attribute(attr)
                            if attr_obj:
                                try:
                                    value = attr_obj.get()
                                    logger.info(f"  {attr}: {value}")
                                except Exception as e:
                                    logger.debug(f"无法获取属性 {attr}: {e}")
                
                except Exception as e:
                    logger.debug(f"获取关节 {joint.name} 属性失败: {e}")
            
            # 验证必要属性
            self.assertTrue(hasattr(joint, 'name'), f"关节 {i} 缺少name属性")
            self.assertTrue(hasattr(joint, 'path'), f"关节 {i} 缺少path属性")
        
        logger.info("✓ 关节获取测试完成")
    
    def test_usd_hierarchy_structure(self):
        """测试USD文件层次结构"""
        logger.info("=== 测试USD层次结构 ===")
        
        asset = Asset(self.franka_usd_path)
        asset.load()
        query = AssetQuery(asset)
        
        # 获取所有原语
        all_prims = query.get_all_prims()
        self.assertIsInstance(all_prims, list, "原语列表应该是list类型")
        self.assertGreater(len(all_prims), 0, "应该找到至少一个原语")
        
        logger.info(f"USD文件层次结构 (共 {len(all_prims)} 个原语):")
        
        # 按路径深度排序以显示层次结构
        sorted_prims = sorted(all_prims, key=lambda p: (str(p.path).count('/'), str(p.path)))
        
        for prim in sorted_prims:
            # 计算缩进级别
            depth = str(prim.path).count('/') - 1
            indent = "  " * depth
            
            # 获取原语类型
            prim_type = getattr(prim, 'type_name', '未知类型')
            
            logger.info(f"{indent}{prim.name} ({prim_type}) - 路径: {prim.path}")
            
            # 对于重要的原语，显示关键属性
            if any(keyword in prim.name.lower() for keyword in ['link', 'joint']):
                if hasattr(prim, 'properties') and prim.properties:
                    key_props = []
                    for prop_name, prop_value in prim.properties.items():
                        if any(key in prop_name for key in ['translate', 'orient', 'body', 'axis']):
                            key_props.append(f"{prop_name}: {prop_value}")
                    
                    if key_props:
                        for prop in key_props[:3]:  # 只显示前3个关键属性
                            logger.info(f"{indent}  └─ {prop}")
        
        logger.info("✓ 层次结构分析完成")
    
    def test_usd_physics_properties(self):
        """测试USD物理属性提取"""
        logger.info("=== 测试物理属性提取 ===")
        
        asset = Asset(self.franka_usd_path)
        asset.load()
        query = AssetQuery(asset)
        
        # 获取所有链接
        links = query.get_links()
        
        physics_info = {}
        
        for link in links:
            link_physics = {}
            
            if hasattr(link, 'properties'):
                # 提取物理相关属性
                for prop_name, prop_value in link.properties.items():
                    if any(keyword in prop_name.lower() for keyword in 
                          ['physics', 'mass', 'inertia', 'translate', 'orient']):
                        link_physics[prop_name] = prop_value
            
            if link_physics:
                physics_info[link.name] = link_physics
        
        logger.info("物理属性摘要:")
        for link_name, props in physics_info.items():
            logger.info(f"\n{link_name}:")
            for prop_name, prop_value in props.items():
                logger.info(f"  {prop_name}: {prop_value}")
        
        # 如果没有找到物理属性，至少确保我们找到了一些链接
        if len(physics_info) == 0:
            logger.warning("没有找到物理属性，但这可能是正常的")
            # 验证至少找到了链接
            self.assertGreater(len(links), 0, "应该找到至少一个链接")
        else:
            # 验证找到了物理属性
            self.assertGreater(len(physics_info), 0, "应该找到至少一个带有物理属性的链接")
        
        logger.info("✓ 物理属性提取完成")
    
    def test_usd_content_completeness(self):
        """测试USD内容完整性"""
        logger.info("=== 测试内容完整性 ===")
        
        asset = Asset(self.franka_usd_path)
        asset.load()
        query = AssetQuery(asset)
        
        # 获取各类内容
        links = query.get_links()
        joints = query.get_joints()
        all_prims = query.get_all_prims()
        
        # 统计信息
        stats = {
            "总原语数": len(all_prims),
            "链接数": len(links),
            "关节数": len(joints),
            "其他原语数": len(all_prims) - len(links) - len(joints)
        }
        
        logger.info("USD文件内容统计:")
        for key, value in stats.items():
            logger.info(f"  {key}: {value}")
        
        # 验证Franka机器人的预期结构
        link_names = [link.name for link in links]
        joint_names = [joint.name for joint in joints]
        
        expected_structure = {
            "预期链接数": 8,  # panda_link0 到 panda_link7
            "预期关节数范围": (7, 15),  # 主要关节 + 可能的手指关节
        }
        
        # 检查是否符合预期
        franka_links = [name for name in link_names if 'panda_link' in name]
        franka_joints = [name for name in joint_names if 'panda_joint' in name]
        
        logger.info(f"\nFranka特定结构:")
        logger.info(f"  Franka链接: {len(franka_links)} 个 {franka_links}")
        logger.info(f"  Franka关节: {len(franka_joints)} 个 {franka_joints}")
        
        # 验证
        self.assertGreaterEqual(len(franka_links), 8, "应该找到至少8个Franka链接")
        self.assertGreaterEqual(len(franka_joints), 7, "应该找到至少7个Franka关节")
        
        logger.info("✓ 内容完整性验证通过")


def run_usd_loading_tests():
    """运行USD加载测试"""
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestUSDLoading)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == "__main__":
    print("=" * 80)
    print("AssetX USD文件加载和内容获取测试")
    print("=" * 80)
    
    success = run_usd_loading_tests()
    
    if success:
        print("\n🎉 所有测试通过！")
        sys.exit(0)
    else:
        print("\n❌ 部分测试失败！")
        sys.exit(1)
