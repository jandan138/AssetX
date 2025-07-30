#!/usr/bin/env python3
"""
测试运行器 - 运行所有测试
"""

import unittest
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def run_unit_tests():
    """运行单元测试"""
    print("=== 运行单元测试 ===")
    loader = unittest.TestLoader()
    suite = loader.discover('unit', pattern='test_*.py')
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


def run_integration_tests():
    """运行集成测试"""
    print("\n=== 运行集成测试 ===")
    loader = unittest.TestLoader()
    suite = loader.discover('integration', pattern='test_*.py')
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


def run_all_tests():
    """运行所有测试"""
    print("=== AssetX 测试套件 ===")
    
    # 运行单元测试
    unit_success = run_unit_tests()
    
    # 运行集成测试
    integration_success = run_integration_tests()
    
    # 总结
    print("\n=== 测试结果总结 ===")
    print(f"单元测试: {'✅ 通过' if unit_success else '❌ 失败'}")
    print(f"集成测试: {'✅ 通过' if integration_success else '❌ 失败'}")
    
    if unit_success and integration_success:
        print("🎉 所有测试通过!")
        return True
    else:
        print("⚠️ 部分测试失败")
        return False


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="AssetX 测试运行器")
    parser.add_argument("--unit", action="store_true", help="只运行单元测试")
    parser.add_argument("--integration", action="store_true", help="只运行集成测试")
    
    args = parser.parse_args()
    
    if args.unit:
        success = run_unit_tests()
    elif args.integration:
        success = run_integration_tests()
    else:
        success = run_all_tests()
    
    sys.exit(0 if success else 1)
