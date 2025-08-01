#!/usr/bin/env python3
"""
AssetX Core 目录重构分析

当前文件数量: 21个Python文件
问题: 文件过多，缺乏清晰的分组，有重复和冗余文件

重构方案:
"""

import os
from pathlib import Path

def analyze_core_structure():
    print("🔍 AssetX Core 目录分析")
    print("=" * 60)
    
    core_files = {
        # 核心基础类 (保留在core根目录)
        "core_basics": [
            "asset.py",         # 主Asset类
            "enums.py",         # 枚举定义
            "__init__.py",      # 包初始化
        ],
        
        # USD风格基础组件 (新建 primitives/ 目录)
        "primitives": [
            "prim.py",          # AssetPrim类
            "stage.py",         # AssetStage类  
            "sdf_path.py",      # SdfPath类
            "property.py",      # AssetProperty类
        ],
        
        # 功能模块 (新建 modules/ 目录)
        "modules": [
            "queries.py",       # 查询功能
            "schemas.py",       # Schema应用
            "meta.py",          # 元数据操作
        ],
        
        # 转换和验证 (新建 processing/ 目录)
        "processing": [
            "converter.py",           # 格式转换器
            "usd_to_urdf_converter.py", # USD转URDF
            "validator.py",           # 验证器
        ],
        
        # 加载器 (保持现有 loaders/ 目录)
        "loaders": [
            "loaders/",         # 加载器目录 (已存在)
        ],
        
        # 冗余文件 (需要清理)
        "redundant": [
            "asset_new.py",     # 重构时的临时文件
            "asset_original.py", # 原始备份文件
        ]
    }
    
    print("📊 文件分类统计:")
    total_files = 0
    for category, files in core_files.items():
        print(f"   {category:15}: {len(files):2}个文件")
        total_files += len(files)
    
    print(f"\n📁 总文件数: {total_files} (不含loaders目录内容)")
    
    return core_files

def proposed_structure():
    print(f"\n🏗️ 建议的新目录结构:")
    print("=" * 60)
    
    structure = """
assetx/core/
├── __init__.py                 # 包初始化和主要导出
├── asset.py                   # 主Asset类
├── enums.py                   # 枚举定义
├── primitives/                # USD风格基础组件
│   ├── __init__.py
│   ├── prim.py               # AssetPrim类
│   ├── stage.py              # AssetStage类  
│   ├── sdf_path.py           # SdfPath类
│   └── property.py           # AssetProperty类
├── modules/                   # 功能模块
│   ├── __init__.py
│   ├── queries.py            # 查询功能
│   ├── schemas.py            # Schema应用
│   └── meta.py               # 元数据操作
├── processing/                # 转换和验证
│   ├── __init__.py
│   ├── converter.py          # 格式转换器
│   ├── usd_to_urdf_converter.py # USD转URDF
│   └── validator.py          # 验证器
└── loaders/                   # 加载器 (保持不变)
    ├── __init__.py
    ├── base.py
    ├── urdf_loader.py
    ├── usd_loader.py
    ├── mjcf_loader.py
    └── genesis_loader.py
"""
    
    print(structure)

if __name__ == "__main__":
    core_files = analyze_core_structure()
    proposed_structure()
    
    print(f"\n✅ 重构收益:")
    print(f"   📉 文件数量: 21 → 16 (减少5个)")
    print(f"   📁 目录层次: 更清晰的功能分组")
    print(f"   🔧 可维护性: 更好的模块化")
    print(f"   🚀 可扩展性: 便于添加新功能")
