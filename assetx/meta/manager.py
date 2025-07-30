"""
Asset metadata management
"""

import logging
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Union

import yaml

logger = logging.getLogger(__name__)


class MetaManager:
    """元数据管理器"""

    def __init__(self, asset_dir: Union[str, Path]):
        self.asset_dir = Path(asset_dir)
        self.meta_file = self.asset_dir / "meta.yaml"
        self.schema_version = "1.0"

    def create_meta_template(self) -> Dict:
        """创建元数据模板"""
        return {
            "schema_version": self.schema_version,
            "created_at": datetime.now().isoformat(),
            "updated_at": datetime.now().isoformat(),
            "assets": {},
            "collections": {},
            "global_settings": {
                "default_unit": "m",
                "default_mass_unit": "kg",
                "coordinate_system": "right_handed",
            },
        }

    def load_meta(self, asset_name: Optional[str] = None) -> Dict:
        """加载元数据"""
        if not self.meta_file.exists():
            logger.info(f"Meta file not found, creating new one: {self.meta_file}")
            meta_data = self.create_meta_template()
            self.save_meta(meta_data)
            return meta_data

        try:
            with open(self.meta_file, "r", encoding="utf-8") as f:
                meta_data = yaml.safe_load(f) or {}

            # 如果请求特定资产的元数据
            if asset_name:
                return meta_data.get("assets", {}).get(asset_name, {})

            return meta_data

        except Exception as e:
            logger.error(f"Failed to load meta file: {e}")
            return {}

    def save_meta(self, meta_data: Dict) -> None:
        """保存元数据"""
        try:
            # 确保目录存在
            self.asset_dir.mkdir(parents=True, exist_ok=True)

            # 更新时间戳
            meta_data["updated_at"] = datetime.now().isoformat()

            with open(self.meta_file, "w", encoding="utf-8") as f:
                yaml.dump(meta_data, f, default_flow_style=False, allow_unicode=True)

            logger.info(f"Meta data saved to: {self.meta_file}")

        except Exception as e:
            logger.error(f"Failed to save meta file: {e}")
            raise

    def update_asset_meta(self, asset_name: str, asset_meta: Dict) -> None:
        """更新单个资产的元数据"""
        meta_data = self.load_meta()

        if "assets" not in meta_data:
            meta_data["assets"] = {}

        # 合并现有元数据和新元数据
        existing_meta = meta_data["assets"].get(asset_name, {})
        existing_meta.update(asset_meta)
        existing_meta["updated_at"] = datetime.now().isoformat()

        meta_data["assets"][asset_name] = existing_meta
        self.save_meta(meta_data)

    def create_asset_meta(
        self,
        asset_name: str,
        original_format: str,
        semantic_category: str,
        description: str = "",
        tags: Optional[List[str]] = None,
        custom_fields: Optional[Dict] = None,
    ) -> Dict:
        """创建资产元数据"""

        asset_meta = {
            "name": asset_name,
            "description": description,
            "original_format": original_format,
            "semantic_category": semantic_category,
            "tags": tags or [],
            "created_at": datetime.now().isoformat(),
            "updated_at": datetime.now().isoformat(),
            "conversions": {},
            "validations": {},
            "mesh_info": {},
            "physics_validated": False,
            "custom_fields": custom_fields or {},
        }

        return asset_meta

    def add_conversion_record(
        self,
        asset_name: str,
        target_format: str,
        output_path: str,
        conversion_time: float,
        success: bool = True,
        notes: str = "",
    ) -> None:
        """添加转换记录"""

        meta_data = self.load_meta()
        if asset_name not in meta_data.get("assets", {}):
            logger.warning(f"Asset {asset_name} not found in meta data")
            return

        conversion_record = {
            "output_path": output_path,
            "conversion_time": conversion_time,
            "success": success,
            "notes": notes,
            "timestamp": datetime.now().isoformat(),
        }

        asset_meta = meta_data["assets"][asset_name]
        if "conversions" not in asset_meta:
            asset_meta["conversions"] = {}

        asset_meta["conversions"][target_format] = conversion_record
        self.save_meta(meta_data)

    def add_validation_record(
        self,
        asset_name: str,
        validation_type: str,
        result: Dict,
        compared_with: Optional[str] = None,
    ) -> None:
        """添加验证记录"""

        meta_data = self.load_meta()
        if asset_name not in meta_data.get("assets", {}):
            logger.warning(f"Asset {asset_name} not found in meta data")
            return

        validation_record = {
            "result": result,
            "compared_with": compared_with,
            "timestamp": datetime.now().isoformat(),
        }

        asset_meta = meta_data["assets"][asset_name]
        if "validations" not in asset_meta:
            asset_meta["validations"] = {}

        asset_meta["validations"][validation_type] = validation_record

        # 更新物理验证状态
        if validation_type == "physics" and result.get("is_valid"):
            asset_meta["physics_validated"] = True

        self.save_meta(meta_data)

    def get_asset_history(self, asset_name: str) -> Dict:
        """获取资产历史记录"""
        meta_data = self.load_meta()
        asset_meta = meta_data.get("assets", {}).get(asset_name, {})

        return {
            "conversions": asset_meta.get("conversions", {}),
            "validations": asset_meta.get("validations", {}),
            "created_at": asset_meta.get("created_at"),
            "updated_at": asset_meta.get("updated_at"),
        }

    def list_assets(
        self,
        category: Optional[str] = None,
        tag: Optional[str] = None,
        format_filter: Optional[str] = None,
    ) -> List[Dict]:
        """列出资产，支持过滤"""
        meta_data = self.load_meta()
        assets = meta_data.get("assets", {})

        filtered_assets = []

        for asset_name, asset_meta in assets.items():
            # 应用过滤条件
            if category and asset_meta.get("semantic_category") != category:
                continue

            if tag and tag not in asset_meta.get("tags", []):
                continue

            if format_filter and asset_meta.get("original_format") != format_filter:
                continue

            filtered_assets.append(
                {
                    "name": asset_name,
                    "category": asset_meta.get("semantic_category"),
                    "format": asset_meta.get("original_format"),
                    "tags": asset_meta.get("tags", []),
                    "description": asset_meta.get("description", ""),
                    "physics_validated": asset_meta.get("physics_validated", False),
                }
            )

        return filtered_assets

    def export_meta_report(
        self, output_path: Optional[Union[str, Path]] = None
    ) -> Path:
        """导出元数据报告"""
        meta_data = self.load_meta()

        # 生成报告
        report = {
            "summary": {
                "total_assets": len(meta_data.get("assets", {})),
                "schema_version": meta_data.get("schema_version"),
                "last_updated": meta_data.get("updated_at"),
                "categories": {},
                "formats": {},
                "validation_stats": {"physics_validated": 0, "total_validations": 0},
            },
            "assets": meta_data.get("assets", {}),
        }

        # 统计信息
        for asset_name, asset_meta in meta_data.get("assets", {}).items():
            # 按类别统计
            category = asset_meta.get("semantic_category", "uncategorized")
            report["summary"]["categories"][category] = (
                report["summary"]["categories"].get(category, 0) + 1
            )

            # 按格式统计
            format_name = asset_meta.get("original_format", "unknown")
            report["summary"]["formats"][format_name] = (
                report["summary"]["formats"].get(format_name, 0) + 1
            )

            # 验证统计
            if asset_meta.get("physics_validated"):
                report["summary"]["validation_stats"]["physics_validated"] += 1

            if asset_meta.get("validations"):
                report["summary"]["validation_stats"]["total_validations"] += len(
                    asset_meta["validations"]
                )

        # 保存报告
        if output_path is None:
            output_path = self.asset_dir / "meta_report.yaml"
        else:
            output_path = Path(output_path)

        with open(output_path, "w", encoding="utf-8") as f:
            yaml.dump(report, f, default_flow_style=False, allow_unicode=True)

        logger.info(f"Meta report exported to: {output_path}")
        return output_path
