"""
Physics parameter validation utilities
"""

import logging
import math
from typing import Any, Dict, List, Optional, Tuple

# 现在使用USD风格的Asset类，不再需要单独的PhysicsProperties
from .asset import Asset

logger = logging.getLogger(__name__)


class ValidationResult:
    """验证结果类"""

    def __init__(self):
        self.is_valid = True
        self.warnings: List[str] = []
        self.errors: List[str] = []
        self.details: Dict = {}

    def add_warning(self, message: str) -> None:
        """添加警告"""
        self.warnings.append(message)
        logger.warning(message)

    def add_error(self, message: str) -> None:
        """添加错误"""
        self.errors.append(message)
        self.is_valid = False
        logger.error(message)

    def get_summary(self) -> Dict:
        """获取验证摘要"""
        return {
            "is_valid": self.is_valid,
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
            "errors": self.errors,
            "warnings": self.warnings,
            "details": self.details,
        }


class PhysicsValidator:
    """物理参数验证器"""

    def __init__(self, tolerance: float = 1e-6):
        self.tolerance = tolerance

    def validate_asset(self, asset: Asset) -> ValidationResult:
        """验证单个资产的物理参数"""
        result = ValidationResult()

        # 验证质量参数
        self._validate_masses(asset, result)

        # 验证惯性矩阵
        self._validate_inertias(asset, result)

        # 验证几何一致性
        self._validate_geometry_consistency(asset, result)

        return result

    def compare_assets(self, asset1: Asset, asset2: Asset) -> ValidationResult:
        """比较两个资产的物理参数一致性"""
        result = ValidationResult()

        # 比较链接数量
        if len(asset1.links) != len(asset2.links):
            result.add_warning(
                f"Different number of links: {len(asset1.links)} vs {len(asset2.links)}"
            )

        # 比较共同链接的物理属性
        common_links = set(asset1.links.keys()) & set(asset2.links.keys())

        for link_name in common_links:
            if (
                link_name in asset1.physics_properties
                and link_name in asset2.physics_properties
            ):
                self._compare_link_physics(
                    link_name,
                    asset1.physics_properties[link_name],
                    asset2.physics_properties[link_name],
                    result,
                )

        # 比较关节数量和类型
        self._compare_joints(asset1, asset2, result)

        return result

    def _validate_masses(self, asset: Asset, result: ValidationResult) -> None:
        """验证质量参数"""
        for link_name, props in asset.physics_properties.items():
            if props.mass is not None:
                if props.mass <= 0:
                    result.add_error(
                        f"Invalid mass for link '{link_name}': {props.mass}"
                    )
                elif props.mass < 1e-6:
                    result.add_warning(
                        f"Very small mass for link '{link_name}': {props.mass}"
                    )

    def _validate_inertias(self, asset: Asset, result: ValidationResult) -> None:
        """验证惯性矩阵"""
        for link_name, props in asset.physics_properties.items():
            if props.inertia is not None and len(props.inertia) >= 3:
                ixx, iyy, izz = props.inertia[:3]

                # 检查主惯性矩是否为正
                if ixx <= 0 or iyy <= 0 or izz <= 0:
                    result.add_error(
                        f"Invalid principal inertias for link '{link_name}': [{ixx}, {iyy}, {izz}]"
                    )

                # 检查惯性矩阵是否满足三角不等式
                if not self._check_inertia_triangle_inequality(ixx, iyy, izz):
                    result.add_error(
                        f"Inertia matrix violates triangle inequality for link '{link_name}'"
                    )

    def _check_inertia_triangle_inequality(
        self, ixx: float, iyy: float, izz: float
    ) -> bool:
        """检查惯性矩阵三角不等式"""
        return ixx + iyy >= izz and iyy + izz >= ixx and izz + ixx >= iyy

    def _validate_geometry_consistency(
        self, asset: Asset, result: ValidationResult
    ) -> None:
        """验证几何一致性"""
        # TODO: 实现几何一致性检查
        # 例如：检查碰撞网格是否包含在视觉网格内
        pass

    def _compare_link_physics(
        self,
        link_name: str,
        props1: Dict[str, Any],
        props2: Dict[str, Any],
        result: ValidationResult,
    ) -> None:
        """比较两个链接的物理属性"""

        # 比较质量
        mass1 = props1.get("physics:mass")
        mass2 = props2.get("physics:mass")
        if mass1 is not None and mass2 is not None:
            mass_diff = abs(mass1 - mass2)
            if mass_diff > self.tolerance:
                result.add_warning(
                    f"Mass difference for link '{link_name}': {mass_diff:.6f}"
                )

        # 比较惯性矩阵
        inertia1 = props1.get("physics:inertia")
        inertia2 = props2.get("physics:inertia")
        if inertia1 is not None and inertia2 is not None:
            inertia_diff = [abs(i1 - i2) for i1, i2 in zip(inertia1, inertia2)]
            max_diff = max(inertia_diff)
            if max_diff > self.tolerance:
                result.add_warning(
                    f"Inertia difference for link '{link_name}': max={max_diff:.6f}"
                )

        # 比较质心位置
        com1 = props1.get("physics:centerOfMass")
        com2 = props2.get("physics:centerOfMass")
        if com1 is not None and com2 is not None:
            com_diff = [abs(c1 - c2) for c1, c2 in zip(com1, com2)]
            max_com_diff = max(com_diff)
            if max_com_diff > self.tolerance:
                result.add_warning(
                    f"Center of mass difference for link '{link_name}': max={max_com_diff:.6f}"
                )

    def _compare_joints(
        self, asset1: Asset, asset2: Asset, result: ValidationResult
    ) -> None:
        """比较关节信息"""
        if len(asset1.joints) != len(asset2.joints):
            result.add_warning(
                f"Different number of joints: {len(asset1.joints)} vs {len(asset2.joints)}"
            )

        common_joints = set(asset1.joints.keys()) & set(asset2.joints.keys())

        for joint_name in common_joints:
            joint1 = asset1.joints[joint_name]
            joint2 = asset2.joints[joint_name]

            if joint1.joint_type != joint2.joint_type:
                result.add_error(
                    f"Joint type mismatch for '{joint_name}': {joint1.joint_type} vs {joint2.joint_type}"
                )

            # 比较关节限制
            if joint1.limits and joint2.limits:
                for limit_type in ["lower", "upper", "effort", "velocity"]:
                    if limit_type in joint1.limits and limit_type in joint2.limits:
                        diff = abs(
                            joint1.limits[limit_type] - joint2.limits[limit_type]
                        )
                        if diff > self.tolerance:
                            result.add_warning(
                                f"Joint limit difference for '{joint_name}.{limit_type}': {diff:.6f}"
                            )
