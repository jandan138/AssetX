#!/usr/bin/env python3
"""
AssetX USD风格架构 - SdfPath路径系统

实现USD风格的分层路径系统，用于标识场景中的对象。
"""

from typing import Union


class SdfPath:
    """USD风格的路径系统

    提供分层路径操作，支持场景对象的唯一标识和导航。

    Examples:
        >>> path = SdfPath("/robot/base_link")
        >>> print(path.get_parent_path())  # /robot
        >>> print(path.get_name())         # base_link
        >>> child = path.append_child("visual")
        >>> print(child)                   # /robot/base_link/visual
    """

    def __init__(self, path_string: str):
        """初始化SdfPath

        Args:
            path_string: 路径字符串，应以'/'开头
        """
        self.path_string = path_string.strip()
        if not self.path_string.startswith("/"):
            self.path_string = "/" + self.path_string

    def __str__(self) -> str:
        return self.path_string

    def __repr__(self) -> str:
        return f"SdfPath('{self.path_string}')"

    def __eq__(self, other) -> bool:
        if isinstance(other, SdfPath):
            return self.path_string == other.path_string
        return False

    def __hash__(self) -> int:
        return hash(self.path_string)

    def get_parent_path(self) -> "SdfPath":
        """获取父路径

        Returns:
            父路径的SdfPath对象，根路径的父路径是自己
        """
        if self.path_string == "/":
            return SdfPath("/")

        parts = self.path_string.rstrip("/").split("/")
        if len(parts) <= 1:
            return SdfPath("/")

        parent_parts = parts[:-1]
        parent_path = "/".join(parent_parts) if len(parent_parts) > 1 else "/"
        return SdfPath(parent_path)

    def get_name(self) -> str:
        """获取路径的最后一个组件名称

        Returns:
            路径名称，根路径返回空字符串
        """
        if self.path_string == "/":
            return ""
        return self.path_string.rstrip("/").split("/")[-1]

    def append_child(self, child_name: str) -> "SdfPath":
        """追加子路径

        Args:
            child_name: 子路径名称

        Returns:
            新的SdfPath对象
        """
        if self.path_string == "/":
            return SdfPath(f"/{child_name}")
        else:
            return SdfPath(f'{self.path_string.rstrip("/")}/{child_name}')

    def is_root_path(self) -> bool:
        """是否为根路径

        Returns:
            True如果是根路径
        """
        return self.path_string == "/"

    @staticmethod
    def absolute_root_path() -> "SdfPath":
        """获取绝对根路径

        Returns:
            根路径SdfPath对象
        """
        return SdfPath("/")


__all__ = ["SdfPath"]
