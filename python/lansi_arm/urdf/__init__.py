"""
URDF模块

提供URDF解析和碰撞检测功能。
"""

from .collision import (
    CollisionDetector,
    CollisionResult,
    CollisionConfig,
    CollisionType,
    BoundingBox,
    LinkBoundingBoxes,
)

__all__ = [
    "CollisionDetector",
    "CollisionResult",
    "CollisionConfig",
    "CollisionType",
    "BoundingBox",
    "LinkBoundingBoxes",
]
