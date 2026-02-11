"""
测试碰撞检测模块
"""
import pytest
import numpy as np
from lansi_arm.urdf import (
    BoundingBox,
    CollisionResult,
    CollisionConfig,
    CollisionType,
)


class TestBoundingBox:
    """包围盒测试类"""

    def test_bounding_box_creation(self):
        """测试包围盒创建"""
        bbox = BoundingBox(
            min_point=[0.0, 0.0, 0.0],
            max_point=[1.0, 1.0, 1.0]
        )
        assert bbox.min_point[0] == 0.0
        assert bbox.max_point[0] == 1.0

    def test_bounding_box_center(self):
        """测试包围盒中心计算"""
        bbox = BoundingBox(
            min_point=[0.0, 0.0, 0.0],
            max_point=[2.0, 4.0, 6.0]
        )
        center = bbox.center
        assert abs(center[0] - 1.0) < 1e-6
        assert abs(center[1] - 2.0) < 1e-6
        assert abs(center[2] - 3.0) < 1e-6

    def test_bounding_box_dimensions(self):
        """测试包围盒尺寸计算"""
        bbox = BoundingBox(
            min_point=[0.0, 0.0, 0.0],
            max_point=[2.0, 4.0, 6.0]
        )
        dimensions = bbox.dimensions
        assert abs(dimensions[0] - 2.0) < 1e-6
        assert abs(dimensions[1] - 4.0) < 1e-6
        assert abs(dimensions[2] - 6.0) < 1e-6

    def test_bounding_box_volume(self):
        """测试包围盒体积计算"""
        bbox = BoundingBox(
            min_point=[0.0, 0.0, 0.0],
            max_point=[2.0, 3.0, 4.0]
        )
        volume = bbox.volume
        assert abs(volume - 24.0) < 1e-6

    def test_bounding_box_intersection(self):
        """测试包围盒相交检测"""
        bbox1 = BoundingBox(
            min_point=[0.0, 0.0, 0.0],
            max_point=[1.0, 1.0, 1.0]
        )
        bbox2 = BoundingBox(
            min_point=[0.5, 0.5, 0.5],
            max_point=[1.5, 1.5, 1.5]
        )

        # 相交
        assert bbox1.intersects(bbox2)

    def test_bounding_box_no_intersection(self):
        """测试包围盒无相交"""
        bbox1 = BoundingBox(
            min_point=[0.0, 0.0, 0.0],
            max_point=[1.0, 1.0, 1.0]
        )
        bbox2 = BoundingBox(
            min_point=[2.0, 2.0, 2.0],
            max_point=[3.0, 3.0, 3.0]
        )

        # 不相交
        assert not bbox1.intersects(bbox2)

    def test_bounding_box_contains(self):
        """测试包围盒包含检测"""
        bbox1 = BoundingBox(
            min_point=[0.0, 0.0, 0.0],
            max_point=[2.0, 2.0, 2.0]
        )
        bbox2 = BoundingBox(
            min_point=[0.5, 0.5, 0.5],
            max_point=[1.5, 1.5, 1.5]
        )

        # bbox2包含于bbox1
        assert bbox1.contains(bbox2)

    def test_bounding_box_contains_point(self):
        """测试包围盒包含点"""
        bbox = BoundingBox(
            min_point=[0.0, 0.0, 0.0],
            max_point=[1.0, 1.0, 1.0]
        )

        # 点在内部
        assert bbox.contains_point([0.5, 0.5, 0.5])

        # 点在边界上
        assert bbox.contains_point([1.0, 0.5, 0.5])

        # 点在外部
        assert not bbox.contains_point([1.5, 0.5, 0.5])


class TestCollisionResult:
    """碰撞结果测试类"""

    def test_collision_result_no_collision(self):
        """测试无碰撞结果"""
        result = CollisionResult(
            is_collision=False,
            collision_type=CollisionType.NONE,
            distance=float('inf')
        )
        assert not result.is_collision
        assert result.collision_type == CollisionType.NONE

    def test_collision_result_self_collision(self):
        """测试自碰撞结果"""
        result = CollisionResult(
            is_collision=True,
            collision_type=CollisionType.SELF,
            distance=0.0,
            colliding_links=["link1", "link2"]
        )
        assert result.is_collision
        assert result.collision_type == CollisionType.SELF
        assert result.colliding_links == ["link1", "link2"]

    def test_collision_result_environment_collision(self):
        """测试环境碰撞结果"""
        result = CollisionResult(
            is_collision=True,
            collision_type=CollisionType.ENVIRONMENT,
            distance=0.0,
            colliding_links=["link3"]
        )
        assert result.is_collision
        assert result.collision_type == CollisionType.ENVIRONMENT


class TestCollisionConfig:
    """碰撞配置测试类"""

    def test_default_config(self):
        """测试默认配置"""
        config = CollisionConfig()
        assert config.enabled is True
        assert config.self_collision_enabled is True
        assert config.environment_collision_enabled is True
        assert config.safety_margin == 0.01

    def test_custom_config(self):
        """测试自定义配置"""
        config = CollisionConfig(
            enabled=True,
            self_collision_enabled=False,
            environment_collision_enabled=True,
            safety_margin=0.05
        )
        assert config.self_collision_enabled is False
        assert config.safety_margin == 0.05


class TestCollisionType:
    """碰撞类型测试类"""

    def test_collision_type_values(self):
        """测试碰撞类型枚举值"""
        assert CollisionType.NONE.value == "none"
        assert CollisionType.SELF.value == "self"
        assert CollisionType.ENVIRONMENT.value == "environment"
        assert CollisionType.EXTERNAL.value == "external"

    def test_collision_type_count(self):
        """测试碰撞类型数量"""
        assert len(CollisionType) == 4


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
