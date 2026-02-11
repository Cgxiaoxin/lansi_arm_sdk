"""
Pytest 配置文件
"""
import pytest
import sys
import os

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


@pytest.fixture(scope="session")
def sample_joint_angles():
    """示例关节角度"""
    return [0.1, -0.2, 0.3, 0.4, -0.5, 0.6, 0.0]


@pytest.fixture(scope="session")
def sample_positions():
    """示例位置列表"""
    return [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
    ]


@pytest.fixture(scope="session")
def sample_waypoints(sample_positions):
    """示例路点"""
    from lansi_arm.controller import Waypoint

    return [
        Waypoint(positions=sample_positions[0], time=0.0),
        Waypoint(positions=sample_positions[1], time=1.0),
        Waypoint(positions=sample_positions[2], time=2.0),
    ]


@pytest.fixture
def cleanup(request):
    """清理fixture"""
    yield
    # 测试后的清理逻辑
    pass


def pytest_configure(config):
    """pytest配置"""
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line(
        "markers", "integration: marks tests as integration tests"
    )


def pytest_collection_modifyitems(config, items):
    """修改测试项目"""
    if not config.getoption("--runslow"):
        skip_slow = pytest.mark.skip(reason="need --runslow option to run")
        for item in items:
            if "slow" in item.keywords:
                item.add_marker(skip_slow)

    if not config.getoption("--runintegration"):
        skip_integration = pytest.mark.skip(reason="need --runintegration option to run")
        for item in items:
            if "integration" in item.keywords:
                item.add_marker(skip_integration)


def pytest_addoption(parser):
    """添加命令行选项"""
    parser.addoption(
        "--runslow", action="store_true", default=False, help="run slow tests"
    )
    parser.addoption(
        "--runintegration",
        action="store_true",
        default=False,
        help="run integration tests",
    )
