from glob import glob
from setuptools import find_packages, setup

package_name = "care_ml_websocket_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "fastapi>=0.63.0",
        "uvicorn>=0.15.0",
        "websockets>=9.1",
        "pydantic>=1.8.2",
        "aiomqtt>=1.0.0",
    ],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Bridge between CareRobotics task allocation and ROS2 Nav2 navigation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "care_ml_websocket_bridge = robot_ros_bridge.main:main",
            "care_bridge_cli = robot_ros_bridge.bridge_cli:main",
        ],
    },
)
