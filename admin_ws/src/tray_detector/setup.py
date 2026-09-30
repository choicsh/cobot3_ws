from setuptools import setup

package_name = "tray_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="choicsh",
    maintainer_email="csh980625@gmail.com",
    description="Isaac Sim 손목 D455 스트림에서 트레이를 검출해 3D 좌표를 돌려주는 관제 PC 노드",
    license="MIT",
    entry_points={
        "console_scripts": [
            "detect_node = tray_detector.detect_node:main",
        ],
    },
)
