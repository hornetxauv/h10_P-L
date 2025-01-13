import os
from glob import glob

from setuptools import find_packages, setup

package_name = "h9_camera_package"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files. (includes both python and xml launch files)
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shengbin",
    maintainer_email="shengbin.chan@gmail.com",
    description="Camera Package to perform Stereo Vision and Object Detection",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lines = h9_camera_package.lines:main",
            "stereo_vision = h9_camera_package.stereo_vision:main",
            "export = h9_camera_package.export:main",
            "flare = h9_camera_package.flare:main",
            "calibration = h9_camera_package.calibration:main",
            "compressed_v2 = h9_camera_package.compressed_v2:main",
            "compressed = h9_camera_package.compressed:main",
            "stereo_rectify = h9_camera_package.stereo_rectify:main",
            "isaac_ros_yolov8_visualizer = h9_camera_package.isaac_ros_yolov8_visualizer:main",
            "compressed_img2video = h9_camera_package.compressed_img2video:main",
            "live_hsv_threshold = h9_camera_package.live_hsv_threshold:main",
            "led_comms = h9_camera_package.led_comms:main",
            "white_balance = h9_camera_package.white_balance:main",
            "debug_hsv = h9_camera_package.debug_hsv:main"
        ],
    },
)
