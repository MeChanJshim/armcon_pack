from glob import glob
import os

from setuptools import setup

package_name = "Y2JoyGUI"
module_name = "y2_joy_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[module_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jaeyun Sim",
    maintainer_email="wodbs02221@gmail.com",
    description="Browser GUI for joystick control and generated PTP/TXTLoad motion commands.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "y2_joy_gui_node = y2_joy_gui.gui_node:main",
        ],
    },
)
