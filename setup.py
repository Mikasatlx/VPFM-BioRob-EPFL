import os
from glob import glob
from setuptools import setup

package_name = "autonomous_furniture"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anonymous",
    maintainer_email="anonymous@epfl.ch",
    description="TODO: Package description",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_publisher = autonomous_furniture.state_publisher:main",
        ],
    },
)
