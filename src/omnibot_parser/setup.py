from setuptools import find_packages, setup

package_name = 'omnibot_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='druegg',
    maintainer_email='david.ruegg@epfl.ch',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automatic_parser = omnibot_parser.automatic_parser:main',
            'safety_stop = omnibot_parser.safety_stop:main',
        ],
    },
)
