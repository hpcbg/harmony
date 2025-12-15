from setuptools import setup
import os
from glob import glob

package_name = 'fiware_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'requests', 'pyyaml'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a@a.com',
    description='Configurable FIWARE-ROS2 Bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'configurable_fiware_bridge = fiware_bridge.configurable_fiware_bridge:main',
        ],
    },
)
