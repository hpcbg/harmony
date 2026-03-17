from setuptools import find_packages, setup

package_name = 'xarm_pack_bottle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/xarm_pack_bottle.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaloyan',
    maintainer_email='kyovchev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'xarm_pack_bottle = xarm_pack_bottle.xarm_pack_bottle:main'
        ],
    },
)
