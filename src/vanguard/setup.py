from setuptools import find_packages, setup

package_name = 'vanguard'

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
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vanguard_node = vanguard.vanguard_node:main',
            'mocap_to_px4_node = vanguard.mocap_to_px4_node:main',
            'px4_odom_bridge = vanguard.px4_odom_bridge:main',
            'vslam_to_px4_node = vanguard.vslam_to_px4_node:main',
            'cross_rel_gt = vanguard.cross_rel_gt:main',
        ],
    },
)
