from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'simple_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='136497181+HatsuharuYasa@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = simple_agent.controller_node:main',
            'path_publisher = simple_agent.path_publisher:main',
            'odom_publisher = simple_agent.odom_publisher:main',
            'dummy_lidar = simple_agent.dummy_lidar:main',
            'pure_pursuit = simple_agent.pure_pursuit:main',
            'map_publisher = simple_agent.map_publisher:main',
            'run_amcl = simple_agent.run_amcl:main'
        ],
    },
)
