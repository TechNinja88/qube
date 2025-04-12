from setuptools import setup
import os
from glob import glob
package_name = 'qube_bringup'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Launch files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')), # URDF files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # RViz files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wahid',
    maintainer_email='wahidas@stud.ntnu.no',
    description='Bringup package for Quanser Qube',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
