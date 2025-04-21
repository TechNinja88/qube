from setuptools import setup
import os
from glob import glob

package_name = 'qube_description'# Corrected package name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),# URDF files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),# Launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),# RViz files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wahid',
    maintainer_email='wahidas@stud.ntnu.no',
    description='Quanser Qube description package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
