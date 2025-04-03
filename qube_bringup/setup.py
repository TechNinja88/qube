<<<<<<< HEAD
from setuptools import find_packages, setup
import os
from glob import glob  # <-- Add this import

=======
from setuptools import setup
import os
from glob import glob
>>>>>>> 330d8b6d6551b8fa5b585ec0b9beba5b58b05ad5

package_name = 'qube_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
<<<<<<< HEAD
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
=======
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
>>>>>>> 330d8b6d6551b8fa5b585ec0b9beba5b58b05ad5
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
