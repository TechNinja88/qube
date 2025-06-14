from setuptools import setup
import os
from glob import glob

package_name = 'qube_controller' #

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), # package.xml file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),# launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wahid',
    maintainer_email='wahidas@stud.ntnu.no',
    description='Controller package for Quanser Qube',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = qube_controller.pid_controller:main', # PID controller
        ],
    },
)
