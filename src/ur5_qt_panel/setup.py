# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/setup.py
# Summary: Setuptools setup for ur5_qt_panel package (panel_v2 entry point).

from setuptools import setup
from glob import glob
import os

package_name = 'ur5_qt_panel'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laboratorio',
    maintainer_email='jesus.lozano.rodriguez@gmail.com',
    description='Qt control panel for UR5 simulation (Gazebo + MoveIt + cameras).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panel_v2 = ur5_qt_panel.panel_v2:main',
        ],
    },
)
