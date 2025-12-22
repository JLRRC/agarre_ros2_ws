# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/setup.py
# Summary: Setuptools setup for ur5_qt_panel package.
"""Setuptools entry point for the ur5_qt_panel package."""
from setuptools import setup

package_name = 'ur5_qt_panel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laboratorio',
    maintainer_email='jesus.lozano.rodriguez@gmail.com',
    description='Qt control panel for UR5 simulation (camera + buttons).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5_qt_panel = ur5_qt_panel.main_panel:main',
        ],
    },
)
