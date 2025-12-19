from setuptools import setup

package_name = 'ur5_tools'

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
    description='Tools for UR5 simulation (camera capture, etc.)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_capture = ur5_tools.camera_capture:main',
            'fake_cameras = ur5_tools.fake_cameras:main',
        ],
    },
)
