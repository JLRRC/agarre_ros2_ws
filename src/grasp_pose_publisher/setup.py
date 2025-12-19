from setuptools import setup

package_name = 'grasp_pose_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laboratorio',
    maintainer_email='laboratorio@example.com',
    description='Nodo ROS 2 que publica una pose de agarre a partir de la cámara simulada en Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_pose_node = grasp_pose_publisher.grasp_pose_node:main',
        ],
    },
)

