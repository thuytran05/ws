from setuptools import find_packages, setup

package_name = 'person_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot-developer',
    maintainer_email='developer@example.invalid',
    description='ROS 2 camera YOLO pose person selection (stage 1)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_pose_node = person_tracking.yolo_pose_node:main',
        ],
    },
)
