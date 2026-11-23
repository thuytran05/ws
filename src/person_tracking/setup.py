import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'person_tracking'

fixture_files = [f for f in glob('fixtures/*') if os.path.isfile(f)]
sample_bag_files = [f for f in glob('fixtures/sample_fixture_bag/*') if os.path.isfile(f)]

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/fixtures', fixture_files),
]
if sample_bag_files:
    data_files.append(('share/' + package_name + '/fixtures/sample_fixture_bag', sample_bag_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tran Van Thuy',
    maintainer_email='maintainer@example.com',
    description='YOLO pose selection, registered depth measurement and person following.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_tracker = person_tracking.person_tracker:main',
            'person_follower = person_tracking.person_follower:main',
            'perception_source = person_tracking.perception_source_node:main',
            'person_detector = person_tracking.person_detector_node:main',
        ],
    },
)
