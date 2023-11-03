from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carrot_hunter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ige',
    maintainer_email='g.ivanov2@g.nsu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_broadcaster = carrot_hunter.static_turtle_tf2_broadcaster:main',
            'broadcaster = carrot_hunter.turtle_tf2_broadcaster:main',
            'listener = carrot_hunter.turtle_tf2_listener:main',
            'dynamic_frame = carrot_hunter.dynamic_frame_tf2_broadcaster:main',
        ],
    },
)
