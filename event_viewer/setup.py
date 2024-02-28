from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'event_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='knorrrr',
    maintainer_email='kunotomoki61@gmail.com',
    description='Convert event information into images',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'event_viewer = event_viewer.event_viewer:main'
        ],
    },
)
