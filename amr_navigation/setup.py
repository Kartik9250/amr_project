from setuptools import setup
import os
from glob import glob

package_name = 'amr_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.pgm'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kartik',
    maintainer_email='kartikrana9250@gmail.com',
    description='AMR simulation package',
    license='Apache License 2.0',
    entry_points={
    },
)
