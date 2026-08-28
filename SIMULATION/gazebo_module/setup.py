from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'gazebo_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),

        (os.path.join('share', package_name, 'config', 'rviz'),
         glob('config/rviz/*.rviz')),

        (os.path.join('share', package_name, 'world'),
         glob('world/*.world')),

        (os.path.join('share', package_name, 'urdf', 'fishbot'),
         glob('urdf/fishbot/*.xacro')),

        (os.path.join('share', package_name, 'urdf', 'fishbot', 'plugins'),
         glob('urdf/fishbot/plugins/*.xacro')),

        (os.path.join('share', package_name, 'urdf', 'fishbot', 'actuator'),
         glob('urdf/fishbot/actuator/*.xacro')),

        (os.path.join('share', package_name, 'urdf', 'fishbot', 'sensor'),
         glob('urdf/fishbot/sensor/*.xacro')),

        (os.path.join('share', package_name, 'urdf_old'),
         glob('urdf_old/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LiuGengrui',
    maintainer_email='2717915639@qq.com',
    description='robot simulation',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)