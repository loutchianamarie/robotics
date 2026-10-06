import glob
import os

from setuptools import find_packages, setup

package_name = 'multi_sensor_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob.glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob.glob('worlds/*')),
        (os.path.join('share', package_name, 'config'),
            glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Loutchiana Marie',
    maintainer_email='loutchiana9marie9@gmail.com',
    description='Simulation-based camera, LiDAR, and IMU perception prototype',
    license='Not specified',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = multi_sensor_perception.camera_node:main',
            'lidar_node = multi_sensor_perception.lidar_node:main',
            'imu_node = multi_sensor_perception.imu_node:main',
            'fusion_node = multi_sensor_perception.fusion_node:main',
            'latency_node = multi_sensor_perception.latency_node:main',
        ],
    },
)
