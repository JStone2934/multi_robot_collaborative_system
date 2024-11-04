from setuptools import setup
import os
from glob import glob

package_name = 'gazebo_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 确保 launch 文件夹和其中的文件被包含
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),  # 修正这里
    ],
    package_data={
        package_name: [
            'worlds/*',  # 包含 worlds 文件夹下的所有文件
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jstone',
    maintainer_email='2823659087@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
