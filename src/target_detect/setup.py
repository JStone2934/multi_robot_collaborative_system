from setuptools import setup
import os
from glob import glob

package_name = 'target_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jstone',
    maintainer_email='2823659087@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'dev': ['pytest'],  # 通过 extras_require 来指定测试依赖
    },
    entry_points={
        'console_scripts': [
            'target_detect = target_detect.target_detect:main',
            'test_detect = target_detect.test_detect:main'
        ],
    },
)
