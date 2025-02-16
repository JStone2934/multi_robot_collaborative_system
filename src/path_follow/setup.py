from setuptools import setup

package_name = 'path_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abd',
    maintainer_email='abd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'dev': ['pytest'],  # 通过 extras_require 来指定测试依赖
    },
    entry_points={
        'console_scripts': [
            'path_follow = path_follow.path_follow:main'
        ],
    },
)
