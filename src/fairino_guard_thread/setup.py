from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fairino_guard_thread'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),glob('config/*')),   # 安装config目录
        (os.path.join('share', package_name, 'launch'),glob('launch/*')),   # 安装launch目录
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miku',
    maintainer_email='1310946137@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "fairino_guard_thread_node=fairino_guard_thread.fairino_guard_thread:main"
        ],
    },
)
