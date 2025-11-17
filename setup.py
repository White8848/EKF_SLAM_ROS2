from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ekf_slam'

# 先放基础的 data_files
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # 安装所有 launch 文件
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    # 安装 RViz 配置文件
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    # 安装 Nav2 配置文件
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]

# ✅ 递归安装 worlds 目录下的所有文件（而不是目录本身）
worlds_dir = 'worlds'
if os.path.isdir(worlds_dir):
    for root, dirs, files in os.walk(worlds_dir):
        if not files:
            continue
        # root 例如：worlds/bookstore 或 worlds/bookstore/models
        install_dir = os.path.join('share', package_name, root)
        file_paths = [os.path.join(root, f) for f in files]
        data_files.append((install_dir, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hqh',
    maintainer_email='hqh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_slam_node = ekf_slam.ekf_slam_node:main',
        ],
    },
)
