import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'view'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kangy',
    maintainer_email='kangy@todo.todo',
    description='红色色块实时检测：HSV 二值化 + 四边形拟合 + 目标面 TF 发布',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_quad_detector = view.detector:main',
        ],
    },
)
