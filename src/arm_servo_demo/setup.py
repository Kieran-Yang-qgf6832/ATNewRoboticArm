import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_servo_demo'

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
    description='伺服模式演示：虚拟球沿水平圆周缓慢移动，末端速度伺服跟随',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_track_demo = arm_servo_demo.servo_track_demo:main',
        ],
    },
)
