from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 등록용
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # 패키지 XML 파일
        ('share/' + package_name, ['package.xml']),
        # launch 폴더 안의 모든 .launch.py 파일
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        # maps 폴더 안의 모든 .yaml 파일
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml')
        ),
        # config  폴더 안의 모든 .yaml 파일
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros25',
    maintainer_email='songunhee5426@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_listener = turtlebot3_controller.sensor_listener:main',
            'navigator = turtlebot3_controller.navigator:main',
            'obstacle_avoider = turtlebot3_controller.obstacle_avoider:main',
        ],
    },
)
