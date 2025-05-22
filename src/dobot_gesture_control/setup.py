from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dobot_gesture_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 필수: ament resource 등록 및 package.xml 포함
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 디렉토리 포함
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssafy',
    maintainer_email='ssafy@todo.todo',
    description='Dobot + Gesture + YOLO 분류 시스템 통합 제어 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_ptp_move = dobot_gesture_control.gesture_ptp_move:main',
            'gesture_bridge_node = dobot_gesture_control.gesture_bridge_node:main',
            'conveyor_controller_node = dobot_gesture_control.conveyor_controller_node:main',
            'realsense_yolo_sorter = dobot_gesture_control.realsense_yolo_sorter:main',
        ],
    },
)
