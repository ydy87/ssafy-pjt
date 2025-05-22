from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 절대 경로 설정 (수정: 각자의 실제 위치에 맞게 변경 필요)
    dobot_bringup_path = '/home/ssafy/magician_ros2_control_system_ws/src/dobot_bringup/launch/dobot_magician_control_system.launch.py'
    dobot_description_path = '/home/ssafy/magician_ros2_control_system_ws/src/dobot_description/launch/display_with_rs.launch.py'

    # 서브 런치 포함
    dobot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dobot_bringup_path)
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dobot_description_path)
    )

    # Dobot homing 서비스 실행
    dobot_homing_action = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'lecture', 'dobot_homing_service'],
                shell=True,
                output='screen'
            )
        ]
    )

    # 개별 노드 실행
    gesture_ptp_move_node = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'dobot_gesture_control', 'gesture_ptp_move'],
                shell=True,
                output='screen'
            )
        ]
    )

    conveyor_controller_node = TimerAction(
        period=16.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'dobot_gesture_control', 'conveyor_controller_node'],
                shell=True,
                output='screen'
            )
        ]
    )

    gesture_bridge_node = TimerAction(
        period=17.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'dobot_gesture_control', 'gesture_bridge_node'],
                shell=True,
                output='screen'
            )
        ]
    )

    yolo_sorter_node = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'dobot_gesture_control', 'realsense_yolo_sorter'],
                shell=True,
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        dobot_bringup_launch,
        display_launch,
        dobot_homing_action,
        gesture_ptp_move_node,
        conveyor_controller_node,
        gesture_bridge_node,
        yolo_sorter_node,
    ])
