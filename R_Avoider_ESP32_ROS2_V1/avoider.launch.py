"""Menjalankan avoider_brain + desktop (radar tkinter) sekaligus.

    source /opt/ros/humble/setup.zsh
    ros2 launch ./avoider.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', os.path.join(HERE, 'avoider_brain.py')], output='screen'),
        ExecuteProcess(cmd=['python3', os.path.join(HERE, 'desktop.py')], output='screen'),
    ])
