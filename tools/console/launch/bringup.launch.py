#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    target = os.path.join(here, "..", "launch_bringup.py")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target))
    ])
