"""Existing FAST_LIO algorithms with parameterised map files and centre reference."""
from campus_lidar_perception.launch_support import localization_launch


def generate_launch_description():
    return localization_launch()
