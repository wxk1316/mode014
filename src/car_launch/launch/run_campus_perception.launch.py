"""Standalone new detector; existing localization may already be running."""
from campus_lidar_perception.launch_support import detection_launch


def generate_launch_description():
    return detection_launch()
