"""Sensors, localization and the new detector; no planning/control nodes."""
from campus_lidar_perception.launch_support import combined_launch


def generate_launch_description():
    return combined_launch(full_stack=False)
