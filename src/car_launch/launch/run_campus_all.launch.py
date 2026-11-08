"""Sensors -> existing localization -> new perception -> existing RL-DWA -> car_ctr."""
from campus_lidar_perception.launch_support import combined_launch


def generate_launch_description():
    return combined_launch(full_stack=True)
