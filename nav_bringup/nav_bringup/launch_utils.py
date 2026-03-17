import json

import yaml
from ament_index_python.packages import get_package_share_directory


def bringup_share() -> str:
    return get_package_share_directory("nav_bringup")


def load_frames() -> dict:
    with open(f"{bringup_share()}/config/frames.yaml") as f:
        return yaml.safe_load(f)


def load_gps_file(course: str) -> dict:
    with open(f"{bringup_share()}/courses/{course}/gps.json") as f:
        return json.load(f)
