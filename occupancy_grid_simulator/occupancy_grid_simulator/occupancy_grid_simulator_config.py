from dataclasses import dataclass


@dataclass(frozen=True)
class OccupancyGridSimulatorConfig:
    map_file_path: str
    width_m: float = 5.0
    height_m: float = 5.0
    offset_x_m: float = 0.0
    offset_y_m: float = -2.5
    map_frame_id: str = "map"
    ground_truth_base_frame_id: str = "base_link_ground_truth"
    publish_period_s: float = 0.03
