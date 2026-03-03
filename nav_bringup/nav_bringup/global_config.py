FRAMES = {
    "base_frame": "base_link",
    "ground_truth_base_frame": "base_link_ground_truth",
    "imu_frame": "imu_link",
    "gps_frame": "gps_link",
    "odom_frame": "odom",
    "map_frame": "map",
}

# GPS origin in WGS84 degrees. All map-frame coordinates are relative to this point.
GPS_ORIGIN = {
    "latitude": 42.294621,
    "longitude": -83.708112,
}

# Magnetic declination at the deployment site (radians, west-negative).
MAGNETIC_DECLINATION_RADIANS = -0.1269
