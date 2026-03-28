# gps_origin_calculator
Collects GPS samples on startup, then computes and prints the median position. This should be used to set "datum" in 
`gps.json` before a run.

```
ros2 launch gps_origin_calculator gps_origin_calculator.launch.py
```

Samples are filtered by horizontal accuracy (`max_horizontal_stdev_m`). The node waits until either `min_samples_required`
samples have been collected for at least `min_sample_duration_s`, or `max_sample_duration_s` has elapsed.

## Subscribed Topics
- `gps` (`sensor_msgs/NavSatFix`) - GPS fix

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `min_samples_required` | `100` | Minimum number of valid GPS samples before setting datum |
| `min_sample_duration_s` | `5.0` | Minimum time (s) to collect samples |
| `max_sample_duration_s` | `30.0` | Maximum time (s) before setting datum regardless of sample count |
| `max_horizontal_stdev_m` | `1.0` | Maximum horizontal accuracy (m) for a sample to be accepted |
