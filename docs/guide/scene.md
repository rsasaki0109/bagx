# scene — 3D State Extraction

Extract position, orientation, and velocity time series from a bag.

## Usage

```bash
bagx scene recording.db3
bagx scene recording.db3 --csv trajectory.csv
bagx scene recording.db3 --json scene.json
bagx scene recording.db3 --topics /odom,/imu
```

## Auto-detected message types

| Type | Extracted fields |
|------|-----------------|
| `sensor_msgs/NavSatFix` | Position (lat, lon, alt) |
| `sensor_msgs/Imu` | Orientation, angular velocity, acceleration |
| `geometry_msgs/PoseStamped` | Position, orientation |
| `geometry_msgs/PoseWithCovarianceStamped` | Position, orientation |
| `nav_msgs/Odometry` | Position, orientation, linear/angular velocity |
| `tf2_msgs/TFMessage` | Position, orientation (from transforms) |

## CSV output columns

```
timestamp, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w,
vel_x, vel_y, vel_z, ang_vel_x, ang_vel_y, ang_vel_z,
accel_x, accel_y, accel_z, source
```

Fields are `None` when not available from the source topic.
