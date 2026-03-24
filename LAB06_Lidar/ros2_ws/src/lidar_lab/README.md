# lidar_lab (LAB06 only)

Real-time wall/floor segmentation node for Lab 6.

Implementation note: this node uses NumPy-based voxel downsampling and RANSAC plane fitting.
No `open3d` runtime dependency is required.

## Build

```bash
cd /home/nasta/Documents/GitHub/SensorsAndSensing/LAB06_Lidar/ros2_ws
colcon build --packages-select lidar_lab
source install/setup.bash
```

## Run

```bash
ros2 run lidar_lab plane_detector
```

or:

```bash
ros2 launch lidar_lab plane_detector.launch.py
```

Publish the stitched Task 2 map to RViz2 (`/map_3d`):

```bash
ros2 run lidar_lab map_publisher
```

or:

```bash
ros2 launch lidar_lab map_publisher.launch.py
```

## Topics

- Subscribed: `/livox/lidar` (`sensor_msgs/msg/PointCloud2`)
- Published: `/planes/markers` (`visualization_msgs/msg/MarkerArray`)
- Published: `/planes/floor` (`sensor_msgs/msg/PointCloud2`)
- Published: `/planes/walls` (`sensor_msgs/msg/PointCloud2`)
- Published: `/planes/objects` (`sensor_msgs/msg/PointCloud2`)
- Published: `/map_3d` (`sensor_msgs/msg/PointCloud2`)

## Recommended RViz2 displays

- PointCloud2: `/planes/floor`
- PointCloud2: `/planes/walls`
- PointCloud2: `/planes/objects`
- MarkerArray: `/planes/markers`

For stitched map visualization add:

- PointCloud2: `/map_3d`
