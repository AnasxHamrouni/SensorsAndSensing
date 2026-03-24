# LAB06 LiDAR (MID-70) 



- Lab notebook: `Lab_6_LiDAR_ROS2.ipynb`
- Lab report draft: `SSensing-Lab6.md`
- Offline snapshots: `snapshots/`
- Exported maps: `room_map_3d*.ply`
- ROS 2 workspace: `ros2_ws/`
- Lab Docker and Livox setup files: `lab_LiDAR/`

## Equipment

- Livox MID-70 LiDAR
- Livox Converter 2.0
- M12 cable and Ethernet cable
- Power supply (9-30 V)
- PC/laptop with Ubuntu 22.04 and ROS 2 Humble

## LiDAR Setup

### 1 Physical connection

1. Connect MID-70 to Livox Converter.
2. Connect Converter to laptop/PC Ethernet port.
3. Power the Converter and LiDAR.

### 2 Network configuration

Use the provided script to configure Ethernet and check connectivity.

```bash
cd LAB06_Lidar/lab_LiDAR
sudo bash set_livox_connection.sh
```

Optional environment overrides:

```bash
sudo ROBOT_IP=192.168.123.104 HOST_IP_CIDR=192.168.123.10/24 bash set_livox_connection.sh <ethernet_iface>
```

Expected result: successful ping to LiDAR and interface configured in 192.168.123.x network.

### 3 Docker environment

Provided files:

- `lab_LiDAR/DockerfileLite`
- `lab_LiDAR/docker-compose.yaml`
- `lab_LiDAR/livox_lidar_config.json`

Start container:

```bash
cd LAB06_Lidar/lab_LiDAR
docker compose up -d
docker compose exec terminal bash
```

The compose file uses `network_mode: host` and mounts the lab workspace into the container.

## ROS 2 Build

Build lab package:

```bash
cd LAB06_Lidar/ros2_ws
colcon build --packages-select lidar_lab
source install/setup.bash
```

## Driver and Topic Check

Start Livox driver (from your Livox workspace/environment) and verify data:

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic echo --once /livox/lidar
```

Expected: `/livox/lidar` is publishing `sensor_msgs/msg/PointCloud2`.

## Task 2 - ICP and Map Stitching

### 1 Capture snapshots

Record from different positions with overlap:

```bash
ros2 bag record -o snapshot_1 /livox/lidar
ros2 bag record -o snapshot_2 /livox/lidar
ros2 bag record -o snapshot_4 /livox/lidar
```

Or timed capture:

```bash
timeout 10 ros2 bag record -o snapshot_1 /livox/lidar
```

### 2 Run notebook workflow

Open `Lab_6_LiDAR_ROS2.ipynb` and run code cells in order.

Notebook covers:

1. Rosbag to point cloud conversion.
2. Preprocessing (voxel, outlier removal, normals).
3. Custom NumPy ICP implementation.
4. Open3D ICP comparison (P2P and P2L).
5. Multi-snapshot alignment and quality gating.
6. Export merged maps.

Generated outputs include:

- `room_map_3d.ply`
- `room_map_3d_colored.ply`
- `room_map_3d_tuned.ply`
- `room_map_3d_colored_tuned.ply`

### 3 Publish combined map

```bash
ros2 run lidar_lab map_publisher
```

or

```bash
ros2 launch lidar_lab map_publisher.launch.py
```

RViz2 display:

- Add `PointCloud2` topic `/map_3d`

## Task 3 - Real-Time RANSAC Plane Segmentation

Run detector:

```bash
ros2 run lidar_lab plane_detector
```

or

```bash
ros2 launch lidar_lab plane_detector.launch.py
```

Published topics:

- `/planes/floor` (PointCloud2)
- `/planes/walls` (PointCloud2)
- `/planes/objects` (PointCloud2)
- `/planes/markers` (MarkerArray)

RViz2 displays:

1. `PointCloud2` -> `/planes/floor`
2. `PointCloud2` -> `/planes/walls`
3. `PointCloud2` -> `/planes/objects`
4. `MarkerArray` -> `/planes/markers`
