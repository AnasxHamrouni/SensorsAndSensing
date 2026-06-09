# Sensors & Sensing Lab 4 (IMU, ROS 2, Raspberry Pi 5)

All implementation and package development were performed on a **Raspberry Pi 5**.
ROS 2 was run inside a **Docker container on the Pi**, and development/control was done via **SSH**.

## ROS 2 usage on Raspberry Pi 5 board

ROS 2 was used directly on the Raspberry Pi 5 through Docker Compose (not on the laptop host OS):

- The container used a prebuilt ARM image: `fabook/iros:arm-lite-v0.0.1`
- Host networking was enabled (`network_mode: host`) so ROS 2 discovery/topics worked on the Pi network stack.
- Hardware was passed into the container, including `/dev/i2c-1`, so the MPU6050 could be read by ROS 2 nodes.
- The lab repository on the Pi was mounted into `/home/fabian/ros2_ws/src`, then built with `colcon` inside the container.
- All ROS 2 commands (`ros2 run`, `ros2 launch`, `rviz2`, topic checks) were executed inside the running container shell.

In short: laptop -> SSH into Pi -> enter ROS 2 container -> build and run nodes on the board.

## 1) Repository layout

- `docker-compose.yaml` — lab container runtime
- `imu_lab_ros2/` — main package for Tasks 3–5
- `tools/test_imu.py` — direct non-ROS I2C quick test

## 2) Prerequisites on Raspberry Pi

1. Enable I2C in `raspi-config`.
2. Confirm sensor appears at `0x68` or `0x69`:

```bash
sudo i2cdetect -y 1
```

3. Start the container:

```bash
docker compose up -d
docker compose exec terminal bash
```

## 3) SSH workflow used in this lab

From your laptop, connect to Raspberry Pi 5:

```bash
ssh <username>@<pi_ip>
```

Then inside the SSH session:

```bash
cd ~/lab
docker compose up -d
docker compose exec terminal bash
```

## 4) Build workspace in container

This repo is mounted to `/home/fabian/ros2_ws/src` by default.

```bash
cd /home/fabian/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## 5) Task 2 quick hardware test (no ROS 2)

```bash
python3 src/tools/test_imu.py --bus 1 --address 0x68 --samples 10
```

## 6) Task 3: raw IMU publisher

Run node:

```bash
ros2 run imu_lab_ros2 mpu6050_node
```

Check:

```bash
ros2 topic echo --once /imu/raw
ros2 topic hz /imu/raw
```

## 7) Task 4: Madgwick filter

```bash
ros2 launch imu_lab_ros2 madgwick.launch.py
```

Checks:

```bash
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/rpy/filtered
```

## 8) Task 5: EKF + RViz

Run full pipeline (raw IMU + Madgwick + static TF + EKF):

```bash
ros2 launch imu_lab_ros2 imu_pipeline.launch.py
```

Checks:

```bash
ros2 topic echo --once /odometry/filtered
ros2 run tf2_ros tf2_echo odom imu_link
```

RViz2:

- Fixed Frame: `odom`
- Add displays: `TF`, `Odometry` (`/odometry/filtered`)