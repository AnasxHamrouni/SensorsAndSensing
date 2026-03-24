import time
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


@dataclass
class PlaneResult:
    points: np.ndarray
    normal: np.ndarray
    centroid: np.ndarray
    label: str


class PlaneDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('plane_detector_node')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('distance_threshold', 0.03)
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('num_iterations', 1500)

        self.declare_parameter('floor_normal_min_abs_z', 0.80)
        self.declare_parameter('wall_normal_max_abs_z', 0.45)
        self.declare_parameter('max_walls', 6)

        self.declare_parameter('min_points_to_process', 200)
        self.declare_parameter('min_floor_inliers', 500)
        self.declare_parameter('min_wall_inliers', 300)
        self.declare_parameter('remove_ceiling', True)
        self.declare_parameter('ceiling_percentile', 0.98)

        input_topic = str(self.get_parameter('input_topic').value)

        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            10,
        )

        self.floor_pub = self.create_publisher(PointCloud2, '/planes/floor', 10)
        self.walls_pub = self.create_publisher(PointCloud2, '/planes/walls', 10)
        self.objects_pub = self.create_publisher(PointCloud2, '/planes/objects', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planes/markers', 10)

        self.get_logger().info(f'Plane detector started. Subscribed to {input_topic}')

    def cloud_callback(self, msg: PointCloud2) -> None:
        t_start = time.perf_counter()
        xyz = self.pointcloud2_to_xyz(msg)

        if xyz.shape[0] < int(self.get_parameter('min_points_to_process').value):
            self.get_logger().warn('Too few points to process frame.')
            return

        t_down_start = time.perf_counter()
        down = self.voxel_downsample(xyz, float(self.get_parameter('voxel_size').value))
        t_down = time.perf_counter() - t_down_start

        if bool(self.get_parameter('remove_ceiling').value) and down.shape[0] > 0:
            ceil_q = float(self.get_parameter('ceiling_percentile').value)
            z_hi = np.quantile(down[:, 2], ceil_q)
            down = down[down[:, 2] <= z_hi]

        t_ransac_start = time.perf_counter()
        planes, objects = self.extract_planes_two_stage(down)
        t_ransac = time.perf_counter() - t_ransac_start

        floor_points = [p.points for p in planes if p.label == 'floor']
        wall_points = [p.points for p in planes if p.label == 'wall']

        floor_xyz = np.vstack(floor_points) if floor_points else np.empty((0, 3), dtype=np.float32)
        wall_xyz = np.vstack(wall_points) if wall_points else np.empty((0, 3), dtype=np.float32)

        t_pub_start = time.perf_counter()
        self.floor_pub.publish(self.xyz_to_pointcloud2(floor_xyz, msg))
        self.walls_pub.publish(self.xyz_to_pointcloud2(wall_xyz, msg))
        self.objects_pub.publish(self.xyz_to_pointcloud2(objects, msg))
        self.marker_pub.publish(self.build_markers(planes, msg))
        t_pub = time.perf_counter() - t_pub_start

        total = time.perf_counter() - t_start
        self.get_logger().info(
            f'Frame: in={xyz.shape[0]} down={down.shape[0]} '
            f'planes={len(planes)} floor={floor_xyz.shape[0]} wall={wall_xyz.shape[0]} obj={objects.shape[0]} | '
            f'down={t_down*1000:.1f}ms ransac={t_ransac*1000:.1f}ms pub={t_pub*1000:.1f}ms total={total*1000:.1f}ms'
        )

    def pointcloud2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not points:
            return np.empty((0, 3), dtype=np.float32)
        xyz = np.asarray([(float(point[0]), float(point[1]), float(point[2])) for point in points], dtype=np.float32)
        valid = np.isfinite(xyz).all(axis=1)
        return xyz[valid]

    def xyz_to_pointcloud2(self, xyz: np.ndarray, ref_msg: PointCloud2) -> PointCloud2:
        header = ref_msg.header
        if xyz.shape[0] == 0:
            return point_cloud2.create_cloud_xyz32(header, [])
        return point_cloud2.create_cloud_xyz32(header, xyz.astype(np.float32).tolist())

    def voxel_downsample(self, xyz: np.ndarray, voxel_size: float) -> np.ndarray:
        if xyz.shape[0] == 0 or voxel_size <= 0.0:
            return xyz.astype(np.float32)

        voxel_indices = np.floor(xyz / voxel_size).astype(np.int64)
        unique_voxels, inverse = np.unique(voxel_indices, axis=0, return_inverse=True)

        sums = np.zeros((unique_voxels.shape[0], 3), dtype=np.float64)
        np.add.at(sums, inverse, xyz.astype(np.float64))
        counts = np.bincount(inverse).reshape(-1, 1)

        down = sums / counts
        return down.astype(np.float32)

    def extract_planes_two_stage(self, xyz: np.ndarray) -> Tuple[List[PlaneResult], np.ndarray]:
        distance_threshold = float(self.get_parameter('distance_threshold').value)
        ransac_n = int(self.get_parameter('ransac_n').value)
        num_iterations = int(self.get_parameter('num_iterations').value)

        floor_normal_min_abs_z = float(self.get_parameter('floor_normal_min_abs_z').value)
        wall_normal_max_abs_z = float(self.get_parameter('wall_normal_max_abs_z').value)
        max_walls = int(self.get_parameter('max_walls').value)
        min_floor_inliers = int(self.get_parameter('min_floor_inliers').value)
        min_wall_inliers = int(self.get_parameter('min_wall_inliers').value)

        if ransac_n != 3:
            self.get_logger().warn('Only ransac_n=3 is supported; using 3.')

        remaining = xyz.copy()
        planes: List[PlaneResult] = []

        floor_normal, _, floor_idx = self.fit_plane_ransac_oriented(
            remaining,
            distance_threshold,
            num_iterations,
            orientation='floor',
            floor_normal_min_abs_z=floor_normal_min_abs_z,
            wall_normal_max_abs_z=wall_normal_max_abs_z,
        )
        if floor_idx.size >= min_floor_inliers:
            floor_pts = remaining[floor_idx]
            floor_centroid = np.mean(floor_pts, axis=0)
            planes.append(
                PlaneResult(
                    points=floor_pts,
                    normal=floor_normal,
                    centroid=floor_centroid,
                    label='floor',
                )
            )
            mask = np.ones(remaining.shape[0], dtype=bool)
            mask[floor_idx] = False
            remaining = remaining[mask]

        for _ in range(max_walls):
            if remaining.shape[0] < 3:
                break

            wall_normal, _, wall_idx = self.fit_plane_ransac_oriented(
                remaining,
                distance_threshold,
                num_iterations,
                orientation='wall',
                floor_normal_min_abs_z=floor_normal_min_abs_z,
                wall_normal_max_abs_z=wall_normal_max_abs_z,
            )

            if wall_idx.size < min_wall_inliers:
                break

            wall_pts = remaining[wall_idx]
            wall_centroid = np.mean(wall_pts, axis=0)
            planes.append(
                PlaneResult(
                    points=wall_pts,
                    normal=wall_normal,
                    centroid=wall_centroid,
                    label='wall',
                )
            )

            mask = np.ones(remaining.shape[0], dtype=bool)
            mask[wall_idx] = False
            remaining = remaining[mask]

        return planes, remaining

    @staticmethod
    def plane_from_points(points3: np.ndarray) -> Tuple[np.ndarray, float] | Tuple[None, None]:
        p1, p2, p3 = points3
        normal = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            return None, None
        normal = normal / norm
        d = -np.dot(normal, p1)
        return normal, float(d)

    def fit_plane_ransac_oriented(
        self,
        points: np.ndarray,
        distance_threshold: float,
        num_iterations: int,
        orientation: str,
        floor_normal_min_abs_z: float,
        wall_normal_max_abs_z: float,
    ) -> Tuple[np.ndarray, float, np.ndarray]:
        num_points = points.shape[0]
        if num_points < 3:
            return np.zeros(3, dtype=np.float64), 0.0, np.array([], dtype=np.int64)

        rng = np.random.default_rng()
        best_inliers = np.array([], dtype=np.int64)
        best_normal = None
        best_d = 0.0

        for _ in range(num_iterations):
            sample_idx = rng.choice(num_points, size=3, replace=False)
            normal, d = self.plane_from_points(points[sample_idx])
            if normal is None:
                continue

            abs_nz = abs(float(normal[2]))
            if orientation == 'floor' and abs_nz < floor_normal_min_abs_z:
                continue
            if orientation == 'wall' and abs_nz > wall_normal_max_abs_z:
                continue

            distances = np.abs(points @ normal + d)
            inliers = np.where(distances <= distance_threshold)[0]

            if inliers.size > best_inliers.size:
                best_inliers = inliers
                best_normal = normal
                best_d = d

                if best_inliers.size > 0.9 * num_points:
                    break

        if best_inliers.size < 3 or best_normal is None:
            return np.zeros(3, dtype=np.float64), 0.0, np.array([], dtype=np.int64)

        inlier_points = points[best_inliers].astype(np.float64)
        centroid = np.mean(inlier_points, axis=0)
        centered = inlier_points - centroid
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        refined_normal = vh[-1]
        refined_norm = np.linalg.norm(refined_normal)
        if refined_norm < 1e-9:
            refined_normal = best_normal
            refined_d = best_d
        else:
            refined_normal = refined_normal / refined_norm
            refined_d = -np.dot(refined_normal, centroid)

        abs_nz_refined = abs(float(refined_normal[2]))
        if orientation == 'floor' and abs_nz_refined < floor_normal_min_abs_z:
            return np.zeros(3, dtype=np.float64), 0.0, np.array([], dtype=np.int64)
        if orientation == 'wall' and abs_nz_refined > wall_normal_max_abs_z:
            return np.zeros(3, dtype=np.float64), 0.0, np.array([], dtype=np.int64)

        refined_dist = np.abs(points @ refined_normal + refined_d)
        refined_inliers = np.where(refined_dist <= distance_threshold)[0]
        return refined_normal.astype(np.float64), float(refined_d), refined_inliers.astype(np.int64)

    def build_markers(self, planes: List[PlaneResult], ref_msg: PointCloud2) -> MarkerArray:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header = ref_msg.header
        clear_marker.ns = 'planes'
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        marker_id = 1
        for i, plane in enumerate(planes):
            if plane.label == 'floor':
                color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.9)
            elif plane.label == 'wall':
                color = ColorRGBA(r=0.95, g=0.2, b=0.2, a=0.9)
            else:
                color = ColorRGBA(r=0.8, g=0.8, b=0.2, a=0.9)

            sphere = Marker()
            sphere.header = ref_msg.header
            sphere.ns = 'planes_centroids'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=float(plane.centroid[0]), y=float(plane.centroid[1]), z=float(plane.centroid[2]))
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color = color
            marker_array.markers.append(sphere)

            text = Marker()
            text.header = ref_msg.header
            text.ns = 'planes_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(
                x=float(plane.centroid[0]),
                y=float(plane.centroid[1]),
                z=float(plane.centroid[2] + 0.25),
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            text.text = f'plane {i}: {plane.label} ({plane.points.shape[0]})'
            marker_array.markers.append(text)

        return marker_array


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
