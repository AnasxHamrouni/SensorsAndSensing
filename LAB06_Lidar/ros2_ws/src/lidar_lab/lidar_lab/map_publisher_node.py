import struct
from pathlib import Path
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class MapPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('map_publisher_node')

        self.declare_parameter(
            'map_path',
            '/home/nasta/Documents/GitHub/SensorsAndSensing/LAB06_Lidar/room_map_3d_colored.ply',
        )
        self.declare_parameter('map_topic', '/map_3d')
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('publish_period_sec', 1.0)

        map_path = str(self.get_parameter('map_path').value)
        map_topic = str(self.get_parameter('map_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_period = float(self.get_parameter('publish_period_sec').value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(PointCloud2, map_topic, qos)

        self.points_xyz, self.points_rgb = self.load_ply_xyz_rgb(map_path)
        if self.points_xyz.shape[0] == 0:
            raise RuntimeError(f'Loaded map has no points: {map_path}')

        self.xyz_points = self.points_xyz.astype(np.float32).tolist()
        self.xyzrgb_points = None
        if self.points_rgb is not None:
            packed = (
                (self.points_rgb[:, 0].astype(np.uint32) << 16)
                | (self.points_rgb[:, 1].astype(np.uint32) << 8)
                | self.points_rgb[:, 2].astype(np.uint32)
            )
            rgb_float = packed.view(np.float32)
            xyz_f32 = self.points_xyz.astype(np.float32)
            self.xyzrgb_points = [
                (float(xyz_f32[i, 0]), float(xyz_f32[i, 1]), float(xyz_f32[i, 2]), float(rgb_float[i]))
                for i in range(xyz_f32.shape[0])
            ]

        self.timer = self.create_timer(publish_period, self.publish_map)

        color_info = 'with RGB colors' if self.points_rgb is not None else 'without RGB colors'
        self.get_logger().info(
            f'Map publisher ready. Loaded {self.points_xyz.shape[0]} points from {map_path}. '
            f'Publishing on {map_topic} with frame {self.frame_id} ({color_info}).'
        )

    def publish_map(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        if self.xyzrgb_points is None:
            msg = point_cloud2.create_cloud_xyz32(header, self.xyz_points)
        else:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg = point_cloud2.create_cloud(header, fields, self.xyzrgb_points)

        self.publisher.publish(msg)

    def load_ply_xyz_rgb(self, ply_path: str) -> Tuple[np.ndarray, np.ndarray | None]:
        path = Path(ply_path)
        if not path.exists():
            raise FileNotFoundError(f'Map file not found: {ply_path}')

        with path.open('rb') as file_obj:
            fmt, vertex_count, properties, header_len = self.parse_ply_header(file_obj)

            if fmt == 'ascii':
                file_obj.seek(header_len)
                return self.read_ascii_vertices(file_obj, vertex_count, properties)
            if fmt == 'binary_little_endian':
                file_obj.seek(header_len)
                return self.read_binary_vertices(file_obj, vertex_count, properties, '<')
            if fmt == 'binary_big_endian':
                file_obj.seek(header_len)
                return self.read_binary_vertices(file_obj, vertex_count, properties, '>')

            raise ValueError(f'Unsupported PLY format: {fmt}')

    def parse_ply_header(self, file_obj) -> Tuple[str, int, List[Tuple[str, str]], int]:
        first = file_obj.readline().decode('ascii', errors='ignore').strip()
        if first != 'ply':
            raise ValueError('Not a valid PLY file: missing ply header')

        fmt = None
        vertex_count = 0
        properties: List[Tuple[str, str]] = []
        in_vertex_element = False

        while True:
            line_bytes = file_obj.readline()
            if not line_bytes:
                raise ValueError('Invalid PLY header: missing end_header')
            line = line_bytes.decode('ascii', errors='ignore').strip()

            if line.startswith('format '):
                parts = line.split()
                if len(parts) < 2:
                    raise ValueError('Invalid PLY format declaration')
                fmt = parts[1]
            elif line.startswith('element '):
                parts = line.split()
                in_vertex_element = len(parts) >= 3 and parts[1] == 'vertex'
                if in_vertex_element:
                    vertex_count = int(parts[2])
                    properties = []
            elif in_vertex_element and line.startswith('property '):
                parts = line.split()
                if len(parts) >= 3 and parts[1] != 'list':
                    properties.append((parts[1], parts[2]))
            elif line == 'end_header':
                break

        if fmt is None:
            raise ValueError('Invalid PLY header: format not found')

        return fmt, vertex_count, properties, file_obj.tell()

    @staticmethod
    def numpy_dtype_from_ply(ply_type: str):
        mapping = {
            'char': np.int8,
            'uchar': np.uint8,
            'short': np.int16,
            'ushort': np.uint16,
            'int': np.int32,
            'uint': np.uint32,
            'float': np.float32,
            'double': np.float64,
        }
        if ply_type not in mapping:
            raise ValueError(f'Unsupported PLY property type: {ply_type}')
        return mapping[ply_type]

    def read_ascii_vertices(
        self,
        file_obj,
        vertex_count: int,
        properties: List[Tuple[str, str]],
    ) -> Tuple[np.ndarray, np.ndarray | None]:
        property_names = [name for _, name in properties]
        try:
            ix = property_names.index('x')
            iy = property_names.index('y')
            iz = property_names.index('z')
        except ValueError as exc:
            raise ValueError('PLY vertex does not contain x,y,z properties') from exc

        rgb_idx = self.find_rgb_indices(property_names)
        xyz = np.zeros((vertex_count, 3), dtype=np.float32)
        rgb = np.zeros((vertex_count, 3), dtype=np.uint8) if rgb_idx is not None else None
        for i in range(vertex_count):
            line = file_obj.readline().decode('ascii', errors='ignore').strip()
            if not line:
                raise ValueError('Unexpected EOF while reading ASCII PLY vertices')
            vals = line.split()
            xyz[i, 0] = float(vals[ix])
            xyz[i, 1] = float(vals[iy])
            xyz[i, 2] = float(vals[iz])
            if rgb_idx is not None:
                ir, ig, ib = rgb_idx
                rgb[i, 0] = np.uint8(float(vals[ir]))
                rgb[i, 1] = np.uint8(float(vals[ig]))
                rgb[i, 2] = np.uint8(float(vals[ib]))

        return xyz, rgb

    def read_binary_vertices(
        self,
        file_obj,
        vertex_count: int,
        properties: List[Tuple[str, str]],
        endian: str,
    ) -> Tuple[np.ndarray, np.ndarray | None]:
        property_names = [name for _, name in properties]
        dtypes = [self.numpy_dtype_from_ply(t) for t, _ in properties]

        try:
            ix = property_names.index('x')
            iy = property_names.index('y')
            iz = property_names.index('z')
        except ValueError as exc:
            raise ValueError('PLY vertex does not contain x,y,z properties') from exc

        rgb_idx = self.find_rgb_indices(property_names)

        struct_map = {
            np.dtype(np.int8): 'b',
            np.dtype(np.uint8): 'B',
            np.dtype(np.int16): 'h',
            np.dtype(np.uint16): 'H',
            np.dtype(np.int32): 'i',
            np.dtype(np.uint32): 'I',
            np.dtype(np.float32): 'f',
            np.dtype(np.float64): 'd',
        }

        fmt_chars = []
        for dtype in dtypes:
            fmt_chars.append(struct_map[np.dtype(dtype)])

        row_struct = struct.Struct(endian + ''.join(fmt_chars))
        row_size = row_struct.size

        xyz = np.zeros((vertex_count, 3), dtype=np.float32)
        rgb = np.zeros((vertex_count, 3), dtype=np.uint8) if rgb_idx is not None else None
        for i in range(vertex_count):
            data = file_obj.read(row_size)
            if len(data) != row_size:
                raise ValueError('Unexpected EOF while reading binary PLY vertices')
            vals = row_struct.unpack(data)
            xyz[i, 0] = float(vals[ix])
            xyz[i, 1] = float(vals[iy])
            xyz[i, 2] = float(vals[iz])
            if rgb_idx is not None:
                ir, ig, ib = rgb_idx
                rgb[i, 0] = np.uint8(vals[ir])
                rgb[i, 1] = np.uint8(vals[ig])
                rgb[i, 2] = np.uint8(vals[ib])

        return xyz, rgb

    @staticmethod
    def find_rgb_indices(property_names: List[str]) -> Tuple[int, int, int] | None:
        if {'red', 'green', 'blue'}.issubset(set(property_names)):
            return property_names.index('red'), property_names.index('green'), property_names.index('blue')
        if {'r', 'g', 'b'}.issubset(set(property_names)):
            return property_names.index('r'), property_names.index('g'), property_names.index('b')
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
