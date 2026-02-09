#!/usr/bin/env python3
"""
3D Point Cloud Mapping Node with Keyboard Controls

Controls:
  s + Enter  ->  SAVE map to PCD file
  l + Enter  ->  LOAD last saved map
  e + Enter  ->  END/PAUSE mapping (stop recording)
  r + Enter  ->  RESUME mapping (start recording)  
  c + Enter  ->  CLEAR current map
  q + Enter  ->  QUIT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
import sys
import threading
import glob
import select
from datetime import datetime

# TF2 for transforming points to map frame
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        self.map_dir = os.path.expanduser('~/3d_rover/maps')
        os.makedirs(self.map_dir, exist_ok=True)
        
        # Point cloud storage
        self.all_points = {}
        self.resolution = 0.05  # 5cm
        self.frame_count = 0
        self.is_mapping = True  # Recording state
        self.loaded_map_file = None
        
        # TF2 buffer and listener for transforming points to map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'odom'  # Transform points to odom frame
        
        # Subscriber for live point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            10
        )
        
        # Publisher for loaded map
        self.map_publisher = self.create_publisher(PointCloud2, '/saved_map', 10)
        self.loaded_points = None
        self.publish_timer = None
        
        # Status timer
        self.status_timer = self.create_timer(3.0, self.print_status)
        
        self.print_banner()
        
    def print_banner(self):
        print('')
        print('=' * 50)
        print('  3D POINT CLOUD MAPPING')
        print('=' * 50)
        print('')
        print('  Drive the robot with teleop to explore.')
        print('')
        print('  CONTROLS:')
        print('  ------------------------------------')
        print('  s + Enter  ->  SAVE map to PCD file')
        print('  l + Enter  ->  LOAD last saved map')
        print('  e + Enter  ->  END/PAUSE mapping')
        print('  r + Enter  ->  RESUME mapping')
        print('  c + Enter  ->  CLEAR current map')
        print('  q + Enter  ->  QUIT')
        print('  ------------------------------------')
        print('')
        
    def pointcloud_callback(self, msg):
        if not self.is_mapping:
            return
        
        # Get transform from lidar frame to odom frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if self.frame_count % 30 == 0:  # Only print occasionally
                self.get_logger().warn(f'TF not ready: {e}')
            return
            
        self.frame_count += 1
        
        # Get translation and rotation from transform
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        
        # Quaternion to rotation matrix
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
        # Rotation matrix from quaternion
        r00 = 1 - 2*(qy*qy + qz*qz)
        r01 = 2*(qx*qy - qz*qw)
        r02 = 2*(qx*qz + qy*qw)
        r10 = 2*(qx*qy + qz*qw)
        r11 = 1 - 2*(qx*qx + qz*qz)
        r12 = 2*(qy*qz - qx*qw)
        r20 = 2*(qx*qz - qy*qw)
        r21 = 2*(qy*qz + qx*qw)
        r22 = 1 - 2*(qx*qx + qy*qy)
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            # Transform point to odom frame
            px, py, pz = point[0], point[1], point[2]
            
            # Apply rotation and translation
            wx = r00*px + r01*py + r02*pz + tx
            wy = r10*px + r11*py + r12*pz + ty
            wz = r20*px + r21*py + r22*pz + tz
            
            key = (
                round(wx / self.resolution),
                round(wy / self.resolution),
                round(wz / self.resolution)
            )
            if key not in self.all_points:
                self.all_points[key] = point[3] if len(point) > 3 else 0.0
    
    def print_status(self):
        if self.is_mapping:
            status = "\033[92mRECORDING\033[0m"  # Green
        else:
            status = "\033[93mPAUSED\033[0m"     # Yellow
        print(f'  [{status}] Points: {len(self.all_points):,} | Frames: {self.frame_count}')
    
    def end_mapping(self):
        self.is_mapping = False
        print('')
        print('  \033[93m*** MAPPING PAUSED ***\033[0m')
        print('  Not recording new points.')
        print('  Press "r" to resume, "s" to save.')
        print('')
        
    def resume_mapping(self):
        self.is_mapping = True
        print('')
        print('  \033[92m*** MAPPING RESUMED ***\033[0m')
        print('  Recording new points...')
        print('')
        
    def clear_map(self):
        self.all_points.clear()
        self.frame_count = 0
        print('')
        print('  Map cleared!')
        print('')
        
    def save_map(self):
        if len(self.all_points) == 0:
            print('')
            print('  No points to save! Drive around first.')
            print('')
            return None
            
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.map_dir, f'map_{timestamp}.pcd')
        
        print('')
        print(f'  Saving {len(self.all_points):,} points...')
        
        # Convert to array
        points = []
        for key, intensity in self.all_points.items():
            x = key[0] * self.resolution
            y = key[1] * self.resolution
            z = key[2] * self.resolution
            points.append([x, y, z, intensity])
        
        points = np.array(points, dtype=np.float32)
        
        # Write PCD
        with open(filename, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z intensity\n')
            f.write('SIZE 4 4 4 4\n')
            f.write('TYPE F F F F\n')
            f.write('COUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            for p in points:
                f.write(f'{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[3]:.1f}\n')
        
        size_mb = os.path.getsize(filename) / (1024 * 1024)
        
        print('')
        print('  \033[92m============================================\033[0m')
        print('  \033[92m       MAP SAVED SUCCESSFULLY!\033[0m')
        print('  \033[92m============================================\033[0m')
        print(f'  Points: {len(points):,}')
        print(f'  Size:   {size_mb:.2f} MB')
        print(f'  File:   {os.path.basename(filename)}')
        print('  ============================================')
        print('')
        print(f'  View with: pcl_viewer {filename}')
        print('')
        
        # Stop recording after save
        self.is_mapping = False
        print('  \033[93m*** RECORDING STOPPED ***\033[0m')
        print('  Press "r" to resume, "l" to load, "q" to quit.')
        print('')
        
        return filename
        
    def load_map(self):
        # Find most recent PCD file
        pcd_files = glob.glob(os.path.join(self.map_dir, '*.pcd'))
        
        if not pcd_files:
            print('')
            print('  No saved maps found!')
            print(f'  Directory: {self.map_dir}')
            print('')
            return
        
        # Sort by modification time, get newest
        pcd_files.sort(key=os.path.getmtime, reverse=True)
        
        print('')
        print('  Available maps:')
        for i, f in enumerate(pcd_files[:5]):
            marker = '>' if i == 0 else ' '
            print(f'    {marker} {os.path.basename(f)}')
        print('')
        
        filename = pcd_files[0]
        print(f'  Loading: {os.path.basename(filename)}...')
        
        # Load PCD
        points = []
        data_started = False
        
        with open(filename, 'r') as f:
            for line in f:
                if data_started:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        intensity = float(parts[3]) if len(parts) > 3 else 0.0
                        points.append([x, y, z, intensity])
                elif line.startswith('DATA'):
                    data_started = True
        
        self.loaded_points = np.array(points, dtype=np.float32)
        self.loaded_map_file = filename
        
        print('')
        print('  \033[96m============================================\033[0m')
        print('  \033[96m        MAP LOADED SUCCESSFULLY!\033[0m')
        print('  \033[96m============================================\033[0m')
        print(f'  Points: {len(self.loaded_points):,}')
        print(f'  File:   {os.path.basename(filename)}')
        print('  ============================================')
        print('')
        print('  Publishing on /saved_map topic.')
        print('  In RViz: Add PointCloud2 -> /saved_map')
        print('')
        
        # Start publishing
        if self.publish_timer is None:
            self.publish_timer = self.create_timer(1.0, self.publish_loaded_map)
            
    def publish_loaded_map(self):
        if self.loaded_points is None:
            return
            
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.height = 1
        msg.width = len(self.loaded_points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = self.loaded_points.tobytes()
        
        self.map_publisher.publish(msg)


def input_thread(node, stop_event):
    import sys
    print('  >> Input thread ready. Type commands and press Enter.')
    sys.stdout.flush()
    
    while not stop_event.is_set():
        try:
            sys.stdout.write('  >> ')
            sys.stdout.flush()
            cmd = sys.stdin.readline().strip().lower()
            
            if not cmd:
                continue
                
            print(f'  [Command: {cmd}]')
            sys.stdout.flush()
            
            if cmd == 's':
                node.save_map()
            elif cmd == 'l':
                node.load_map()
            elif cmd == 'e':
                node.end_mapping()
            elif cmd == 'r':
                node.resume_mapping()
            elif cmd == 'c':
                node.clear_map()
            elif cmd == 'q':
                print('')
                print('  Quitting...')
                stop_event.set()
                break
            else:
                print(f'  Unknown command: {cmd}')
                print('  Valid: s=save, l=load, e=end, r=resume, c=clear, q=quit')
        except EOFError:
            break
        except Exception as ex:
            print(f'  Input error: {ex}')
            break


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    
    stop_event = threading.Event()
    input_handler = threading.Thread(target=input_thread, args=(node, stop_event))
    input_handler.daemon = True
    input_handler.start()
    
    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n  Interrupted. Saving map...')
        node.save_map()
    finally:
        stop_event.set()
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
