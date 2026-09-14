#!/usr/bin/env python3
import time
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import Header
import tf2_ros

class TestOpponentPointcloudFeeder(Node):
    """
    Standalone unit test tool for opponent_tracker.
    Publishes synthetic opponent robot point clouds and field markers
    for testing without physical LiDAR or localization nodes.
    """
    def __init__(self):
        super().__init__('test_opponent_pointcloud_feeder')
        self.pub_cloud = self.create_publisher(PointCloud2, '/dynamic_cloud', 10)
        self.pub_field = self.create_publisher(MarkerArray, '/field_map_markers', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz
        self.start_time = time.time()

        self.init_field_cad()
        self.get_logger().info("=" * 60)
        self.get_logger().info(" [TEST] Opponent Point Cloud Feeder Started!")
        self.get_logger().info(" Streaming test point clouds to /dynamic_cloud at 20 Hz...")
        self.get_logger().info("=" * 60)

    def init_field_cad(self):
        markers = MarkerArray()

        # 1. Field Outer Border (10.5m x 11.4m centered at (0,0))
        border = Marker()
        border.header.frame_id = "map"
        border.ns = "field_border"
        border.id = 0
        border.type = Marker.LINE_STRIP
        border.action = Marker.ADD
        border.scale.x = 0.08
        border.color.r = 0.2
        border.color.g = 0.7
        border.color.b = 0.9
        border.color.a = 0.9

        pts_border = [
            (-5.25, -5.70, 0.0), (5.25, -5.70, 0.0),
            (5.25, 5.70, 0.0), (-5.25, 5.70, 0.0),
            (-5.25, -5.70, 0.0)
        ]
        for pt in pts_border:
            p = Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
            border.points.append(p)
        markers.markers.append(border)

        # 2. Central Barrier (0.0, 0.0, L=10.5m, W=0.6m, H=0.2m)
        barrier = Marker()
        barrier.header.frame_id = "map"
        barrier.ns = "field_barrier"
        barrier.id = 1
        barrier.type = Marker.CUBE
        barrier.action = Marker.ADD
        barrier.pose.position.x = 0.0
        barrier.pose.position.y = 0.0
        barrier.pose.position.z = 0.10
        barrier.scale.x = 10.5
        barrier.scale.y = 0.60
        barrier.scale.z = 0.20
        barrier.color.r = 0.8
        barrier.color.g = 0.5
        barrier.color.b = 0.2
        barrier.color.a = 0.75
        markers.markers.append(barrier)

        # 3. Field Posts
        posts = [(-1.27, 1.48), (-1.27, -1.48), (1.27, 1.48), (1.27, -1.48)]
        for idx, (px, py) in enumerate(posts):
            post = Marker()
            post.header.frame_id = "map"
            post.ns = "field_posts"
            post.id = 10 + idx
            post.type = Marker.CYLINDER
            post.action = Marker.ADD
            post.pose.position.x = float(px)
            post.pose.position.y = float(py)
            post.pose.position.z = 0.30
            post.scale.x = 0.30
            post.scale.y = 0.30
            post.scale.z = 0.60
            post.color.r = 0.7
            post.color.g = 0.7
            post.color.b = 0.2
            post.color.a = 0.85
            markers.markers.append(post)

        self.field_markers = markers

    def timer_callback(self):
        t = time.time() - self.start_time
        now = self.get_clock().now()

        # 1. Base TF for Our Robot (stationed in Area B: x=0.0, y=-2.5m)
        t_msg = TransformStamped()
        t_msg.header.stamp = now.to_msg()
        t_msg.header.frame_id = "map"
        t_msg.child_frame_id = "base_link"
        t_msg.transform.translation.x = 0.0
        t_msg.transform.translation.y = -2.5
        t_msg.transform.translation.z = 0.0
        t_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_msg)

        # 2. Opponent Robot Position along trajectory inside Area A (Y > 0.8m)
        orbit_speed = 0.40 # rad/s (~15 sec per cycle)
        opp_x = 2.5 * np.cos(orbit_speed * t)
        opp_y = 2.8 + 1.2 * np.sin(orbit_speed * t) # Moving in [1.6m, 4.0m] in Area A
        bucket_z = 1.45

        # 3. Generate Dense Test Point Cloud for Opponent Robot + Bucket
        pts = []
        # Chassis cube points (0.8m x 0.8m x 1.0m)
        for x in np.linspace(opp_x - 0.35, opp_x + 0.35, 9):
            for y in np.linspace(opp_y - 0.35, opp_y + 0.35, 9):
                for z in np.linspace(0.05, 0.95, 7):
                    if np.random.rand() > 0.10:
                        pts.append([x + np.random.normal(0, 0.005),
                                    y + np.random.normal(0, 0.005),
                                    z + np.random.normal(0, 0.005), 60.0])

        # Mobile Bucket PO-24A on Top (diameter=0.27m, H=0.25m, Z=1.35 to 1.55)
        for theta in np.linspace(0, 2 * np.pi, 20, endpoint=False):
            for z in np.linspace(bucket_z - 0.10, bucket_z + 0.10, 5):
                bx = opp_x + 0.135 * np.cos(theta)
                by = opp_y + 0.135 * np.sin(theta)
                pts.append([bx + np.random.normal(0, 0.005),
                            by + np.random.normal(0, 0.005),
                            z + np.random.normal(0, 0.005), 90.0])

        # Publish PointCloud2
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = "map"

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(pts)
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width

        buffer = bytearray(msg.row_step)
        for i, pt in enumerate(pts):
            struct.pack_into('ffff', buffer, i * 16, float(pt[0]), float(pt[1]), float(pt[2]), float(pt[3]))
        msg.data = bytes(buffer)

        self.pub_cloud.publish(msg)

        # Publish field CAD markers
        for m in self.field_markers.markers:
            m.header.stamp = now.to_msg()
        self.pub_field.publish(self.field_markers)

def main():
    rclpy.init()
    node = TestOpponentPointcloudFeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

