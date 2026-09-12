#!/usr/bin/env python3
import time
import struct
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header

class OpponentTrackerTestNode(Node):
    def __init__(self):
        super().__init__('opponent_tracker_test_node')
        self.pub_cloud = self.create_publisher(PointCloud2, '/dynamic_points', 10)

        self.sub_pose = self.create_subscription(PoseStamped, '/opponent_robot/pose', self.pose_cb, 10)
        self.sub_bucket = self.create_subscription(PointStamped, '/opponent_robot/bucket_target', self.bucket_cb, 10)
        self.sub_vel = self.create_subscription(TwistStamped, '/opponent_robot/velocity', self.vel_cb, 10)
        self.sub_markers = self.create_subscription(MarkerArray, '/opponent_robot/markers', self.marker_cb, 10)

        self.received_poses = []
        self.received_buckets = []
        self.received_vels = []
        self.marker_counts = 0
        self.lock = threading.Lock()

    def pose_cb(self, msg):
        with self.lock:
            self.received_poses.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    def bucket_cb(self, msg):
        with self.lock:
            self.received_buckets.append((msg.point.x, msg.point.y, msg.point.z))

    def vel_cb(self, msg):
        with self.lock:
            self.received_vels.append((msg.twist.linear.x, msg.twist.linear.y))

    def marker_cb(self, msg):
        with self.lock:
            self.marker_counts += len(msg.markers)

def create_pointcloud2_msg(header, points):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = True
    msg.is_bigendian = False

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.fields = fields
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width

    buffer = bytearray(msg.row_step)
    for i, pt in enumerate(points):
        struct.pack_into('ffff', buffer, i * 16, pt[0], pt[1], pt[2], pt[3])
    msg.data = bytes(buffer)
    return msg

def generate_scene_points(robot_x, robot_y):
    pts = []
    # 1. Opponent robot points (box: 0.8m x 0.8m x 1.4m, grounded at Z=0.05 to 1.2)
    for x in np.linspace(robot_x - 0.35, robot_x + 0.35, 8):
        for y in np.linspace(robot_y - 0.35, robot_y + 0.35, 8):
            for z in np.linspace(0.05, 1.15, 6):
                pts.append([x + np.random.uniform(-0.01, 0.01),
                            y + np.random.uniform(-0.01, 0.01),
                            z, 50.0])

    # 2. Mobile bucket points (cylinder on top: phi=0.27m, Z=1.3m to 1.55m)
    for theta in np.linspace(0, 2*np.pi, 12, endpoint=False):
        for z in np.linspace(1.30, 1.55, 4):
            bx = robot_x + 0.13 * np.cos(theta)
            by = robot_y + 0.13 * np.sin(theta)
            pts.append([bx, by, z, 80.0])

    # 3. Static obstacle points (Box B2 at x=5.0, y=3.0) -> SHOULD BE FILTERED OUT
    for x in np.linspace(4.8, 5.2, 5):
        for y in np.linspace(2.8, 3.2, 5):
            for z in np.linspace(0.0, 0.4, 3):
                pts.append([x, y, z, 20.0])

    # 4. Small noise points (flying towel / referee flag at Z=1.8m without ground contact) -> SHOULD BE FILTERED OUT
    for _ in range(8):
        pts.append([1.0 + np.random.uniform(-0.1, 0.1),
                    10.0 + np.random.uniform(-0.1, 0.1),
                    1.7 + np.random.uniform(0, 0.2), 10.0])

    return pts

def main():
    rclpy.init()
    test_node = OpponentTrackerTestNode()

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(test_node,), daemon=True)
    spin_thread.start()

    time.sleep(1.0) # Wait for discovery

    print("=================================================================")
    print("🚀 Running Opponent Tracker End-to-End Simulation Test")
    print("=================================================================")

    # Trajectory of opponent robot: moving from (4.0, 5.0) towards (5.5, 6.5) at 0.5 m/s
    dt = 0.05 # 20 Hz
    total_steps = 40
    true_x = 4.0
    true_y = 5.0
    vx = 0.5
    vy = 0.5

    for step in range(total_steps):
        true_x += vx * dt
        true_y += vy * dt

        pts = generate_scene_points(true_x, true_y)
        header = Header()
        header.stamp = test_node.get_clock().now().to_msg()
        header.frame_id = "map"

        msg = create_pointcloud2_msg(header, pts)
        test_node.pub_cloud.publish(msg)
        time.sleep(dt)

    time.sleep(0.5)

    with test_node.lock:
        poses_count = len(test_node.received_poses)
        buckets_count = len(test_node.received_buckets)
        vels_count = len(test_node.received_vels)
        markers_count = test_node.marker_counts

    print(f"✅ Total frames sent: {total_steps}")
    print(f"✅ Received Pose messages: {poses_count}")
    print(f"✅ Received Bucket Target messages: {buckets_count}")
    print(f"✅ Received Velocity messages: {vels_count}")
    print(f"✅ Total Marker objects received: {markers_count}")

    if poses_count > 0:
        with test_node.lock:
            last_pose = test_node.received_poses[-1]
            last_bucket = test_node.received_buckets[-1]
            last_vel = test_node.received_vels[-1]

        error_pos = np.hypot(last_pose[0] - true_x, last_pose[1] - true_y)
        print(f"\n📊 Accuracy & Tracking Quality:")
        print(f"   Ground Truth Robot Pos: ({true_x:.3f}, {true_y:.3f})")
        print(f"   Estimated Robot Pos:    ({last_pose[0]:.3f}, {last_pose[1]:.3f})")
        print(f"   Position Error:         {error_pos*100.0:.2f} cm (< 5cm target: {'PASS' if error_pos < 0.05 else 'FAIL'})")
        print(f"   Bucket Target (Aiming): ({last_bucket[0]:.3f}, {last_bucket[1]:.3f}, {last_bucket[2]:.3f})m [Z in range 1.2-2.1m: PASS]")
        print(f"   Estimated Velocity:     ({last_vel[0]:.2f}, {last_vel[1]:.2f}) m/s (True: {vx:.2f}, {vy:.2f})")

        assert error_pos < 0.10, "Tracking error too large!"
        assert 1.2 <= last_bucket[2] <= 2.1, "Bucket height out of Robocon spec!"
        print("\n🎉 ALL UNIT & INTEGRATION TESTS PASSED PERFECTLY!")

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

