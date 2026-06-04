#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import random
from typing import Optional, Tuple

import numpy as np
import rospy
import yaml

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler


Pose2D = Tuple[float, float, float]


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def load_map_yaml(yaml_path: str):
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)
    resolution = float(meta["resolution"])
    origin = tuple(meta["origin"])  # (x, y, yaw)
    return resolution, origin


def world_to_map(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    height: int,
    width: int,
) -> Optional[Tuple[int, int]]:
    mx = int((x - origin_x) / resolution)
    my = int((y - origin_y) / resolution)

    if mx < 0 or mx >= width or my < 0 or my >= height:
        return None

    row = height - 1 - my
    col = mx
    return row, col


def map_to_world(
    row: int,
    col: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    height: int,
) -> Tuple[float, float]:
    mx = col
    my = height - 1 - row
    x = origin_x + (mx + 0.5) * resolution
    y = origin_y + (my + 0.5) * resolution
    return x, y


def apply_motion_model(
    prev_odom: Pose2D,
    curr_odom: Pose2D,
    pose: Pose2D,
    alpha1: float = 0.05,
    alpha2: float = 0.05,
    alpha3: float = 0.05,
    alpha4: float = 0.05,
) -> Pose2D:
    x0, y0, th0 = prev_odom
    x1, y1, th1 = curr_odom
    px, py, pth = pose

    dx = x1 - x0
    dy = y1 - y0
    delta_trans = math.sqrt(dx * dx + dy * dy)

    if delta_trans < 1e-9:
        delta_rot1 = 0.0
    else:
        delta_rot1 = wrap_angle(math.atan2(dy, dx) - th0)

    delta_rot2 = wrap_angle(th1 - th0 - delta_rot1)

    std_rot1 = math.sqrt(alpha1 * (delta_rot1 ** 2) + alpha2 * (delta_trans ** 2))
    std_trans = math.sqrt(alpha3 * (delta_trans ** 2) + alpha4 * ((delta_rot1 ** 2) + (delta_rot2 ** 2)))
    std_rot2 = math.sqrt(alpha1 * (delta_rot2 ** 2) + alpha2 * (delta_trans ** 2))

    noisy_rot1 = delta_rot1 + random.gauss(0.0, std_rot1)
    noisy_trans = delta_trans + random.gauss(0.0, std_trans)
    noisy_rot2 = delta_rot2 + random.gauss(0.0, std_rot2)

    new_x = px + noisy_trans * math.cos(pth + noisy_rot1)
    new_y = py + noisy_trans * math.sin(pth + noisy_rot1)
    new_th = wrap_angle(pth + noisy_rot1 + noisy_rot2)

    return (new_x, new_y, new_th)


def likelihood_field_range_finder_model(
    scan: LaserScan,
    pose: Pose2D,
    likelihood_field: np.ndarray,
    resolution: float,
    origin: Tuple[float, float, float],
    z_hit: float = 0.95,
    z_rand: float = 0.05,
    max_beams: int = 20,
) -> float:
    x, y, theta = pose
    origin_x, origin_y, _ = origin

    h, w = likelihood_field.shape
    ranges = scan.ranges
    n = len(ranges)

    if n == 0:
        return 1e-12

    step = max(1, n // max_beams)
    q = 1.0

    for i in range(0, n, step):
        r = ranges[i]

        if not math.isfinite(r):
            continue
        if r <= scan.range_min or r >= scan.range_max:
            continue

        beam_angle = theta + scan.angle_min + i * scan.angle_increment
        hit_x = x + r * math.cos(beam_angle)
        hit_y = y + r * math.sin(beam_angle)

        rc = world_to_map(hit_x, hit_y, origin_x, origin_y, resolution, h, w)

        if rc is None:
            p = z_rand / max(scan.range_max, 1e-6)
        else:
            row, col = rc
            p_hit = float(likelihood_field[row, col])
            p = z_hit * p_hit + z_rand / max(scan.range_max, 1e-6)

        q *= max(p, 1e-12)

    return q


class ParticleFilterNode:
    def __init__(self) -> None:
        rospy.init_node("particle_filter_d2")

        self.map_yaml = rospy.get_param("~map_yaml")
        self.metric_map_npy = rospy.get_param("~metric_map_npy")
        self.likelihood_field_npy = rospy.get_param("~likelihood_field_npy")

        self.num_particles = int(rospy.get_param("~num_particles", 300))
        self.max_beams = int(rospy.get_param("~max_beams", 20))
        self.z_hit = float(rospy.get_param("~z_hit", 0.95))
        self.z_rand = float(rospy.get_param("~z_rand", 0.05))

        self.alpha1 = float(rospy.get_param("~alpha1", 0.02))
        self.alpha2 = float(rospy.get_param("~alpha2", 0.02))
        self.alpha3 = float(rospy.get_param("~alpha3", 0.02))
        self.alpha4 = float(rospy.get_param("~alpha4", 0.02))

        self.random_particle_ratio = float(rospy.get_param("~random_particle_ratio", 0.02))
        self.motion_threshold_trans = float(rospy.get_param("~motion_threshold_trans", 0.002))
        self.motion_threshold_rot = float(rospy.get_param("~motion_threshold_rot", 0.002))

        self.resolution, self.origin = load_map_yaml(self.map_yaml)

        if not os.path.exists(self.metric_map_npy):
            raise FileNotFoundError(f"Metric map file not found: {self.metric_map_npy}")
        if not os.path.exists(self.likelihood_field_npy):
            raise FileNotFoundError(f"Likelihood field file not found: {self.likelihood_field_npy}")

        self.metric_map = np.load(self.metric_map_npy)
        self.likelihood_field = np.load(self.likelihood_field_npy)

        self.h, self.w = self.metric_map.shape
        self.origin_x, self.origin_y, _ = self.origin

        self.free_cells = np.argwhere(self.metric_map == 0)
        if len(self.free_cells) == 0:
            raise RuntimeError("No free cells found in metric map.")

        self.prev_odom: Optional[Pose2D] = None
        self.curr_odom: Optional[Pose2D] = None
        self.latest_scan: Optional[LaserScan] = None
        self.initialized = False

        self.particles = np.zeros((self.num_particles, 3), dtype=np.float64)
        self.weights = np.ones(self.num_particles, dtype=np.float64) / self.num_particles
        self.est_pose: Pose2D = (0.0, 0.0, 0.0)

        self.particle_pub = rospy.Publisher("/pf/particles", PoseArray, queue_size=1)
        self.pose_pub = rospy.Publisher("/pf/estimated_pose", PoseStamped, queue_size=1)

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        rospy.loginfo("Particle filter D2 node ready.")

    def odom_msg_to_pose2d(self, msg: Odometry) -> Pose2D:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return (x, y, yaw)

    def sample_random_free_pose(self) -> Pose2D:
        idx = np.random.randint(len(self.free_cells))
        row, col = self.free_cells[idx]
        x, y = map_to_world(row, col, self.origin_x, self.origin_y, self.resolution, self.h)
        theta = np.random.uniform(-math.pi, math.pi)
        return (x, y, theta)

    def initialize_particles(self) -> None:
        for i in range(self.num_particles):
            self.particles[i, :] = self.sample_random_free_pose()
        self.weights.fill(1.0 / self.num_particles)
        self.est_pose = self.estimate_pose_uniform()
        self.initialized = True
        rospy.loginfo("Initialized particles over free space.")

    def pose_is_valid(self, pose: Pose2D) -> bool:
        x, y, _ = pose
        rc = world_to_map(x, y, self.origin_x, self.origin_y, self.resolution, self.h, self.w)
        if rc is None:
            return False
        row, col = rc
        return self.metric_map[row, col] == 0

    def odom_callback(self, msg: Odometry) -> None:
        new_odom = self.odom_msg_to_pose2d(msg)

        if self.curr_odom is None:
            self.curr_odom = new_odom
            self.prev_odom = new_odom
            return

        self.prev_odom = self.curr_odom
        self.curr_odom = new_odom

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

        if self.curr_odom is None or self.prev_odom is None:
            return

        if not self.initialized:
            self.initialize_particles()
            self.publish_particles()
            self.publish_estimated_pose()
            return

        if not self.has_significant_motion():
            self.publish_particles()
            self.publish_estimated_pose()
            return

        self.motion_update()
        self.sensor_update(msg)
        self.normalize_weights()
        self.est_pose = self.estimate_pose_weighted()
        self.resample()
        self.inject_random_particles()
        self.publish_particles()
        self.publish_estimated_pose()

        rospy.loginfo_throttle(
            1.0,
            f"[PF] est_pose=({self.est_pose[0]:.2f}, {self.est_pose[1]:.2f}, {self.est_pose[2]:.2f})"
        )

    def has_significant_motion(self) -> bool:
        dx = self.curr_odom[0] - self.prev_odom[0]
        dy = self.curr_odom[1] - self.prev_odom[1]
        dtrans = math.sqrt(dx * dx + dy * dy)
        drot = abs(wrap_angle(self.curr_odom[2] - self.prev_odom[2]))
        return (dtrans > self.motion_threshold_trans) or (drot > self.motion_threshold_rot)

    def motion_update(self) -> None:
        new_particles = np.zeros_like(self.particles)
        for i in range(self.num_particles):
            propagated = apply_motion_model(
                self.prev_odom,
                self.curr_odom,
                tuple(self.particles[i, :]),
                self.alpha1,
                self.alpha2,
                self.alpha3,
                self.alpha4,
            )
            if self.pose_is_valid(propagated):
                new_particles[i, :] = propagated
            else:
                new_particles[i, :] = self.sample_random_free_pose()
        self.particles = new_particles

    def sensor_update(self, scan: LaserScan) -> None:
        log_weights = np.zeros(self.num_particles, dtype=np.float64)

        for i in range(self.num_particles):
            q = likelihood_field_range_finder_model(
                scan,
                tuple(self.particles[i, :]),
                self.likelihood_field,
                self.resolution,
                self.origin,
                self.z_hit,
                self.z_rand,
                self.max_beams,
            )
            log_weights[i] = math.log(max(q, 1e-300))

        log_weights -= np.max(log_weights)
        self.weights = np.exp(log_weights)

    def normalize_weights(self) -> None:
        s = np.sum(self.weights)
        if s <= 1e-300:
            self.weights.fill(1.0 / self.num_particles)
        else:
            self.weights /= s

    def estimate_pose_uniform(self) -> Pose2D:
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        c = np.mean(np.cos(self.particles[:, 2]))
        s = np.mean(np.sin(self.particles[:, 2]))
        theta = math.atan2(s, c)
        return (x, y, theta)

    def estimate_pose_weighted(self) -> Pose2D:
        x = np.sum(self.weights * self.particles[:, 0])
        y = np.sum(self.weights * self.particles[:, 1])

        c = np.sum(self.weights * np.cos(self.particles[:, 2]))
        s = np.sum(self.weights * np.sin(self.particles[:, 2]))
        theta = math.atan2(s, c)

        return (x, y, theta)

    def resample(self) -> None:
        N = self.num_particles
        cumulative = np.cumsum(self.weights)
        positions = (np.arange(N) + np.random.uniform(0.0, 1.0)) / N
        indexes = np.zeros(N, dtype=np.int32)

        i = 0
        j = 0
        while i < N:
            if positions[i] < cumulative[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
                if j >= N:
                    j = N - 1

        self.particles = self.particles[indexes, :]
        self.weights.fill(1.0 / N)

    def inject_random_particles(self) -> None:
        k = int(self.random_particle_ratio * self.num_particles)
        if k <= 0:
            return

        replace_idx = np.random.choice(self.num_particles, size=k, replace=False)
        for idx in replace_idx:
            self.particles[idx, :] = self.sample_random_free_pose()

    def publish_particles(self) -> None:
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"

        poses = []
        for i in range(self.num_particles):
            x, y, theta = self.particles[i, :]
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)

            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.05
            p.orientation = Quaternion(qx, qy, qz, qw)
            poses.append(p)

        msg.poses = poses
        self.particle_pub.publish(msg)

    def publish_estimated_pose(self) -> None:
        x, y, theta = self.est_pose
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.10
        msg.pose.orientation = Quaternion(qx, qy, qz, qw)
        self.pose_pub.publish(msg)


if __name__ == "__main__":
    try:
        ParticleFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass