#!/usr/bin/env python3
"""Realistic wheel odometry and IMU yaw from the simulator's ground truth.

gazebo_ros_planar_move reports perfect odometry; an omni-wheel robot does not. This node turns
the true motion into what the robot would measure:
  /odom      body velocities with a per-run scale error (larger sideways), slip and white noise,
             integrated into an odom-frame pose; also broadcasts odom -> base_footprint
  /imu/yaw   true yaw + an arbitrary constant offset + a slow random-walk bias + white noise
The true pose comes from planar_move (/sim/odom_true, world = field frame).
"""
import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Header


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class OdomNoise:
    def __init__(self):
        p = lambda k, d: rospy.get_param('~' + k, d)
        self.scale_std = p('scale_error_std', [0.02, 0.05, 0.02])     # vx, vy, wz (1 sigma, per run)
        self.noise_std = p('noise_std', [0.01, 0.02, 0.01])           # m/s, m/s, rad/s (white)
        self.slip = p('lateral_slip', 0.03)                            # extra |vy| loss fraction
        self.imu_noise = p('imu_noise_std', 0.002)                     # rad
        self.imu_bias_walk = p('imu_bias_walk', 0.0005)                # rad / sqrt(s)
        self.max_speed = p('max_plausible_speed', 3.0)                 # m/s; larger jumps = teleport
        self.rng = np.random.default_rng(p('seed', 0) or None)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/yaw', Float64, queue_size=10)
        self.tfb = tf2_ros.TransformBroadcaster()
        self.new_run()
        rospy.Subscriber('/avoid_sim/reset_event', Header, lambda _: self.new_run())
        rospy.Subscriber('/sim/odom_true', Odometry, self.on_truth, queue_size=50)

    def new_run(self):
        self.scale = 1.0 + self.rng.normal(0.0, self.scale_std)
        self.imu_offset = self.rng.uniform(-math.pi, math.pi)
        self.imu_bias = 0.0
        self.pose = [0.0, 0.0, 0.0]          # odom frame starts where the robot is at reset
        self.last = None

    def on_truth(self, msg):
        t = msg.header.stamp.to_sec()
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = yaw_of(msg.pose.pose.orientation)
        if self.last is None:
            self.last = (t, x, y, yaw)
            return
        t0, x0, y0, yaw0 = self.last
        dt = t - t0
        if dt <= 0:
            return
        self.last = (t, x, y, yaw)
        vxw, vyw, wz = (x - x0) / dt, (y - y0) / dt, wrap(yaw - yaw0) / dt
        if math.hypot(vxw, vyw) > self.max_speed:                      # teleported by a reset
            return
        c, s = math.cos(yaw0), math.sin(yaw0)
        vb = np.array([c * vxw + s * vyw, -s * vxw + c * vyw, wz])     # true body velocity
        vb = vb * self.scale
        vb[1] *= 1.0 - self.slip
        vb += self.rng.normal(0.0, self.noise_std)
        if math.hypot(vxw, vyw) < 1e-4 and abs(wz) < 1e-4:
            vb[:] = 0.0                                                # wheels stopped: no drift
        px, py, pth = self.pose
        c, s = math.cos(pth), math.sin(pth)
        px += (c * vb[0] - s * vb[1]) * dt
        py += (s * vb[0] + c * vb[1]) * dt
        pth = wrap(pth + vb[2] * dt)
        self.pose = [px, py, pth]

        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = msg.header.stamp, 'odom', 'base_footprint'
        od.pose.pose.position.x, od.pose.pose.position.y = px, py
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(pth / 2), math.cos(pth / 2)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = vb
        for i, sd in zip((0, 7, 35), self.noise_std):
            od.twist.covariance[i] = sd * sd
        self.odom_pub.publish(od)

        tf = TransformStamped()
        tf.header, tf.child_frame_id = od.header, 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = px, py
        tf.transform.rotation = od.pose.pose.orientation
        self.tfb.sendTransform(tf)

        self.imu_bias += self.rng.normal(0.0, self.imu_bias_walk * math.sqrt(dt))
        self.imu_pub.publish(Float64(wrap(yaw + self.imu_offset + self.imu_bias +
                                          self.rng.normal(0.0, self.imu_noise))))


if __name__ == '__main__':
    rospy.init_node('odom_noise')
    OdomNoise()
    rospy.spin()
