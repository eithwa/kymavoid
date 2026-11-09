#!/usr/bin/env python3
"""Real-robot bridge between REP 103 topics and the motion / IMU nodes. NOT CALIBRATED YET.

  /cmd_vel (m/s, rad/s, base frame)  -> /motion/cmd_vel  (Attack_motion_node, % of max speed)
  /motion/motionFB                   -> /odom + TF odom -> base_footprint
  /imu_3d (imu_3d/inertia, yaw rad)  -> /imu/yaw

Defaults come from reading the old code, not from measurements:
  * FIRA_strategy -> velocity_S_planning -> /motion/cmd_vel: linear.y = forward, linear.x = right,
    values are percent of the maximum motor speed (Attack_motion prints "speed(%)").
  * motionFB uses the same axes; its unit depends on wheel_radius / gearing in base_control.cpp.
  * the IMU yaw sign relative to REP 103 is unknown.
Measure on the robot and set: cmd_percent_per_mps, fb_mps_per_unit, cmd_yaw_scale, imu_sign.
"""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

try:
    from imu_3d.msg import inertia
except ImportError:          # imu_3d not built: run without the IMU bridge
    inertia = None


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MotionAdapter:
    def __init__(self):
        p = lambda k, d: rospy.get_param('~' + k, d)
        self.cmd_scale = p('cmd_percent_per_mps', 100.0)      # % per m/s        (uncalibrated)
        self.cmd_yaw_scale = p('cmd_yaw_scale', 1.0)          # motion unit per rad/s (uncalibrated)
        self.fb_scale = p('fb_mps_per_unit', 1.0)             # m/s per motionFB unit (uncalibrated)
        self.imu_sign = p('imu_sign', 1.0)                    # +1 if imu yaw grows counter-clockwise
        self.max_percent = p('max_percent', 100.0)
        rospy.logwarn('motion_adapter: axis mapping and scales are NOT calibrated (see the module docstring)')
        self.cmd_pub = rospy.Publisher('/motion/cmd_vel', Twist, queue_size=5)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/yaw', Float64, queue_size=10)
        self.tfb = tf2_ros.TransformBroadcaster()
        self.pose = [0.0, 0.0, 0.0]
        self.last_fb = None
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=5)
        rospy.Subscriber('/motion/motionFB', Twist, self.on_fb, queue_size=20)
        if inertia is not None:
            rospy.Subscriber('/imu_3d', inertia, self.on_imu, queue_size=20)

    def on_cmd(self, m):
        out = Twist()
        clip = lambda v: max(-self.max_percent, min(self.max_percent, v))
        out.linear.y = clip(m.linear.x * self.cmd_scale)      # forward
        out.linear.x = clip(-m.linear.y * self.cmd_scale)     # right = -left
        out.angular.z = m.angular.z * self.cmd_yaw_scale
        self.cmd_pub.publish(out)

    def on_fb(self, m):
        now = rospy.Time.now()
        vx = m.linear.y * self.fb_scale                        # forward
        vy = -m.linear.x * self.fb_scale                       # left
        if self.last_fb is not None:
            dt = (now - self.last_fb).to_sec()
            if 0 < dt < 0.5:
                x, y, th = self.pose
                c, s = math.cos(th), math.sin(th)
                self.pose = [x + (c * vx - s * vy) * dt, y + (s * vx + c * vy) * dt, th]
        self.last_fb = now
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = now, 'odom', 'base_footprint'
        od.pose.pose.position.x, od.pose.pose.position.y = self.pose[0], self.pose[1]
        od.pose.pose.orientation.w = 1.0
        od.twist.twist.linear.x, od.twist.twist.linear.y = vx, vy
        self.odom_pub.publish(od)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = od.header, 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = self.pose[0], self.pose[1]
        tf.transform.rotation.w = 1.0
        self.tfb.sendTransform(tf)

    def on_imu(self, m):
        self.imu_pub.publish(Float64(wrap(self.imu_sign * m.yaw)))


if __name__ == '__main__':
    rospy.init_node('motion_adapter')
    MotionAdapter()
    rospy.spin()
