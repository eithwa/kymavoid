#!/usr/bin/env python3
"""Virtual panoramic camera: publishes what vision/black_item would publish.

/vision/BlackRealDis and /vision/redRealDis (std_msgs/Int32MultiArray, 120 x cm, 999 = none),
from the true robot pose and the obstacle layout, through avoid_sim.omni (rendered masks +
black_item's own scan). Configurable imperfections: mirror-centre calibration error, pixel
noise, dropouts, latency.
"""
import collections
import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int32MultiArray

from avoid_sim.geometry import Field
from avoid_sim.omni import MaskRenderer, OmniModel, Scanner


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class VirtualCamera:
    def __init__(self):
        p = lambda k, d: rospy.get_param('~' + k, d)
        self.model = OmniModel(rospy.get_param('/camera'))
        self.f = Field.from_params(rospy.get_param('/field'), rospy.get_param('/obstacles'))
        err = p('center_error_px', [0.0, 0.0])                         # true mirror centre - configured
        true_c = (self.model.cx + err[0], self.model.cy + err[1])
        self.renderer = MaskRenderer(self.model, self.f, true_center=true_c)
        self.scanner = Scanner(self.model)                              # black_item uses the configured centre
        self.pixel_noise = p('pixel_noise_std', 0.5)
        self.dropout = p('dropout', 0.01)
        self.latency = p('latency', 0.03)
        self.rng = np.random.default_rng(p('seed', 0) or None)
        self.history = collections.deque(maxlen=200)
        self.obstacles = []
        self.black_pub = rospy.Publisher('/vision/BlackRealDis', Int32MultiArray, queue_size=1)
        self.red_pub = rospy.Publisher('/vision/redRealDis', Int32MultiArray, queue_size=1)
        rospy.Subscriber('/avoid_sim/layout', PoseArray, self.on_layout)
        # reseeded on every field reset, so a seed reproduces the same camera noise
        rospy.Subscriber('/avoid_sim/reset_event', Header,
                         lambda ev: setattr(self, 'rng', np.random.default_rng(int(ev.seq) * 104729 + 2)))
        rospy.Subscriber('/sim/odom_true', Odometry, self.on_truth, queue_size=50)
        rospy.Timer(rospy.Duration(1.0 / p('rate', 30.0)), self.tick)

    def on_layout(self, msg):
        self.obstacles = [(q.position.x, q.position.y) for q in msg.poses]
        self.history.clear()                                            # the robot was teleported

    def on_truth(self, msg):
        q = msg.pose.pose
        self.history.append((msg.header.stamp.to_sec(), q.position.x, q.position.y, yaw_of(q.orientation)))

    def pose_at(self, t):
        for s in reversed(self.history):
            if s[0] <= t:
                return s[1:]
        return self.history[0][1:] if self.history else None

    def tick(self, _):
        pose = self.pose_at(rospy.Time.now().to_sec() - self.latency)
        if pose is None:
            return
        black, red = self.renderer.render(*pose, self.obstacles)
        self.black_pub.publish(Int32MultiArray(data=self.scanner.scan(
            black, self.pixel_noise, self.dropout, self.rng).tolist()))
        self.red_pub.publish(Int32MultiArray(data=self.scanner.scan(
            red, self.pixel_noise, self.dropout, self.rng).tolist()))


if __name__ == '__main__':
    rospy.init_node('virtual_camera')
    VirtualCamera()
    rospy.spin()
