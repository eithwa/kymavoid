#!/usr/bin/env python3
"""Virtual panoramic camera: publishes what vision/black_item would publish.

/vision/BlackRealDis and /vision/redRealDis (std_msgs/Int32MultiArray, 120 x cm, 999 = none),
from the true robot pose and the obstacle layout, through avoid_sim.omni (rendered masks +
black_item's own scan). Configurable imperfections: mirror-centre calibration error, pixel
noise, dropouts, latency.

/camera/black and /camera/red (sensor_msgs/Image, rgb8): the masks with black_item's scan drawn
on them, the same topics black_item publishes, for the web UI (through mjpeg_server). Drawn only
while someone subscribes, at most image_rate Hz.
"""
import collections
import math
import queue
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
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
        # published exactly this long after the moment the scan shows (camera_adapter stamps it so)
        self.latency = p('latency', rospy.get_param('/camera/latency', 0.03))
        self.due = queue.Queue()              # rendered scans waiting for their publish time
        self.render_ms = collections.deque(maxlen=100)
        self.rng = np.random.default_rng(p('seed', 0) or None)
        self.history = collections.deque(maxlen=200)
        self.lock = threading.Lock()          # history is filled by the odometry callback, read by the timer
        self.obstacles = []
        self.black_pub = rospy.Publisher('/vision/BlackRealDis', Int32MultiArray, queue_size=1)
        self.red_pub = rospy.Publisher('/vision/redRealDis', Int32MultiArray, queue_size=1)
        self.black_img_pub = rospy.Publisher('/camera/black', Image, queue_size=1)
        self.red_img_pub = rospy.Publisher('/camera/red', Image, queue_size=1)
        self.image_period = 1.0 / p('image_rate', 10.0)
        self.last_image = 0.0
        self.image_layers_cache = self.image_layers()
        rospy.Subscriber('/avoid_sim/layout', PoseArray, self.on_layout)
        # reseeded on every field reset, so a seed reproduces the same camera noise
        rospy.Subscriber('/avoid_sim/reset_event', Header,
                         lambda ev: setattr(self, 'rng', np.random.default_rng(int(ev.seq) * 104729 + 2)))
        rospy.Subscriber('/sim/odom_true', Odometry, self.on_truth, queue_size=50)
        rospy.Timer(rospy.Duration(1.0 / p('rate', 30.0)), self.tick)
        threading.Thread(target=self.deliver, daemon=True).start()

    def on_layout(self, msg):
        with self.lock:
            self.obstacles = [(q.position.x, q.position.y, yaw_of(q.orientation)) for q in msg.poses]
            self.history.clear()                                        # the robot was teleported
        while not self.due.empty():                                     # scans of the old field
            try:
                self.due.get_nowait()
            except queue.Empty:
                break

    def on_truth(self, msg):
        q = msg.pose.pose
        with self.lock:
            self.history.append((msg.header.stamp.to_sec(), q.position.x, q.position.y, yaw_of(q.orientation)))

    def tick(self, _):
        # an exception would end the rospy timer for good: the camera would go silent while the node
        # still answers pings (the web UI then shows the camera data stale, the node alive)
        try:
            self.publish()
        except Exception as e:
            rospy.logerr_throttle(5, 'virtual_camera: %s' % e)

    # The image is taken at the latest true pose and published `latency` later, as black_item does.
    # Before, the pose was picked first (the last sample before now - latency) and the scan published
    # when rendering was done: the real latency was the configured one + 10 ms + the rendering time,
    # 40-55 ms more than camera_adapter assumed (measured from wall hits; more under CPU load). That
    # made the robot's x estimate lag (wall fits) and put faces ahead too far in its obstacle memory.
    def publish(self):
        with self.lock:
            if not self.history:
                return
            t_image, x, y, yaw = self.history[-1]
            obstacles = list(self.obstacles)
        t0 = time.perf_counter()
        black, red = self.renderer.render(x, y, yaw, obstacles)
        scans = (Int32MultiArray(data=self.scanner.scan(black, self.pixel_noise, self.dropout, self.rng).tolist()),
                 Int32MultiArray(data=self.scanner.scan(red, self.pixel_noise, self.dropout, self.rng).tolist()))
        self.render_ms.append((time.perf_counter() - t0) * 1000)
        self.due.put((t_image + self.latency, scans, black, red))

    def deliver(self):
        while not rospy.is_shutdown():
            try:
                due, (black_scan, red_scan), black, red = self.due.get(timeout=0.5)
            except queue.Empty:
                continue
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() < due:
                time.sleep(0.001)
            late = rospy.Time.now().to_sec() - due
            if late > 0.01:
                rospy.logwarn_throttle(10, 'virtual_camera: scans %.0f ms late (rendering takes %.0f ms, the camera '
                                       'latency is %.0f ms): the robot sees the world later than it assumes; '
                                       'CPU overloaded?' % (late * 1000, np.median(self.render_ms), self.latency * 1000))
            self.black_pub.publish(black_scan)
            self.red_pub.publish(red_scan)
            self.publish_images(black, red)

    def image_layers(self):
        """What black_item marks on its frames, around the configured mirror centre. Under the
        masks: the ring it scans (the floor it can see). Over them: the inner disk it skips (the
        robot), the outer circle and scan line 0 (straight ahead)."""
        m = self.model
        v, u = np.mgrid[0:m.h, 0:m.w]
        r = np.hypot(u - m.cx, v - m.cy)
        under = np.zeros((m.h, m.w, 3), np.uint8)
        under[(r >= m.inner) & (r <= m.outer)] = (28, 30, 34)
        over = np.zeros((m.h, m.w, 3), np.uint8)
        over[r < m.inner] = (70, 70, 70)
        over[np.abs(r - m.outer) <= 1.0] = (76, 141, 255)
        over[self.scanner.iy[0], self.scanner.ix[0]] = (63, 185, 80)
        return under, over, over.any(axis=2)

    @staticmethod
    def thicken(mask):
        """One pixel wider each way: the red line is ~1 px in the image, hidden under the scan dots."""
        out = mask.copy()
        out[1:] |= mask[:-1]
        out[:-1] |= mask[1:]
        out[:, 1:] |= mask[:, :-1]
        out[:, :-1] |= mask[:, 1:]
        return out

    def publish_images(self, black, red):
        t = time.monotonic()
        if t - self.last_image < self.image_period:
            return
        todo = [(pub, mask, colour, hit) for pub, mask, colour, hit in (
            (self.black_img_pub, black, (235, 235, 235), (255, 152, 0)),
            (self.red_img_pub, red, (229, 57, 53), (242, 201, 76))) if pub.get_num_connections()]
        if not todo:
            return
        self.last_image = t
        h, w = self.model.h, self.model.w
        under, over, over_mask = self.image_layers_cache
        stamp = rospy.Time.now()
        for pub, mask, colour, hit in todo:
            img = under.copy()
            img[self.thicken(mask) if mask is red else mask] = colour
            img[over_mask] = over[over_mask]
            hx, hy = self.scanner.hit_pixels(mask)                     # what the scan found: 3x3 dots
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    img[np.clip(hy + dy, 0, h - 1), np.clip(hx + dx, 0, w - 1)] = hit
            pub.publish(Image(header=Header(stamp=stamp, frame_id='camera'), height=h, width=w,
                              encoding='rgb8', is_bigendian=0, step=w * 3, data=img.tobytes()))

if __name__ == '__main__':
    rospy.init_node('virtual_camera')
    VirtualCamera()
    rospy.spin()
