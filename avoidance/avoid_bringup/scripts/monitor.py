#!/usr/bin/env python3
"""Terminal status monitor (tmux window 'monitor'): nav_node state, input ages and topic rates."""
import os
import sys
import threading
import time
from collections import deque

import rospy
from rospy import AnyMsg

from avoid_msgs.msg import Status

TOPICS = ['/scan', '/boundary', '/odom', '/imu/yaw', '/cmd_vel', '/vision/BlackRealDis', '/scan_0',
          '/scan_1', '/scan_2', '/sim/odom_true']
COLOR = sys.stdout.isatty()


def c(code, s):
    return '\033[%sm%s\033[0m' % (code, s) if COLOR else s


class Monitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.status, self.status_t = None, 0.0
        self.stamps = {t: deque(maxlen=200) for t in TOPICS}
        rospy.Subscriber('/avoid/status', Status, self.on_status, queue_size=1)
        for t in TOPICS:
            rospy.Subscriber(t, AnyMsg, self.on_msg, callback_args=t, queue_size=1)

    def on_status(self, m):
        with self.lock:
            self.status, self.status_t = m, time.time()

    def on_msg(self, _, topic):
        with self.lock:
            self.stamps[topic].append(time.time())

    def rate(self, topic, now):
        s = [t for t in self.stamps[topic] if now - t <= 2.0]
        if len(s) < 2:
            return None, (now - self.stamps[topic][-1]) if self.stamps[topic] else None
        return (len(s) - 1) / (s[-1] - s[0]) if s[-1] > s[0] else None, now - s[-1]

    def draw(self):
        now = time.time()
        out = [c('1', 'avoid monitor') + '   ROS_MASTER_URI=%s   %s' % (
            os.environ.get('ROS_MASTER_URI', '?'), time.strftime('%H:%M:%S')), '']
        with self.lock:
            st, age = self.status, now - self.status_t
            if st is None:
                out.append(c('31', 'nav_node: no /avoid/status yet'))
            else:
                alive = c('32', 'alive') if age < 1.0 else c('31', 'SILENT %.1f s' % age)
                colour = {'IDLE': '37', 'MAPPING': '36', 'RUNNING': '32', 'FINISHED': '34'}.get(st.state_name, '31')
                out.append('nav_node %s   state %s   %s   [%s]' % (alive, c('1;' + colour, st.state_name),
                                                                 st.reason, st.sensor_config))
                out.append('pose  x %+.3f  y %+.3f  th %+6.1f deg   std %.3f %.3f %.1f deg   t %.1f s' % (
                    st.pose.x, st.pose.y, st.pose.theta * 57.2958, st.pose_std_x, st.pose_std_y,
                    st.pose_std_theta * 57.2958, st.elapsed))
                out.append('cmd   vx %+.2f  vy %+.2f  wz %+.2f' % (st.cmd.linear.x, st.cmd.linear.y, st.cmd.angular.z))
                ages = [('obstacles', st.age_obstacles), ('boundary', st.age_boundary), ('odom', st.age_odom),
                        ('imu', st.age_imu)]
                out.append('ages  ' + '   '.join(
                    '%s %s' % (n, c('31', 'none') if a < 0 else (c('32', '%.2f' % a) if a < 0.3 else c('31', '%.2f' % a)))
                    for n, a in ages))
            out += ['', '%-24s %8s %8s' % ('topic', 'Hz', 'last [s]')]
            for t in TOPICS:
                hz, last = self.rate(t, now)
                if last is None:
                    continue
                txt = '%-24s %8s %8.2f' % (t, '%.1f' % hz if hz else '-', last)
                out.append(c('32', txt) if last < 0.5 else c('31', txt))
        sys.stdout.write('\033[H\033[2J' + '\n'.join(out) + '\n')
        sys.stdout.flush()


if __name__ == '__main__':
    rospy.init_node('avoid_monitor', anonymous=True, disable_signals=False)
    m = Monitor()
    while not rospy.is_shutdown():
        m.draw()
        time.sleep(0.5)
