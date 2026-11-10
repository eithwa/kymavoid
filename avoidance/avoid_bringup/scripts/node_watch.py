#!/usr/bin/env python3
"""Which nodes are really alive, for the web UI.

The master's node list is not enough: a node that crashes (segfault, kill -9) never unregisters
and stays listed. Twice a second this pings every registered node's XML-RPC API (as `rosnode ping`
does) and publishes the result as JSON on /avoid/nodes (latched):
  {"stamp": <wall time>, "nodes": {"/nav_node": true, "/virtual_camera": false, ...}}
"""
import json
import socket
import time
import xmlrpc.client
from concurrent.futures import ThreadPoolExecutor

import rosgraph
import rospy
from std_msgs.msg import String


class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        conn = super().make_connection(host)
        conn.timeout = self.timeout
        return conn


def ping(uri, timeout):
    try:
        code, _, _ = xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport(timeout)).getPid('/node_watch')
        return code == 1
    except (socket.error, OSError, xmlrpc.client.Error):
        return False


def main():
    rospy.init_node('node_watch')
    period = rospy.get_param('~period', 0.5)
    timeout = rospy.get_param('~timeout', 0.5)
    pub = rospy.Publisher('/avoid/nodes', String, queue_size=1, latch=True)
    master = rosgraph.Master(rospy.get_name())
    pool = ThreadPoolExecutor(max_workers=8)
    while not rospy.is_shutdown():
        t0 = time.time()
        try:
            pubs, subs, srvs = master.getSystemState()
            names = sorted({n for group in (pubs, subs, srvs) for _, ns in group for n in ns})
            uris = {}
            for n in names:
                try:
                    uris[n] = master.lookupNode(n)
                except rosgraph.MasterException:
                    pass                                  # unregistered meanwhile
            names = sorted(uris)
            alive = dict(zip(names, pool.map(lambda n: ping(uris[n], timeout), names)))
            pub.publish(String(json.dumps({'stamp': time.time(), 'nodes': alive})))
        except (socket.error, OSError, rosgraph.MasterException) as e:
            rospy.logwarn_throttle(10, 'node_watch: %s' % e)
        time.sleep(max(0.05, period - (time.time() - t0)))   # wall clock: works with sim time too


if __name__ == '__main__':
    main()
