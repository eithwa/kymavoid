#!/usr/bin/env python3
"""Spawns the obstacles and serves /avoid_sim/reset and /avoid_sim/save_case.

reset: pause physics -> robot back to the start pose (optionally with a placement error, or an
explicit pose) -> obstacles re-placed (random to spec with a seed, the original 'classic' layout,
or explicit centres from a recorded case) -> publish the layout (latched) and an event so the other
sim nodes can reset -> unpause.
save_case: append the current field to a case file in avoid_sim/cases (avoid_sim.cases).
"""
import math
import os

import numpy as np
import rospkg
import rospy
import yaml
from avoid_msgs.srv import ResetField, ResetFieldResponse, SaveCase, SaveCaseResponse
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetWorldProperties, SetModelState, SpawnModel
from geometry_msgs.msg import Pose, Pose2D, PoseArray
from std_msgs.msg import Header
from std_srvs.srv import Empty

from avoid_sim import cases
from avoid_sim.geometry import CLASSIC_LAYOUT, Field, layout_violations, load_footprint, sample_layout, ConfigSpace

ROBOT = 'avoid_robot'

BOX_SDF = """<sdf version="1.6"><model name="{name}"><static>true</static><link name="link">
<pose>0 0 {hz} 0 0 0</pose>
<collision name="c"><geometry><box><size>{s} {s} {h}</size></box></geometry></collision>
<visual name="v"><geometry><box><size>{s} {s} {h}</size></box></geometry>
<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
</link></model></sdf>"""


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class FieldManager:
    def __init__(self):
        self.f = Field.from_params(rospy.get_param('/field'), rospy.get_param('/obstacles'))
        fp = rospy.get_param('~footprint', os.path.join(
            rospkg.RosPack().get_path('robot6th_description'), 'config', 'robot6th_footprint.yaml'))
        self.footprint = load_footprint(fp)
        self.names = ['obstacle_%d' % (i + 1) for i in range(self.f.count)]
        self.layout_pub = rospy.Publisher('/avoid_sim/layout', PoseArray, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher('/avoid_sim/reset_event', Header, queue_size=1, latch=True)
        for s in ('/gazebo/spawn_sdf_model', '/gazebo/set_model_state', '/gazebo/get_world_properties',
                  '/gazebo/pause_physics', '/gazebo/unpause_physics'):
            rospy.wait_for_service(s)
        self.spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.spawn_obstacles()
        deadline = rospy.Time.now() + rospy.Duration(60.0)
        while not rospy.is_shutdown() and ROBOT not in self.world().model_names:
            if rospy.Time.now() > deadline:
                raise RuntimeError('robot model %s never appeared' % ROBOT)
            rospy.sleep(0.2)
        self.current = None
        rospy.Service('/avoid_sim/reset', ResetField, self.handle_reset)
        rospy.Service('/avoid_sim/save_case', SaveCase, self.handle_save)
        layout = rospy.get_param('~layout', 'random')
        seed = int(rospy.get_param('~seed', -1))
        res = self.reset(seed, layout, 0.0, 0.0)
        rospy.loginfo('field_manager ready: %s', res.message)

    def spawn_obstacles(self):
        sdf_park = BOX_SDF.format(name='{name}', s=self.f.size, h=self.f.height, hz=self.f.height / 2)
        existing = set(self.world().model_names)
        for k, n in enumerate(self.names):
            if n in existing:
                continue
            p = Pose()
            p.position.x, p.position.y = 10.0 + k, 10.0                     # parked until the first reset
            p.orientation.w = 1.0
            ok = self.spawn(n, sdf_park.format(name=n), '', p, 'world')
            if not ok.success:
                raise RuntimeError('spawning %s failed: %s' % (n, ok.status_message))

    def place(self, name, x, y, yaw=0.0, z=0.0):
        ms = ModelState()
        ms.model_name, ms.reference_frame = name, 'world'
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = x, y, z
        (ms.pose.orientation.x, ms.pose.orientation.y,
         ms.pose.orientation.z, ms.pose.orientation.w) = yaw_quat(yaw)
        r = self.set_state(ms)
        if not r.success:
            raise RuntimeError('set_model_state %s failed: %s' % (name, r.status_message))

    def reset(self, seed, layout_name, err_xy, err_yaw, obstacles=None, start=None):
        if seed < 0:
            seed = int(np.random.SeedSequence().entropy % (2 ** 31))
        rng = np.random.default_rng(seed)
        attempts = 0
        if obstacles:
            if len(obstacles) > self.f.count:
                raise ValueError('%d obstacles given, only %d spawned (/obstacles/count)' % (len(obstacles), self.f.count))
            layout = [(o.x, o.y) for o in obstacles]
            layout_name = layout_name or 'case'
        elif layout_name == 'classic':
            layout = list(CLASSIC_LAYOUT[:self.f.count])
        else:
            layout, attempts = sample_layout(self.f, self.footprint, rng)
        if start:
            sx, sy, syaw = start[0].x, start[0].y, start[0].theta
        else:
            sx = self.f.start_x + (rng.normal(0, err_xy) if err_xy > 0 else 0.0)
            sy = self.f.start_y + (rng.normal(0, err_xy) if err_xy > 0 else 0.0)
            syaw = self.f.start_yaw + (rng.normal(0, err_yaw) if err_yaw > 0 else 0.0)
        parked = [(10.0 + k, 10.0) for k in range(len(layout), self.f.count)]   # unused boxes stay out of the field

        self.pause()
        try:
            for n, (x, y) in zip(self.names, layout + parked):
                self.place(n, x, y)
            self.place(ROBOT, sx, sy, syaw, 0.01)
        finally:
            self.unpause()

        pa = PoseArray()
        pa.header.stamp, pa.header.frame_id = rospy.Time.now(), 'field'
        for x, y in layout:
            p = Pose()
            p.position.x, p.position.y, p.orientation.w = x, y, 1.0
            pa.poses.append(p)
        self.layout_pub.publish(pa)
        ev = Header()
        ev.stamp, ev.frame_id, ev.seq = rospy.Time.now(), layout_name, seed
        self.event_pub.publish(ev)

        self.current = dict(seed=seed, layout=layout_name, start=(sx, sy, syaw), obstacles=list(layout))
        res = ResetFieldResponse()
        res.success, res.seed, res.attempts = True, seed, attempts
        res.start = Pose2D(sx, sy, syaw)
        res.obstacles = [Pose2D(x, y, 0.0) for x, y in layout]
        bad = layout_violations(layout, self.f)
        ok = ConfigSpace(self.f, self.footprint).reachable(layout, (sx, sy))
        res.message = '%s layout, seed %d, %d rejected, gap violations %d, reachable %s' % (
            layout_name, seed, attempts, len(bad), ok)
        return res

    def handle_reset(self, req):
        try:
            layout = req.layout or ('' if req.use_obstacles else 'random')
            return self.reset(req.seed, layout, req.start_error_xy, req.start_error_yaw,
                              req.use_obstacles, req.use_start)
        except Exception as e:   # report instead of killing the service
            rospy.logerr('reset failed: %s', e)
            return ResetFieldResponse(success=False, message=str(e))

    def handle_save(self, req):
        try:
            if self.current is None:
                raise RuntimeError('no field placed yet')
            expect = yaml.safe_load(req.expect) if req.expect.strip() else None
            if expect is not None and not isinstance(expect, dict):
                raise ValueError('expect must be a mapping, e.g. {result: success, max_time: 8.0}')
            c = self.current
            case = cases.make(c['seed'], c['layout'], c['start'], c['obstacles'], req.sensor, req.note.strip(),
                              req.observed.strip(), expect, req.status.strip() or 'open', req.id.strip())
            path = cases.resolve(req.file.strip() or 'recorded.yaml')
            cid = cases.append(path, case)
            rospy.loginfo('saved case %s to %s', cid, path)
            return SaveCaseResponse(success=True, message='saved %s' % cid, id=cid, path=os.path.realpath(path))
        except Exception as e:
            rospy.logerr('save_case failed: %s', e)
            return SaveCaseResponse(success=False, message=str(e))


if __name__ == '__main__':
    rospy.init_node('field_manager')
    FieldManager()
    rospy.spin()
