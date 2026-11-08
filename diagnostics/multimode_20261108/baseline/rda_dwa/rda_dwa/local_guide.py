"""Bounded local Hybrid-A* guidance from observed confirmed-static geometry.

This is not a global map, a complete planner, or an executable command sequence.
Kinematic arcs choose a persistent topology; production DWA must independently
check its rate-limited trajectory, current dynamics, reaction and braking.
"""
from __future__ import annotations

import heapq
import math
import time
import numpy as np


class LocalStaticGuide:
    def __init__(self, collision_checker, resolution=.3, primitive_length=.9,
                 budget_s=.03, max_expansions=400, replan_interval=2.0):
        self.cc = collision_checker
        self.resolution = float(resolution)
        self.primitive_length = float(primitive_length)
        self.budget_s = float(budget_s)
        self.max_expansions = int(max_expansions)
        self.replan_interval = float(replan_interval)
        self.heading_bins = 24
        self.path = []
        self.cursor = 0
        self.status = 'idle'
        self.expansions = 0
        self.elapsed_s = 0.0
        self._since = float('-inf')
        self._goal = None
        self._retry_after = float('-inf')

    def reset(self):
        self.path = []
        self.cursor = 0
        self._goal = None
        self._retry_after = float('-inf')
        self.status = 'idle'

    def _key(self, pose, origin):
        return (int(round((pose[0]-origin[0])/self.resolution)),
                int(round((pose[1]-origin[1])/self.resolution)),
                int(round((pose[2] % (2*math.pi))/(2*math.pi)*self.heading_bins)) % self.heading_bins)

    def _arc(self, pose, direction, steering):
        ap = self.cc.ack
        count = 6
        ds = direction*self.primitive_length/count
        x, y, yaw = pose
        offset = ap.reference_rear_offset
        x += offset*math.cos(yaw)
        y += offset*math.sin(yaw)
        result = [pose]
        for _ in range(count):
            x += ds*math.cos(yaw)
            y += ds*math.sin(yaw)
            yaw += ds*math.tan(steering)/ap.wheelbase
            result.append((x-offset*math.cos(yaw), y-offset*math.sin(yaw), yaw))
        return np.asarray(result)

    def update(self, robot, goal, obstacles, now, margin=.2, budget_s=None):
        """Return persistent map waypoint, or None with an explicit status.

        Unknown or moving objects never become static obstacles. They remain in
        the downstream DWA input; guidance grants no dynamic safety exemption.
        New goals invalidate all memory. Reused geometry is guidance only.
        """
        goal = tuple(float(x) for x in goal[:2])
        if self._goal is not None and math.hypot(goal[0]-self._goal[0],goal[1]-self._goal[1]) > .3:
            self.reset()
        static = [o for o in obstacles if getattr(o, 'obstacle_type', '') in ('block','static_large')
                  and float(o.speed) <= .05 and not getattr(o, 'velocity_unknown', False)]
        if not static:
            self.status = 'no_static_evidence'
            self.path = []
            return None
        if self.path and now-self._since <= self.replan_interval:
            while self.cursor < len(self.path)-1 and math.hypot(robot.x-self.path[self.cursor][0],robot.y-self.path[self.cursor][1]) < .45:
                self.cursor += 1
            if self.cursor == len(self.path)-1 and math.hypot(robot.x-goal[0],robot.y-goal[1]) < .5:
                self.status = 'complete'
                self.path = []
                return None
            self.status = 'path'
            return self.path[self.cursor][:2]
        if now < self._retry_after:
            return None
        started = time.perf_counter()
        deadline = started+max(0.,self.budget_s if budget_s is None else float(budget_s))
        origin = (float(robot.x),float(robot.y))
        source = (float(robot.x),float(robot.y),float(robot.theta))
        # Source and goal must lie within the finite observed local search patch.
        radius = max(4.,min(8.,math.hypot(goal[0]-source[0],goal[1]-source[1])+2.))
        if math.hypot(goal[0]-source[0],goal[1]-source[1]) > radius:
            self.status = 'goal_outside_patch'
            self._retry_after = now+self.replan_interval
            return None
        source_key = self._key(source,origin)+(0,)
        states = {source_key:source}
        parents = {}
        g = {source_key:0.}
        queue = [(math.hypot(goal[0]-source[0],goal[1]-source[1]),0,source_key)]
        counter = 0
        self.expansions = 0
        terminal = None
        old_scene = getattr(self.cc,'_scene_obstacles',None)
        old_geometry = getattr(self.cc,'_scene_geometry',None)
        self.cc.begin_scene(static)
        try:
            while queue and self.expansions < self.max_expansions:
                if time.perf_counter() >= deadline:
                    self.status = 'timeout'
                    break
                _,_,key = heapq.heappop(queue)
                pose = states[key]
                if math.hypot(pose[0]-goal[0],pose[1]-goal[1]) <= .55:
                    terminal = key
                    self.status = 'path'
                    break
                self.expansions += 1
                for direction in (1,-1):
                    for steering in (0.,-self.cc.ack.delta_max,self.cc.ack.delta_max):
                        if time.perf_counter() >= deadline:
                            self.status = 'timeout'
                            break
                        arc = self._arc(pose,direction,steering)
                        end = tuple(arc[-1])
                        if abs(end[0]-origin[0])>radius or abs(end[1]-origin[1])>radius:
                            continue
                        if not self.cc.time_collision_free(arc,np.arange(len(arc))*.1,static,margin,0.):
                            continue
                        successor = self._key(end,origin)+(direction,)
                        transition = self.primitive_length*(1. if direction>0 else 1.3)
                        transition += .15*abs(steering)/max(self.cc.ack.delta_max,1e-9)
                        if key[-1] and key[-1] != direction:
                            transition += .8  # stop and reversal preference, not elapsed time
                        cost = g[key]+transition
                        if cost >= g.get(successor,float('inf'))-1e-9:
                            continue
                        g[successor] = cost
                        states[successor] = end
                        parents[successor] = key
                        counter += 1
                        h = math.hypot(goal[0]-end[0],goal[1]-end[1])
                        heapq.heappush(queue,(cost+h,counter,successor))
            if terminal is not None:
                path = []
                key = terminal
                while key != source_key:
                    path.append(states[key]); key = parents[key]
                self.path = list(reversed(path))
                self.cursor = 0
                self._goal = goal
                self._since = now
                return self.path[0][:2] if self.path else None
            self.path = []
            if self.status != 'timeout':
                self.status = 'expansion_limit' if queue else 'no_path'
            self._retry_after = now+self.replan_interval
            return None
        finally:
            self.elapsed_s = time.perf_counter()-started
            self.cc._scene_obstacles,self.cc._scene_geometry = old_scene,old_geometry
