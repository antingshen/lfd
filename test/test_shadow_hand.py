#!/usr/bin/env python

from __future__ import division

import openravepy
import unittest

import numpy as np
import trajoptpy
from time import sleep

from lfd.environment.simulation import DynamicSimulation
from lfd.environment.simulation_object import XmlSimulationObject, BoxSimulationObject, CylinderSimulationObject, RopeSimulationObject
from lfd.environment import sim_util


class TestShadowHand():

    def __init__(self):
        pass

    def setUp(self):
        table_height = -0.14
        helix_ang0 = 0
        helix_ang1 = 4*np.pi
        helix_radius = .2
        helix_center = np.r_[.6, 0]
        helix_height0 = table_height + .15
        helix_height1 = table_height + .15 + .3
        helix_length = np.linalg.norm(np.r_[(helix_ang1 - helix_ang0) * helix_radius, helix_height1 - helix_height0])
        num = np.round(helix_length/.02)
        helix_angs = np.linspace(helix_ang0, helix_ang1, num)
        helix_heights = np.linspace(helix_height0, helix_height1, num)
        init_rope_nodes = np.c_[helix_center + helix_radius * np.c_[np.cos(helix_angs), np.sin(helix_angs)], helix_heights]
        rope_params = sim_util.RopeParams()

        cyl_radius = 0.018
        cyl_height = 0.3
        cyl_pos0 = np.r_[.6, helix_radius, table_height + .15]
        cyl_pos1 = np.r_[.6, -helix_radius, table_height + .15]


        self.sim = ShadowHandDynamicSimulation()
        self.sim.add_objects([BoxSimulationObject("table", [1, 0, table_height-.1], [.85, .85, .1], dynamic=False)])

        sim_objs = []
        sim_objs.append(CylinderSimulationObject("cyl0", cyl_pos0, cyl_radius, cyl_height, dynamic=True))
        sim_objs.append(CylinderSimulationObject("cyl1", cyl_pos1, cyl_radius, cyl_height, dynamic=True))
        self.sim.add_objects(sim_objs)

        for i in range(2):
            bt_cyl = self.sim.bt_env.GetObjectByName('cyl%d' % i)
            transform = openravepy.matrixFromAxisAngle(np.array([np.pi/2, 0, 0]))
            transform[:3, 3] = bt_cyl.GetTransform()[:3, 3]
            bt_cyl.SetTransform(transform)  # SetTransform needs to be used in the Bullet object, not the openrave body
        self.sim.update()

        self.sim.add_objects([XmlSimulationObject("robots/shadow-hand.zae", dynamic=False)])

        transform = openravepy.matrixFromAxisAngle(np.array([np.pi, 0, 0]))
        transform[:3, 3] = [0,0,0]
        self.sim.robot.SetTransform(transform)


class ShadowHandDynamicSimulation(DynamicSimulation):
    def __init__(self, env=None):
        super(ShadowHandDynamicSimulation, self).__init__(env=env)

    FINGER_CLOSED_DOF = 1.2
    FINGER_OPEN_DOF = 0
    FINGERS = [1,2,3]
    def close_fingers(self, target_vals=None, max_vel=.02):
        # generate gripper finger trajectory

        joint_inds = [self.robot.GetJoint("JF%d_2" % finger_number).GetDOFIndex() for finger_number in self.FINGERS]
        start_vals = self.robot.GetDOFValues(joint_inds)
        if target_vals is None:
            target_vals = [self.FINGER_CLOSED_DOF] * len(self.FINGERS)
        max_dist = max(map(lambda x: abs(x[1] - x[0]), zip(start_vals, target_vals)))
        joint_trajs = [np.linspace(start_val, target_val, np.ceil(max_dist / max_vel)) for start_val, target_val in zip(start_vals, target_vals)]

        # execute gripper finger trajectory
        for vals in zip(*joint_trajs):
            self.robot.SetDOFValues(vals, joint_inds)
            self.step()
            self.viewer.Step()

    def open_fingers(self, **kwargs):
        self.close_fingers(target_vals=[self.FINGER_OPEN_DOF]*len(self.FINGERS), **kwargs)

    def set_hand_location(self, location, max_vel=.002):
        transform = self.robot.GetTransform()
        starts = transform[:3, 3]
        max_dist = max(map(lambda x: abs(x[1] - x[0]), zip(starts, location)))
        traj = [np.linspace(start, target, np.ceil(max_dist / max_vel)) for start, target in zip(starts, location)]

        for vals in zip(*traj):
            transform[:3, 3] = vals
            self.robot.SetTransform(transform)
            self.step()
            self.viewer.Step()


T = TestShadowHand()
T.setUp()
robot = T.sim.robot
sim = T.sim
sim.create_viewer()
viewer = sim.viewer

sim.set_hand_location([0.51, 0.155, 0])
sim.close_fingers()
sim.set_hand_location([0.51, 0, 0])
sim.open_fingers()
sim.settle()
