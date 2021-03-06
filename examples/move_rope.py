#!/usr/bin/env python

from __future__ import division

import numpy as np
import trajoptpy

from lfd.rapprentice import planning, resampling
import lfd.rapprentice.math_utils as mu
from lfd.environment.simulation import DynamicSimulationRobotWorld
from lfd.environment.simulation_object import XmlSimulationObject, BoxSimulationObject, CylinderSimulationObject, RopeSimulationObject
from lfd.environment import sim_util


table_height = 0.77
cyl_radius = 0.025
cyl_height = 0.3
cyl_pos0 = np.r_[.7, -.15, table_height+cyl_height/2]
cyl_pos1 = np.r_[.7, .15, table_height+cyl_height/2]
cyl_pos2 = np.r_[.4, -.15, table_height+cyl_height/2]
sample_grid = np.array(np.meshgrid(np.linspace(0,1,5), np.linspace(0,1,5))).T.reshape((-1,2))
cyl_positions = cyl_pos0 + sample_grid[:,0][:,None].dot(cyl_pos1[None,:] - cyl_pos0[None,:]) + sample_grid[:,1][:,None].dot(cyl_pos2[None,:] - cyl_pos0[None,:])
rope_pos0 = np.r_[.2, -.2, table_height+0.006]
rope_pos1 = np.r_[.8, -.2, table_height+0.006]
rope_pos2 = np.r_[.8, .2, table_height+0.006]
rope_pos3 = np.r_[.2, .2, table_height+0.006]
init_rope_nodes = np.r_[mu.linspace2d(rope_pos0, rope_pos1, np.round(np.linalg.norm(rope_pos1 - rope_pos0)/.02)),
                        mu.linspace2d(rope_pos1, rope_pos2, np.round(np.linalg.norm(rope_pos2 - rope_pos1)/.02))[1:,:],
                        mu.linspace2d(rope_pos2, rope_pos3, np.round(np.linalg.norm(rope_pos3 - rope_pos2)/.02))[1:,:]]
rope_params = sim_util.RopeParams()

sim_objs = []
sim_objs.append(XmlSimulationObject("robots/pr2-beta-static.zae", dynamic=False))
sim_objs.append(BoxSimulationObject("table", [1, 0, table_height-.1], [.85, .85, .1], dynamic=False))
# add grid of cylinders
cyl_sim_objs = []
for (i,cyl_pos) in enumerate(cyl_positions):
   cyl_sim_objs.append(CylinderSimulationObject("cyl%i"%i, cyl_pos, cyl_radius, cyl_height, dynamic=True))
sim_objs.extend(cyl_sim_objs)
sim_objs.append(RopeSimulationObject("rope", init_rope_nodes, rope_params))

sim = DynamicSimulationRobotWorld()
sim.add_objects(sim_objs)
sim.robot.SetDOFValues([0.25], [sim.robot.GetJoint('torso_lift_joint').GetJointIndex()])
sim_util.reset_arms_to_side(sim)

# set random colors to cylinders
for sim_obj in cyl_sim_objs:
    color = np.random.random(3)
    for bt_obj in sim_obj.get_bullet_objects():
        for link in bt_obj.GetKinBody().GetLinks():
            for geom in link.GetGeometries():
                geom.SetDiffuseColor(color)

viewer = trajoptpy.GetViewer(sim.env)
camera_matrix = np.array([[ 0,    1, 0,   0],
                          [-1,    0, 0.5, 0],
                          [ 0.5,  0, 1,   0],
                          [ 2.25, 0, 4.5, 1]])
viewer.SetWindowProp(0,0,1500,1500)
viewer.SetCameraManipulatorMatrix(camera_matrix)

lr = 'r'
manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
ee_link_name = "%s_gripper_tool_frame"%lr
ee_link = sim.robot.GetLink(ee_link_name)
n_steps = 10

def get_move_traj(t_start, t_end, R_start, R_end, start_fixed):
    hmat_start = np.r_[np.c_[R_start, t_start], np.c_[0,0,0,1]]
    hmat_end = np.r_[np.c_[R_end, t_end], np.c_[0,0,0,1]]
    new_hmats = np.asarray(resampling.interp_hmats(np.arange(n_steps), np.r_[0, n_steps-1], [hmat_start, hmat_end]))
    dof_vals = sim.robot.GetManipulator(manip_name).GetArmDOFValues()
    old_traj = np.tile(dof_vals, (n_steps,1))
    
    traj, _, _ = planning.plan_follow_traj(sim.robot, manip_name, ee_link, new_hmats, old_traj, start_fixed=start_fixed, beta_rot=10000.0)
    return traj

pick_pos = rope_pos0 + .1 * (rope_pos1 - rope_pos0)
drop_pos = rope_pos3 + .1 * (rope_pos2 - rope_pos3) + np.r_[0, .2, 0]
pick_R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
drop_R = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])
move_height = .2
dof_inds = sim_util.dof_inds_from_name(sim.robot, manip_name)

traj = get_move_traj(pick_pos + np.r_[0,0,move_height], pick_pos, pick_R, pick_R, False)
sim.execute_trajectory((traj, dof_inds))
sim.close_gripper(lr)

traj = get_move_traj(pick_pos, pick_pos + np.r_[0,0,move_height], pick_R, pick_R, True)
sim.execute_trajectory((traj, dof_inds))

traj = get_move_traj(pick_pos + np.r_[0,0,move_height], drop_pos + np.r_[0,0,move_height], pick_R, drop_R, True)
sim.execute_trajectory((traj, dof_inds))

traj = get_move_traj(drop_pos + np.r_[0,0,move_height], drop_pos, drop_R, drop_R, True)
sim.execute_trajectory((traj, dof_inds))
sim.open_gripper(lr)

traj = get_move_traj(drop_pos, drop_pos + np.r_[0,0,move_height], drop_R, drop_R, True)
sim.execute_trajectory((traj, dof_inds))

sim.settle(max_steps=10000)
viewer.Idle()
