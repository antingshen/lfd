#!/usr/bin/env python

from __future__ import division

import pprint
import argparse
from core import demonstration, registration, transfer, sim_util
from core.constants import ROPE_RADIUS, MAX_ACTIONS_TO_TRY, DS_SIZE, TORSO_HEIGHT, TORSO_IND

from core.demonstration import SceneState, GroundTruthRopeSceneState, AugmentedTrajectory, Demonstration
from core.simulation import DynamicSimulationRobotWorld, DynamicRopeSimulationRobotWorld
from core.simulation_object import XmlSimulationObject, BoxSimulationObject, CylinderSimulationObject, RopeSimulationObject
from core.environment import LfdEnvironment, GroundTruthRopeLfdEnvironment
from core.registration import TpsRpmBijRegistrationFactory, TpsRpmRegistrationFactory, TpsSegmentRegistrationFactory, GpuTpsRpmBijRegistrationFactory, GpuTpsRpmRegistrationFactory
from core.transfer import PoseTrajectoryTransferer, FingerTrajectoryTransferer
from core.transfer_simulate import BatchTransferSimulate
from core.registration_transfer import TwoStepRegistrationAndTrajectoryTransferer, UnifiedRegistrationAndTrajectoryTransferer
from core.action_selection import GreedyActionSelection
from core.file_utils import fname_to_obj
from rapprentice import eval_util, util
import sys

import pdb, time

import trajoptpy
from rapprentice.knot_classifier import isKnot as is_knot, calculateCrossings
import os, os.path, numpy as np, h5py
from rapprentice.util import redprint, yellowprint
import atexit
import importlib
from itertools import combinations
import IPython as ipy
import random
import hashlib

class GlobalVars:
    exec_log = None
    actions = None
    actions_cache = None
    demos = None

def eval_on_holdout(args, action_selection, reg_and_traj_transferer, lfd_env, sim):
    """TODO
    
    Args:
        action_selection: ActionSelection
        reg_and_traj_transferer: RegistrationAndTrajectoryTransferer
        lfd_env: LfdEnvironment
        sim: DynamicSimulation
    """
    holdoutfile = h5py.File(args.eval.holdoutfile, 'r')
    holdout_items = eval_util.get_indexed_items(holdoutfile, task_list=args.tasks, task_file=args.taskfile, i_start=args.i_start, i_end=args.i_end)

    rope_params = sim_util.RopeParams()
    if args.eval.rope_param_radius is not None:
        rope_params.radius = args.eval.rope_param_radius
    if args.eval.rope_param_angStiffness is not None:
        rope_params.angStiffness = args.eval.rope_param_angStiffness

    num_successes = 0
    num_total = 0

    sim_util.reset_arms_to_side(sim)
    init_state = sim.get_state()

    for i_task, demo_id_rope_nodes in holdout_items:
        redprint("task %s" % i_task)
        init_rope_nodes = demo_id_rope_nodes["rope_nodes"][:]
        rope = RopeSimulationObject("rope", init_rope_nodes, rope_params)

        sim.set_state(init_state)
        sim.add_objects([rope])
        sim.settle(step_viewer=args.animation)
        
        for i_step in range(args.eval.num_steps):
            redprint("task %s step %i" % (i_task, i_step))
            
            sim_util.reset_arms_to_side(sim)
            if args.animation:
                sim.viewer.Step()
            sim_state = sim.get_state()
            sim.set_state(sim_state)
            scene_state = lfd_env.observe_scene()

            # plot cloud of the test scene
            handles = []
            if args.plotting:
                handles.append(sim.env.plot3(scene_state.cloud[:,:3], 2, scene_state.color if scene_state.color is not None else (0,0,1)))
                sim.viewer.Step()
            
            eval_stats = eval_util.EvalStats()
            
            start_time = time.time()
            try:
                agenda, q_values_root = action_selection.plan_agenda(scene_state)
            except ValueError: #e.g. if cloud is empty - any action is hopeless
                break
            eval_stats.action_elapsed_time += time.time() - start_time
            
            eval_stats.generalized = True
            num_actions_to_try = MAX_ACTIONS_TO_TRY if args.eval.search_until_feasible else 1
            for i_choice in range(num_actions_to_try):
                if q_values_root[i_choice] == -np.inf: # none of the demonstrations generalize
                    eval_stats.generalized = False
                    break
                redprint("TRYING %s"%agenda[i_choice])

                best_root_action = agenda[i_choice]

                start_time = time.time()
                test_aug_traj = reg_and_traj_transferer.transfer(GlobalVars.demos[best_root_action], scene_state, sim_state=sim_state, plotting=args.plotting)
                eval_stats.feasible, eval_stats.misgrasp = lfd_env.execute_augmented_trajectory(test_aug_traj, step_viewer=args.animation, interactive=args.interactive, check_feasible=args.eval.check_feasible)
                eval_stats.exec_elapsed_time += time.time() - start_time
                
                if not args.eval.check_feasible or eval_stats.feasible:  # try next action if TrajOpt cannot find feasible action and we care about feasibility
                     break
                else:
                     sim.set_state(sim_state)
            print "BEST ACTION:", best_root_action

            knot = is_knot(rope.rope.GetControlPoints())
            results = {'scene_state':scene_state, 
                       'best_action':str(best_root_action), 
                       'values':q_values_root, 
                       'aug_traj':test_aug_traj, 
                       'eval_stats':eval_stats, 
                       'sim_state':sim_state, 
                       'knot':knot}
            eval_util.save_task_results_step(args.resultfile, i_task, i_step, results)

            if not eval_stats.generalized:
                assert not knot
                break
            
            if args.eval.check_feasible and not eval_stats.feasible:
                # Skip to next knot tie if the action is infeasible -- since
                # that means all future steps (up to 5) will have infeasible trajectories
                assert not knot
                break
            
            if knot:
                num_successes += 1
                break;
        
        sim.remove_objects([rope])
        
        num_total += 1
        redprint('Eval Successes / Total: ' + str(num_successes) + '/' + str(num_total))
    return num_successes / num_total

def eval_on_holdout_parallel(args, action_selection, lfd_env, sim):
    holdoutfile = h5py.File(args.eval.holdoutfile, 'r')
    holdout_items = eval_util.get_indexed_items(holdoutfile, task_list=args.tasks, task_file=args.taskfile, i_start=args.i_start, i_end=args.i_end)

    bts = BatchTransferSimulate(args, GlobalVars.demos)

    rope_params = sim_util.RopeParams()
    if args.eval.rope_param_radius is not None:
        rope_params.radius = args.eval.rope_param_radius
    if args.eval.rope_param_angStiffness is not None:
        rope_params.angStiffness = args.eval.rope_param_angStiffness

    results = {}
    successes = {}
    blank_state = sim.get_state()
    for i_task, demo_id_rope_nodes in holdout_items:
        redprint("task %s" % i_task)
        sim.set_state(blank_state)
        init_rope_nodes = demo_id_rope_nodes["rope_nodes"][:]
        rope = RopeSimulationObject("rope", init_rope_nodes, rope_params)

        sim.add_objects([rope])
        sim.settle(step_viewer=args.animation)
        sim_util.reset_arms_to_side(sim)
        scene_state = lfd_env.observe_scene()
        sim_state = sim.get_state()
        agenda, q_values_root = action_selection.plan_agenda(scene_state)
        results[i_task] = {'scene_state':scene_state, 'best_action':str(agenda[0]), 'sim_state':sim_state, 'values':q_values_root}
        bts.queue_transfer_simulate(sim_state, scene_state, agenda[0], (i_task, 0))

    while results:
        cur_expansions = bts.get_results()
        for transfer_data in cur_expansions:
            (next_scene, old_key, knot_success, next_simstate, aug_traj) = (transfer_data['result_state'],
                                                                            transfer_data['metadata'],
                                                                            transfer_data['is_knot'],
                                                                            transfer_data['next_simstate'],
                                                                            transfer_data['aug_traj'])
            sys.stdout.write("\rReceived results for key {}\tmax_steps is {}\t{} successes of {} total                    ".format(old_key, args.eval.num_steps, np.sum(successes.values()), len(holdout_items)))
            sys.stdout.flush()
            i_task, i_step = old_key
            results[i_task]['aug_traj'] = aug_traj
            results[i_task]['knot'] = knot_success
            eval_util.save_task_results_step(args.resultfile, i_task, i_step, results[i_task])
            if knot_success:
                successes[i_task] = True
                del results[i_task]
                continue
            next_i_step = i_step + 1
            if next_i_step < args.eval.num_steps:
                sim.set_state(next_simstate)
                scene_state = lfd_env.observe_scene() # re-observe scene in case we're doing a different
                # type of lfd_env (e.g. GroundTruth)
                try:
                    agenda, q_values_root = action_selection.plan_agenda(scene_state)
                except ValueError: #e.g. if cloud is empty - any action is hopeless
                    del results[i_task]
                    successes[i_task] = False
                    continue
                if len(next_scene.full_cloud) == 0:
                    next_scene.full_cloud = np.zeros((1,3))
                    next_scene.cloud = np.zeros((1,3))
                    print "in eval_on_holdout_parallel: empty cloud"
                    # ipy.embed()
                results[i_task] = {'scene_state':next_scene, 'best_action':str(agenda[0]), 'sim_state':next_simstate, 'values':q_values_root}
                bts.queue_transfer_simulate(next_simstate, scene_state, agenda[0], (i_task, next_i_step))
            else:
                del results[i_task]
                successes[i_task] = False

    num_successes = np.sum(successes.values())
    num_total = len(successes)
    redprint('Eval Successes / Total: ' + str(num_successes) + '/' + str(num_total))
    return num_successes / num_total

def replay_on_holdout(args, action_selection, transfer, lfd_env, sim):
    loadresultfile = h5py.File(args.replay.loadresultfile, 'r')
    loadresult_items = eval_util.get_indexed_items(loadresultfile, task_list=args.tasks, task_file=args.taskfile, i_start=args.i_start, i_end=args.i_end)
    
    num_successes = 0
    num_total = 0
    
    for i_task, task_info in loadresult_items:
        redprint("task %s" % i_task)

        for i_step in range(len(task_info)):
            redprint("task %s step %i" % (i_task, i_step))
            
            replay_results = eval_util.load_task_results_step(args.replay.loadresultfile, i_task, i_step)
            sim_state = replay_results['sim_state']

            if i_step > 0: # sanity check for reproducibility
                sim_util.reset_arms_to_side(sim)
                if sim.simulation_state_equal(sim_state, sim.get_state()):
                    yellowprint("Reproducible results OK")
                else:
                    yellowprint("The replayed simulation state doesn't match the one from the result file")
                
            sim.set_state(sim_state)

            if args.replay.simulate_traj_steps is not None and i_step not in args.replay.simulate_traj_steps:
                continue
            
            if i_step in args.replay.compute_traj_steps: # compute the trajectory in this step
                best_root_action = replay_results['best_action']
                scene_state = replay_results['scene_state']
                # plot cloud of the test scene
                handles = []
                if args.plotting:
                    handles.append(sim.env.plot3(scene_state.cloud[:,:3], 2, scene_state.color if scene_state.color is not None else (0,0,1)))
                    sim.viewer.Step()
                test_aug_traj = reg_and_traj_transferer.transfer(GlobalVars.demos[best_root_action], scene_state, plotting=args.plotting)
            else:
                test_aug_traj = replay_results['aug_traj']
            feasible, misgrasp = lfd_env.execute_augmented_trajectory(test_aug_traj, step_viewer=args.animation, interactive=args.interactive, check_feasible=args.eval.check_feasible)
            
            if replay_results['knot']:
                num_successes += 1
        
        num_total += 1
        redprint('REPLAY Successes / Total: ' + str(num_successes) + '/' + str(num_total))

def build_parser():
    parser = util.ArgumentParser()
    
    parser.add_argument("--animation", type=int, default=0, help="animates if it is non-zero. the viewer is stepped according to this number")
    parser.add_argument("--plotting", type=int, default=1, help="plots if animation != 0 and plotting != 0")
    parser.add_argument("--interactive", action="store_true", help="step animation and optimization if specified")
    parser.add_argument("--resultfile", type=str, help="no results are saved if this is not specified")

    # selects tasks to evaluate/replay
    parser.add_argument("--tasks", type=int, nargs='*', metavar="i_task")
    parser.add_argument("--taskfile", type=str)
    parser.add_argument("--i_start", type=int, default=-1, metavar="i_task")
    parser.add_argument("--i_end", type=int, default=-1, metavar="i_task")
    
    parser.add_argument("--camera_matrix_file", type=str, default='../.camera_matrix.txt')
    parser.add_argument("--window_prop_file", type=str, default='../.win_prop.txt')
    parser.add_argument("--random_seed", type=int, default=None)
    parser.add_argument("--log", type=str, default="")

    subparsers = parser.add_subparsers(dest='subparser_name')

    parser_eval = subparsers.add_parser('eval')
    
    parser_eval.add_argument('actionfile', type=str, nargs='?', default='../bigdata/misc/overhand_actions.h5')
    parser_eval.add_argument('holdoutfile', type=str, nargs='?', default='../bigdata/misc/holdout_set_Jun20_0.10.h5')

    parser_eval.add_argument("transferopt", type=str, nargs='?', choices=['pose', 'finger'], default='finger')
    parser_eval.add_argument("reg_type", type=str, choices=['segment', 'rpm', 'bij'], default='bij')
    parser_eval.add_argument("--unified", type=int, default=0)
    
    parser_eval.add_argument("--obstacles", type=str, nargs='*', choices=['bookshelve', 'boxes', 'cylinders'], default=[])
    parser_eval.add_argument("--downsample_size", type=int, default=DS_SIZE)
    parser_eval.add_argument("--upsample", type=int, default=0)
    parser_eval.add_argument("--upsample_rad", type=int, default=1, help="upsample_rad > 1 incompatible with downsample != 0")
    parser_eval.add_argument("--ground_truth", type=int, default=1)
    
    parser_eval.add_argument("--fake_data_segment",type=str, default='demo1-seg00')
    parser_eval.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
        default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")
    
    parser_eval.add_argument("--search_until_feasible", action="store_true")
    parser_eval.add_argument("--check_feasible", type=int, default=0)

    parser_eval.add_argument("--alpha", type=float, default=1000000.0)
    parser_eval.add_argument("--beta_pos", type=float, default=1000000.0)
    parser_eval.add_argument("--beta_rot", type=float, default=100.0)
    parser_eval.add_argument("--gamma", type=float, default=1000.0)
    parser_eval.add_argument("--use_collision_cost", type=int, default=1)

    parser_eval.add_argument("--num_steps", type=int, default=5, help="maximum number of steps to simulate each task")
    parser_eval.add_argument("--dof_limits_factor", type=float, default=1.0)
    parser_eval.add_argument("--rope_param_radius", type=str, default=None)
    parser_eval.add_argument("--rope_param_angStiffness", type=str, default=None)
    
    parser_eval.add_argument("--parallel", action="store_true")
    parser_eval.add_argument("--gpu", action="store_true", default=False)

    parser_replay = subparsers.add_parser('replay')
    parser_replay.add_argument("loadresultfile", type=str)
    parser_replay.add_argument("--compute_traj_steps", type=int, default=[], nargs='*', metavar='i_step', help="recompute trajectories for the i_step of all tasks")
    parser_replay.add_argument("--simulate_traj_steps", type=int, default=None, nargs='*', metavar='i_step', 
                               help="if specified, restore the rope state from file and then simulate for the i_step of all tasks")
                               # if not specified, the rope state is not restored from file, but it is as given by the sequential simulation
    return parser

def parse_input_args():
    parser = build_parser()
    args = parser.parse_args()
    if not args.animation:
        args.plotting = 0
    return args

def setup_log_file(args):
    if args.log:
        redprint("Writing log to file %s" % args.log)
        GlobalVars.exec_log = task_execution.ExecutionLog(args.log)
        atexit.register(GlobalVars.exec_log.close)
        GlobalVars.exec_log(0, "main.args", args)

def set_global_vars(args):
    if args.random_seed is not None: np.random.seed(args.random_seed)
    GlobalVars.demos = fname_to_obj(args.eval.actionfile)

def setup_lfd_environment_sim(args):
    actions = h5py.File(args.eval.actionfile, 'r')
    
    init_rope_xyz = GlobalVars.demos.values()[0].scene_state.transfer_cld()
    table_height = init_rope_xyz[:,2].mean() - .045
    
    sim_objs = []
    sim_objs.append(XmlSimulationObject("robots/pr2-beta-static.zae", dynamic=False))
    sim_objs.append(BoxSimulationObject("table", [1, 0, table_height + (-.1 + .01)], [.85, .85, .1], dynamic=False))
    if 'bookshelve' in args.eval.obstacles:
        sim_objs.append(XmlSimulationObject("../data/bookshelve.env.xml", dynamic=False))
    if 'boxes' in args.eval.obstacles:
        sim_objs.append(BoxSimulationObject("box0", [.7,.43,table_height+(.01+.12)], [.12,.12,.12], dynamic=False))
        sim_objs.append(BoxSimulationObject("box1", [.74,.47,table_height+(.01+.12*2+.08)], [.08,.08,.08], dynamic=False))
    if 'cylinders' in args.eval.obstacles:
        sim_objs.append(CylinderSimulationObject("cylinder0", [.7,.43,table_height+(.01+.5)], .12, 1., dynamic=False))
        sim_objs.append(CylinderSimulationObject("cylinder1", [.7,-.43,table_height+(.01+.5)], .12, 1., dynamic=False))
        sim_objs.append(CylinderSimulationObject("cylinder2", [.4,.2,table_height+(.01+.65)], .06, .5, dynamic=False))
        sim_objs.append(CylinderSimulationObject("cylinder3", [.4,-.2,table_height+(.01+.65)], .06, .5, dynamic=False))
    
    sim = DynamicSimulationRobotWorld()
    world = sim
    sim.add_objects(sim_objs)
    if args.eval.ground_truth:
        lfd_env = GroundTruthRopeLfdEnvironment(sim, world, upsample=args.eval.upsample, upsample_rad=args.eval.upsample_rad, downsample_size=args.eval.downsample_size)
    else:
        lfd_env = LfdEnvironment(sim, world, downsample_size=args.eval.downsample_size)

    sim.robot.SetDOFValues([TORSO_HEIGHT], [TORSO_IND])
    sim_util.reset_arms_to_side(sim)
    
    if args.animation:
        viewer = trajoptpy.GetViewer(sim.env)
        if os.path.isfile(args.window_prop_file) and os.path.isfile(args.camera_matrix_file):
            print "loading window and camera properties"
            window_prop = np.loadtxt(args.window_prop_file)
            camera_matrix = np.loadtxt(args.camera_matrix_file)
            try:
                viewer.SetWindowProp(*window_prop)
                viewer.SetCameraManipulatorMatrix(camera_matrix)
            except:
                print "SetWindowProp and SetCameraManipulatorMatrix are not defined. Pull and recompile Trajopt."
        else:
            print "move viewer to viewpoint that isn't stupid"
            print "then hit 'p' to continue"
            viewer.Idle()
            print "saving window and camera properties"
            try:
                window_prop = viewer.GetWindowProp()
                camera_matrix = viewer.GetCameraManipulatorMatrix()
                np.savetxt(args.window_prop_file, window_prop, fmt='%d')
                np.savetxt(args.camera_matrix_file, camera_matrix)
            except:
                print "GetWindowProp and GetCameraManipulatorMatrix are not defined. Pull and recompile Trajopt."
        viewer.Step()
    
    if args.eval.dof_limits_factor != 1.0:
        assert 0 < args.eval.dof_limits_factor and args.eval.dof_limits_factor <= 1.0
        active_dof_indices = sim.robot.GetActiveDOFIndices()
        active_dof_limits = sim.robot.GetActiveDOFLimits()
        for lr in 'lr':
            manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
            dof_inds = sim.robot.GetManipulator(manip_name).GetArmIndices()
            limits = np.asarray(sim.robot.GetDOFLimits(dof_inds))
            limits_mean = limits.mean(axis=0)
            limits_width = np.diff(limits, axis=0)
            new_limits = limits_mean + args.eval.dof_limits_factor * np.r_[-limits_width/2.0, limits_width/2.0]
            for i, ind in enumerate(dof_inds):
                active_dof_limits[0][active_dof_indices.tolist().index(ind)] = new_limits[0,i]
                active_dof_limits[1][active_dof_indices.tolist().index(ind)] = new_limits[1,i]
        sim.robot.SetDOFLimits(active_dof_limits[0], active_dof_limits[1])
    return lfd_env, sim

def setup_registration_and_trajectory_transferer(args, sim):
    sim_transfer = DynamicRopeSimulationRobotWorld()
    if args.eval.gpu:
        if args.eval.reg_type == 'rpm':
            reg_factory = GpuTpsRpmRegistrationFactory(GlobalVars.demos)
        elif args.eval.reg_type == 'bij':
            reg_factory = GpuTpsRpmBijRegistrationFactory(GlobalVars.demos)
        else:
            raise RuntimeError("Invalid reg_type option %s"%args.eval.reg_type)
    else:
        if args.eval.reg_type == 'segment':
            reg_factory = TpsSegmentRegistrationFactory(GlobalVars.demos)
        elif args.eval.reg_type == 'rpm':
            reg_factory = TpsRpmRegistrationFactory(GlobalVars.demos)
        elif args.eval.reg_type == 'bij':
            reg_factory = TpsRpmBijRegistrationFactory(GlobalVars.demos, n_iter=10) #TODO
        else:
            raise RuntimeError("Invalid reg_type option %s"%args.eval.reg_type)

    if args.eval.transferopt == 'pose' or args.eval.transferopt == 'finger':
        traj_transferer = PoseTrajectoryTransferer(sim_transfer, args.eval.beta_pos, args.eval.beta_rot, args.eval.gamma, args.eval.use_collision_cost)
        if args.eval.transferopt == 'finger':
            traj_transferer = FingerTrajectoryTransferer(sim_transfer, args.eval.beta_pos, args.eval.gamma, args.eval.use_collision_cost, init_trajectory_transferer=traj_transferer)
    else:
        raise RuntimeError("Invalid transferopt option %s"%args.eval.transferopt)
    
    if args.eval.unified:
        reg_and_traj_transferer = UnifiedRegistrationAndTrajectoryTransferer(reg_factory, traj_transferer)
    else:
        reg_and_traj_transferer = TwoStepRegistrationAndTrajectoryTransferer(reg_factory, traj_transferer)
    return reg_and_traj_transferer

def main():
    args = parse_input_args()

    if args.subparser_name == "eval":
        eval_util.save_results_args(args.resultfile, args)
    elif args.subparser_name == "replay":
        loaded_args = eval_util.load_results_args(args.replay.loadresultfile)
        assert 'eval' not in vars(args)
        args.eval = loaded_args.eval
    else:
        raise RuntimeError("Invalid subparser name")
    
    setup_log_file(args)
    
    set_global_vars(args)

    ipy.embed()

    trajoptpy.SetInteractive(args.interactive)
    lfd_env, sim = setup_lfd_environment_sim(args)
    reg_and_traj_transferer = setup_registration_and_trajectory_transferer(args, sim)
    action_selection = GreedyActionSelection(reg_and_traj_transferer.registration_factory)

    if args.subparser_name == "eval":
        start = time.time()
        if args.eval.parallel:
            eval_on_holdout_parallel(args, action_selection, reg_and_traj_transferer, lfd_env, sim)
        else:
            eval_on_holdout(args, action_selection, reg_and_traj_transferer, lfd_env, sim)
        print "eval time is:\t{}".format(time.time() - start)
    elif args.subparser_name == "replay":
        replay_on_holdout(args, action_selection, reg_and_traj_transferer, lfd_env, sim)
    else:
        raise RuntimeError("Invalid subparser name")

if __name__ == "__main__":
    main()
