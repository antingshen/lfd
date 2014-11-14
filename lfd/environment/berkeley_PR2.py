import numpy as np
from base_robot import BaseRobot
from lfd.environment import sim_util
from lfd.environment.simulation import DynamicSimulation
from lfd.rapprentice import ropesim, animate_traj


class _BerkeleyPR2(BaseRobot):

    def __init__(self):
        BaseRobot.__init__(self)
        self.including_gripper_finger_collisions = 0
        self.finger_link_names = ["%s_gripper_%s_finger_tip_link" % (lr, flr)
                                  for lr in 'lr' for flr in 'lr']

    def pre_add_objects(self, simulation):
        if not self.including_gripper_finger_collisions:
            self._include_gripper_finger_collisions(simulation)
        self.including_gripper_finger_collisions += 1
    pre_remove_objects = pre_add_objects
    pre_set_state = pre_add_objects

    def post_add_objects(self, simulation):
        self.including_gripper_finger_collisions -= 1
        assert self.including_gripper_finger_collisions >= 0
        if not self.including_gripper_finger_collisions:
            self._exclude_gripper_finger_collisions(simulation)
    post_remove_objects = post_add_objects
    post_set_state = post_add_objects

BerkeleyPR2 = _BerkeleyPR2()


class BerkeleyPR2DynamicSimulation(DynamicSimulation):
    def __init__(self, env=None, T_w_k=None, range_k=2.):
        """
        T_w_k: world transform of the depth camera
        range_k: length of the rays. 2 meters by default
        """
        self.constraints = {"l": [], "r": []}
        self.constraints_links = {"l": [], "r": []}
        super(BerkeleyPR2DynamicSimulation, self).__init__(robot_type=BerkeleyPR2, env=env)
        self.T_w_k = T_w_k
        self.range_k = range_k

    def observe_cloud(self):
        if self.T_w_k is None:
            if self.robot is None:
                raise RuntimeError("Can't observe cloud when there is no robot")
            else:
                from lfd.rapprentice import berkeley_pr2
                self.T_w_k = berkeley_pr2.get_kinect_transform(self.robot)

        # camera's parameters
        cx = 320.-.5
        cy = 240.-.5
        f = 525.  # focal length
        w = 640.
        h = 480.

        pixel_ij = np.array(np.meshgrid(np.arange(w), np.arange(h))).T.reshape((-1, 2))  # all pixel positions
        rays_to = self.range_k * np.c_[(pixel_ij - np.array([cx, cy])) / f, np.ones(pixel_ij.shape[0])]
        rays_from = np.zeros_like(rays_to)
        # transform the rays from the camera frame to the world frame
        rays_to = rays_to.dot(self.T_w_k[:3,:3].T) + self.T_w_k[:3,3]
        rays_from = rays_from.dot(self.T_w_k[:3,:3].T) + self.T_w_k[:3,3]

        cloud = []
        for sim_obj in self.dyn_sim_objs:
            for bt_obj in sim_obj.get_bullet_objects():
                ray_collisions = self.bt_env.RayTest(rays_from, rays_to, bt_obj)

                pts = np.empty((len(ray_collisions), 3))
                for i, ray_collision in enumerate(ray_collisions):
                    pts[i, :] = ray_collision.pt
                cloud.append(pts)
        cloud = np.concatenate(cloud)

        # hack to filter out point below the top of the table. TODO: fix this hack
        table_sim_objs = [sim_obj for sim_obj in self.sim_objs if "table" in sim_obj.names]
        assert len(table_sim_objs) == 1
        table_sim_obj = table_sim_objs[0]
        table_height = table_sim_obj.translation[2] + table_sim_obj.extents[2]
        cloud = cloud[cloud[:, 2] > table_height, :]
        return cloud

    def open_gripper(self, lr, target_val=None, step_viewer=1, max_vel=.02):
        self._remove_constraints(lr)

        # generate gripper finger trajectory
        joint_ind = self.robot.GetJoint("%s_gripper_l_finger_joint" % lr).GetDOFIndex()
        start_val = self.robot.GetDOFValues([joint_ind])[0]
        if target_val is None:
            target_val = sim_util.get_binary_gripper_angle(True)
        joint_traj = np.linspace(start_val, target_val, np.ceil(abs(target_val - start_val) / max_vel))

        # execute gripper finger trajectory
        for val in joint_traj:
            self.robot.SetDOFValues([val], [joint_ind])
            self.step()
        if self.viewer and step_viewer:
            self.viewer.Step()

    def close_gripper(self, lr, step_viewer=1, max_vel=.02, close_dist_thresh=0.004, grab_dist_thresh=0.005):
        print 'CLOSING GRIPPER'
        # generate gripper finger trajectory
        joint_ind = self.robot.GetJoint("%s_gripper_l_finger_joint" % lr).GetDOFIndex()
        start_val = self.robot.GetDOFValues([joint_ind])[0]
        print 'start_val: ', start_val
        # execute gripper finger trajectory
        dyn_bt_objs = [bt_obj for sim_obj in self.dyn_sim_objs for bt_obj in sim_obj.get_bullet_objects()]
        next_val = start_val
        while next_val:
            flr2finger_pts_grid = self._get_finger_pts_grid(lr)
            ray_froms, ray_tos = flr2finger_pts_grid['l'], flr2finger_pts_grid['r']

            # stop closing if any ray hits a dynamic object within a distance of close_dist_thresh from both sides
            next_vel = max_vel
            for bt_obj in dyn_bt_objs:
                from_to_ray_collisions = self.bt_env.RayTest(ray_froms, ray_tos, bt_obj)
                to_from_ray_collisions = self.bt_env.RayTest(ray_tos, ray_froms, bt_obj)
                rays_dists = np.inf * np.ones((len(ray_froms), 2))
                for rc in from_to_ray_collisions:
                    ray_id = np.argmin(np.apply_along_axis(np.linalg.norm, 1, ray_froms - rc.rayFrom))
                    rays_dists[ray_id, 0] = np.linalg.norm(rc.pt - rc.rayFrom)
                for rc in to_from_ray_collisions:
                    ray_id = np.argmin(np.apply_along_axis(np.linalg.norm, 1, ray_tos - rc.rayFrom))
                    rays_dists[ray_id, 1] = np.linalg.norm(rc.pt - rc.rayFrom)
                colliding_rays_inds = np.logical_and(rays_dists[:, 0] != np.inf, rays_dists[:, 1] != np.inf)
                if np.any(colliding_rays_inds):
                    rays_dists = rays_dists[colliding_rays_inds, :]
                    if np.any(np.logical_and(rays_dists[:, 0] < close_dist_thresh,
                                             rays_dists[:, 1] < close_dist_thresh)):
                        next_vel = 0
                    else:
                        next_vel = np.minimum(next_vel, np.min(rays_dists.sum(axis=1)))
            if next_vel == 0:
                break
            next_val = np.maximum(next_val - next_vel, 0)

            self.robot.SetDOFValues([next_val], [joint_ind])
            self.step()
            if self.viewer and step_viewer:
                self.viewer.Step()
        handles = []
        # add constraints at the points where a ray hits a dynamic link within a distance of grab_dist_thresh
        for bt_obj in dyn_bt_objs:
            from_to_ray_collisions = self.bt_env.RayTest(ray_froms, ray_tos, bt_obj)
            to_from_ray_collisions = self.bt_env.RayTest(ray_tos, ray_froms, bt_obj)

            for i in range(ray_froms.shape[0]):
                self.viewer.Step()
            ray_collisions = [rc for rcs in [from_to_ray_collisions, to_from_ray_collisions] for rc in rcs]

            for rc in ray_collisions:
                if rc.link == bt_obj.GetKinBody().GetLink('rope_59'):
                    self.viewer.Step()
                if np.linalg.norm(rc.pt - rc.rayFrom) < grab_dist_thresh:
                    link_tf = rc.link.GetTransform()
                    link_tf[:3, 3] = rc.pt
                    self._add_constraints(lr, rc.link, link_tf)
        if self.viewer and step_viewer:
            self.viewer.Step()

    def execute_trajectory(self, full_traj, step_viewer=1, interactive=False,
                           max_cart_vel_trans_traj=.05, sim_callback=None):
        # TODO: incorporate other parts of sim_full_traj_maybesim
        if sim_callback is None:
            sim_callback = lambda i: self.step()

        traj, dof_inds = full_traj

    #     # clip finger joint angles to the binary gripper angles if necessary
    #     for lr in 'lr':
    #         joint_ind = self.robot.GetJoint("%s_gripper_l_finger_joint"%lr).GetDOFIndex()
    #         if joint_ind in dof_inds:
    #             ind = dof_inds.index(joint_ind)
    #             traj[:,ind] = np.minimum(traj[:,ind], get_binary_gripper_angle(True))
    #             traj[:,ind] = np.maximum(traj[:,ind], get_binary_gripper_angle(False))

        # in simulation mode, we must make sure to gradually move to the new starting position
        self.robot.SetActiveDOFs(dof_inds)
        curr_vals = self.robot.GetActiveDOFValues()
        transition_traj = np.r_[[curr_vals], [traj[0]]]
        sim_util.unwrap_in_place(transition_traj, dof_inds=dof_inds)
        transition_traj = ropesim.retime_traj(self.robot, dof_inds, transition_traj,
                                              max_cart_vel=max_cart_vel_trans_traj)
        animate_traj.animate_traj(transition_traj, self.robot, restore=False, pause=interactive,
                                  callback=sim_callback, step_viewer=step_viewer if self.viewer else 0)

        traj[0] = transition_traj[-1]
        sim_util.unwrap_in_place(traj, dof_inds=dof_inds)
        traj = ropesim.retime_traj(self.robot, dof_inds, traj)  # make the trajectory slow enough for the simulation

        animate_traj.animate_traj(traj, self.robot, restore=False, pause=interactive,
                                  callback=sim_callback, step_viewer=step_viewer if self.viewer else 0)
        if self.viewer and step_viewer:
            self.viewer.Step()
        return True

    def _create_bullet(self):
        for lr in 'lr':
            if self.constraints[lr] or self.constraints_links[lr]:
                raise RuntimeError("Bullet environment can't be removed while the robot is grasping an object")
        super(BerkeleyPR2DynamicSimulation, self)._create_bullet()

    def _remove_bullet(self):
        for lr in 'lr':
            if self.constraints[lr] or self.constraints_links[lr]:
                raise RuntimeError("Bullet environment can't be removed while the robot is grasping an object")
        super(BerkeleyPR2DynamicSimulation, self)._remove_bullet()

    def _get_finger_pts_grid(self, lr, min_sample_dist=0.005):
        sample_grid = None
        flr2finger_pts_grid = {}
        for finger_lr in 'lr':
            world_from_finger = self.robot.GetLink("%s_gripper_%s_finger_tip_link" % (lr, finger_lr)).GetTransform()
            finger_pts = world_from_finger[:3, 3] \
                + sim_util.get_finger_rel_pts(finger_lr).dot(world_from_finger[:3, :3].T)
            pt0 = finger_pts[0 if finger_lr == 'l' else 3][None, :]
            pt1 = finger_pts[1 if finger_lr == 'l' else 2][None, :]
            pt3 = finger_pts[3 if finger_lr == 'l' else 0][None, :]
            if sample_grid is None:
                num_sample_01 = np.round(np.linalg.norm(pt1 - pt0)/min_sample_dist)
                num_sample_03 = np.round(np.linalg.norm(pt3 - pt0)/min_sample_dist)
                sample_grid = np.array(np.meshgrid(np.linspace(0, 1, num_sample_01),
                                                   np.linspace(0, 1, num_sample_03))).T.reshape((-1, 2))
            flr2finger_pts_grid[finger_lr] = pt0 + sample_grid[:, 0][:, None].dot(pt1 - pt0) \
                + sample_grid[:, 1][:, None].dot(pt3 - pt0)
        return flr2finger_pts_grid

    def _remove_constraints(self, lr, grab_link=None):
        """
        If grab_link is None, remove all constraints that attaches the lr gripper,
        else remove all constraints that attaches between the lr gripper and grab_link
        """
        num_links_removed = 0
        for (cnt, link) in zip(self.constraints[lr], self.constraints_links[lr]):
            if grab_link is None or link == grab_link:
                self.bt_env.RemoveConstraint(cnt)
                num_links_removed += 1
        # TODO: provide option to color the contrained links and save color before overriding it
        for link in self.constraints_links[lr]:
            if grab_link is None or link == grab_link:
                for geom in link.GetGeometries():
                    geom.SetDiffuseColor([1., 1., 1.])
        if grab_link is None:
            self.constraints[lr] = []
            self.constraints_links[lr] = []
        else:
            if grab_link in self.constraints_links[lr]:
                constraints_links_pairs = zip(*[(cnt, link) for (cnt, link)
                                                in zip(self.constraints[lr], self.constraints_links[lr])
                                                if link != grab_link])
                if constraints_links_pairs:
                    constraints, constraints_links = constraints_links_pairs
                    self.constraints[lr] = list(constraints)
                    self.constraints_links[lr] = list(constraints_links)
                else:
                    self.constraints[lr] = []
                    self.constraints_links[lr] = []

    def _add_constraints(self, lr, grab_link, grab_tf=None):
        if grab_tf is None:
            grab_tf = grab_link.GetTransform()
        # TODO: provide option to color the contrained links and save color before overriding it
        for geom in grab_link.GetGeometries():
            geom.SetDiffuseColor([1.,0.,0.])
        for flr in 'lr':
            robot_link = self.robot.GetLink("%s_gripper_%s_finger_tip_link" % (lr, flr))
            cnt = self.bt_env.AddConstraint({
                "type": "generic6dof",
                "params": {
                    "link_a": robot_link,
                    "link_b": grab_link,
                    "frame_in_a": np.linalg.inv(robot_link.GetTransform()).dot(grab_tf),
                    "frame_in_b": np.linalg.inv(grab_link.GetTransform()).dot(grab_tf),
                    "use_linear_reference_frame_a": False,
                    "stop_erp": .8,
                    "stop_cfm": .1,
                    "disable_collision_between_linked_bodies": True,
                }
            })
            self.constraints[lr].append(cnt)
            self.constraints_links[lr].append(grab_link)


class BerkeleyPR2DynamicRopeSimulation(BerkeleyPR2DynamicSimulation):

    def in_grasp_region(self,robot, lr, pt):
        tol = .00

        manip_name = {"l": "leftarm", "r": "rightarm"}[lr]
        manip = robot.GetManipulator(manip_name)
        l_finger = robot.GetLink("%s_gripper_l_finger_tip_link"%lr)
        r_finger = robot.GetLink("%s_gripper_r_finger_tip_link"%lr)

        def transform(hmat, p):
            return hmat[:3,:3].dot(p) + hmat[:3,3]
        def on_inner_side(pt, finger_lr):
            finger = l_finger
            closing_dir = np.cross(manip.GetLocalToolDirection(), [-1, 0, 0])
            local_inner_pt = np.array([0.234402, -0.299, 0])/20.
            if finger_lr == "r":
                finger = r_finger
                closing_dir *= -1
                local_inner_pt[1] *= -1
            inner_pt = transform(finger.GetTransform(), local_inner_pt)
            return manip.GetTransform()[:3,:3].dot(closing_dir).dot(pt - inner_pt) > 0

        # check that pt is behind the gripper tip
        pt_local = transform(np.linalg.inv(manip.GetTransform()), pt)
        if pt_local[2] > .03 + tol:
            return False

        # check that pt is within the finger width
        if abs(pt_local[0]) > .01 + tol:
            return False

        # check that pt is between the fingers
        if not on_inner_side(pt, "l") or not on_inner_side(pt, "r"):
            return False

        return True



    def close_gripper(self, lr, step_viewer=1, max_vel=.02, close_dist_thresh=0.004, grab_dist_thresh=0.005):
        print 'CLOSING GRIPPER'
        # generate gripper finger trajectory
        joint_ind = self.robot.GetJoint("%s_gripper_l_finger_joint"%lr).GetDOFIndex()
        start_val = self.robot.GetDOFValues([joint_ind])[0]
        print 'start_val: ', start_val
        # execute gripper finger trajectory
        dyn_bt_objs = [bt_obj for sim_obj in self.dyn_sim_objs for bt_obj in sim_obj.get_bullet_objects()]
        next_val = start_val
        while next_val:
            flr2finger_pts_grid = self._get_finger_pts_grid(lr)
            ray_froms, ray_tos = flr2finger_pts_grid['l'], flr2finger_pts_grid['r']

            # stop closing if any ray hits a dynamic object within a distance of close_dist_thresh from both sides
            next_vel = max_vel
            for bt_obj in dyn_bt_objs:
                from_to_ray_collisions = self.bt_env.RayTest(ray_froms, ray_tos, bt_obj)
                to_from_ray_collisions = self.bt_env.RayTest(ray_tos, ray_froms, bt_obj)
                rays_dists = np.inf * np.ones((len(ray_froms), 2))
                for rc in from_to_ray_collisions:
                    ray_id = np.argmin(np.apply_along_axis(np.linalg.norm, 1, ray_froms - rc.rayFrom))
                    rays_dists[ray_id,0] = np.linalg.norm(rc.pt - rc.rayFrom)
                for rc in to_from_ray_collisions:
                    ray_id = np.argmin(np.apply_along_axis(np.linalg.norm, 1, ray_tos - rc.rayFrom))
                    rays_dists[ray_id,1] = np.linalg.norm(rc.pt - rc.rayFrom)
                colliding_rays_inds = np.logical_and(rays_dists[:,0] != np.inf, rays_dists[:,1] != np.inf)
                if np.any(colliding_rays_inds):
                    rays_dists = rays_dists[colliding_rays_inds,:]
                    if np.any(np.logical_and(rays_dists[:,0] < close_dist_thresh, rays_dists[:,1] < close_dist_thresh)):
                        next_vel = 0
                    else:
                        next_vel = np.minimum(next_vel, np.min(rays_dists.sum(axis=1)))
            if next_vel == 0:
                break
            next_val = np.maximum(next_val - next_vel, 0)

            self.robot.SetDOFValues([next_val], [joint_ind])
            self.step()
            if self.viewer and step_viewer:
                self.viewer.Step()


        rope = [bt_obj for sim_obj in self.dyn_sim_objs for bt_obj in sim_obj.get_bullet_objects()][0]
        nodes, ctl_pts = rope.GetNodes(), rope.GetControlPoints()

        graspable_nodes = np.array([self.in_grasp_region(self.robot, lr, n) for n in nodes])
        graspable_ctl_pts = np.array([self.in_grasp_region(self.robot, lr, n) for n in ctl_pts])
        graspable_inds = np.flatnonzero(np.logical_or(graspable_nodes, np.logical_or(graspable_ctl_pts[:-1], graspable_ctl_pts[1:])))
        print 'graspable inds for %s: %s' % (lr, str(graspable_inds))
        if len(graspable_inds) == 0:
            return False

        robot_link = self.robot.GetLink("%s_gripper_l_finger_tip_link"%lr)
        rope_links = rope.GetKinBody().GetLinks()
        for i_node in graspable_inds:
            i_cnt = i_node
            for geom in rope_links[i_cnt].GetGeometries():
                geom.SetDiffuseColor([1.,0.,0.])
            link = rope_links[i_cnt]
            self._add_constraints(lr, link, link.GetTransform())
        if step_viewer and self.viewer:
            self.viewer.Step()

        return True

    def release_rope(self, lr):
#         print 'RELEASE: %s (%d constraints)' % (lr, len(self.constraints[lr]))
        for c in self.constraints[lr]:
            self.bt_env.RemoveConstraint(c)
        rope_links = self.rope.GetKinBody().GetLinks()
        for link in self.constraints_links[lr]:
            for geom in link.GetGeometries():
                geom.SetDiffuseColor([1.,1.,1.])
        self.constraints[lr] = []
        self.constraints_links[lr] = []