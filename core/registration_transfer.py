from __future__ import division

class RegistrationAndTrajectoryTransferer(object):
    def __init__(self, registration_factory, trajectory_transferer):
        self.registration_factory = registration_factory
        self.trajectory_transferer = trajectory_transferer
    
    def transfer(self, demo, test_scene_state, plotting=False):
        """Registers demonstration scene onto the test scene and uses this registration to transfer the demonstration trajectory
        
        Args:
            demo: Demonstration that has the demonstration scene and the trajectory to transfer
            test_scene_state: SceneState of the test scene
        
        Returns:
            The transferred Trajectory
        """
        raise NotImplementedError

class TwoStepRegistrationAndTrajectoryTransferer(RegistrationAndTrajectoryTransferer):
    def transfer(self, demo, test_scene_state, plotting=False):
        reg = self.registration_factory.register(demo, test_scene_state)
        print 'computed registration'
        test_aug_traj = self.trajectory_transferer.transfer(reg, demo, plotting=plotting)
        print 'computed test traj'
        return test_aug_traj

class UnifiedRegistrationAndTrajectoryTransferer(RegistrationAndTrajectoryTransferer):
    def __init__(self, registration_factory, trajectory_transferer):
        self.registration_factory = registration_factory
        self.trajectory_transferer = trajectory_transferer
        # TODO
        raise NotImplementedError

    def transfer(self, demo, test_scene_state, plotting=False):
        # TODO
        raise NotImplementedError

