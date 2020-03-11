from hpp.corbaserver.rbprm.scenarios.demos.talos_plateformes_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.talos_contact_generator import TalosContactGenerator


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())

    def load_fullbody(self):
        from talos_rbprm.talos import Robot
        # use a model with upscaled collision geometry for the feet
        Robot.urdfSuffix += "_safeFeet"
        self.fullbody = Robot()
        self.q_ref = self.fullbody.referenceConfig[::] + [0] * self.path_planner.extra_dof
        self.weight_postural = self.fullbody.postureWeights[::] + [0] * self.path_planner.extra_dof

    def set_joints_bounds(self):
        super().set_joints_bounds()
        # set conservative bounds for the leg z joint
        self.fullbody.setJointBounds('leg_left_1_joint', [-0.1, 0.2])
        self.fullbody.setJointBounds('leg_right_1_joint', [-0.2, 0.1])

    def run(self):
        self.load_fullbody()
        self.set_joints_bounds()
        self.set_reference(False)
        self.load_limbs("fixedStep08", "ReferenceConfiguration", nb_samples=100000)
        self.init_problem()
        self.init_viewer()
        self.compute_configs_from_guide()
        # set the height to match the platefrom height
        self.q_init[2] = self.q_ref[2] + 0.16
        self.q_goal[2] = self.q_ref[2] + 0.16
        # set right foot first
        self.init_contacts = [self.fullbody.rLegId, self.fullbody.lLegId]
        self.interpolate()
        # remove constrained bounds on joints set before
        self.fullbody.resetJointsBounds()


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
