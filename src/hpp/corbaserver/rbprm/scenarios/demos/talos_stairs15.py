from hpp.corbaserver.rbprm.scenarios.demos.talos_stairs15_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.talos_contact_generator import TalosContactGenerator


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())
        self.test_reachability = False

    def load_fullbody(self):
        super().load_fullbody()
        self.q_ref = self.fullbody.referenceConfig_elbowsUp[::] + [0] * self.path_planner.extra_dof
        # path planning use different limbs, reset it to right/left feet
        self.used_limbs = [self.fullbody.lLegId, self.fullbody.rLegId, self.fullbody.rArmId]
        self.init_contacts = [self.fullbody.lLegId, self.fullbody.rLegId]
        self.end_contacts = self.used_limbs

    def load_limbs(self):
        super().load_limbs("static", nb_samples=100000)

    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        self.q_goal[2] = self.q_ref[2] + 0.75  # set height to the top of the stairs


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
