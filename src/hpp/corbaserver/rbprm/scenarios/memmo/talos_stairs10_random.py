from hpp.corbaserver.rbprm.scenarios.memmo.talos_stairs10_random_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.memmo.talos_contact_generator import TalosContactGenerator
import time


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())

    def load_fullbody(self):
        super().load_fullbody()
        # path planning use different limbs, reset it to right/left feet
        self.used_limbs = [self.fullbody.lLegId, self.fullbody.rLegId]
        self.init_contacts = self.used_limbs
        self.end_contacts = self.used_limbs

    def set_reference(self):
        self.q_ref = self.fullbody.referenceConfig_legsApart[::] + [0] * 6
        super().set_reference()

    def load_limbs(self):
        self.fullbody.minDist = 0.85
        super().load_limbs(analysis="ReferenceConfiguration")

    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        if self.path_planner.q_goal[2] > self.path_planner.q_init[2]:
            self.q_goal[2] += 0.6
        else:
            self.q_init[2] += 0.6

    def run(self):
        super().run()
        self.write_status(30)


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
