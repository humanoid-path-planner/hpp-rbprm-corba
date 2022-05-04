from hpp.corbaserver.rbprm.scenarios.demos.hyq_darpa_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.hyq_contact_generator import HyqContactGenerator


class ContactGenerator(HyqContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())

    def load_limbs(self):
        dict_heuristic = {
            self.fullbody.rLegId: "fixedStep04",
            self.fullbody.lLegId: "fixedStep04",
            self.fullbody.rArmId: "static",
            self.fullbody.lArmId: "static",
        }
        super().load_limbs(dict_heuristic, "ReferenceConfiguration")

    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        self.q_init[2] = self.q_ref[2]
        self.q_goal[2] = self.q_ref[2]


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
    cg.play_guide_path()
    cg.display_sequence()
