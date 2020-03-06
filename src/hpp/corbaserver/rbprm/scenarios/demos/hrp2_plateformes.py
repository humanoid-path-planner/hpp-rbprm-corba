from hpp.corbaserver.rbprm.scenarios.demos.hrp2_plateformes_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.hrp2_contact_generator import HRP2ContactGenerator


class ContactGenerator(HRP2ContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())
        self.root_translation_bounds = [-5, 5, -1.5, 1.5, 0.65, 0.9]
        self.robustness = 0.
        self.quasi_static = False

    def load_limbs(self):
        super().load_limbs("fixedStep1")

    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        self.q_init[2] = self.q_ref[2] + 0.16
        self.q_goal[2] = self.q_ref[2] + 0.16


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
