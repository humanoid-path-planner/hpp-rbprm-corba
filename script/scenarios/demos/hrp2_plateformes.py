from scenarios.demos.hrp2_plateformes_path import PathPlanner
from scenarios.hrp2_contact_generator import HRP2ContactGenerator

class ContactGenerator(HRP2ContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())

    def run(self):
        self.root_translation_bounds = [-5, 5, -1.5, 1.5, 0.65, 0.9]
        self.robustness = 0.
        self.quasi_static = False

        self.load_fullbody()
        self.set_joints_bounds()
        self.set_reference(True)

        self.fullbody.minDist = 0.3
        self.load_limbs("fixedStep1")
        self.init_problem()
        self.init_viewer()
        self.compute_configs_from_guide()
        # force root height to be at exactly on the first platform
        self.q_init[2] = self.q_ref[2] + 0.16
        self.q_goal[2] = self.q_ref[2] + 0.16
        self.interpolate()


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()


