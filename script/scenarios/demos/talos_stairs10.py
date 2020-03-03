from scenarios.demos.talos_stairs10_path import PathPlanner
from scenarios.talos_contact_generator import TalosContactGenerator

class ContactGenerator(TalosContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())

    def set_joints_bounds(self):
        super().set_joints_bounds()
        self.fullbody.setConstrainedJointsBounds()

    def run(self):
        self.load_fullbody()
        # path planning use different limbs, reset it to right/left feet
        self.used_limbs = [self.fullbody.lLegId, self.fullbody.rLegId]
        self.init_contacts = self.used_limbs
        self.end_contacts = self.used_limbs
        self.set_joints_bounds()
        self.set_reference(True)
        self.load_limbs("fixedStep06", nb_samples=100000)
        self.init_problem()
        self.init_viewer()
        self.compute_configs_from_guide()
        self.q_goal[2] = self.q_ref[2] + 0.6 # set height to the top of the stairs
        self.interpolate()


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()

