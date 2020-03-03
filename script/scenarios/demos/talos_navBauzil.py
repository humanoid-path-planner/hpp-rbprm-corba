from scenarios.demos.talos_navBauzil_path import PathPlanner
from scenarios.talos_contact_generator import TalosContactGenerator

class ContactGenerator(TalosContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())


    def run(self):
        self.load_fullbody()
        # set reference configuration with the arms bended to avoid collisions with the plateform
        self.q_ref = self.fullbody.referenceConfig_elbowsUp[::] + [0] * self.path_planner.extra_dof
        self.set_joints_bounds()
        self.set_reference(True)
        self.load_limbs("fixedStep06")
        self.init_problem()
        self.init_viewer()
        self.compute_configs_from_guide()
        self.interpolate()


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()



