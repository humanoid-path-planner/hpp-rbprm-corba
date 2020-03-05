from scenarios.demos.talos_navBauzil_path import PathPlanner
from scenarios.talos_contact_generator import TalosContactGenerator

class ContactGenerator(TalosContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())

    def set_reference(self):
        super().set_reference()
        self.q_ref = self.fullbody.referenceConfig_elbowsUp[::] + [0] * self.path_planner.extra_dof


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()



