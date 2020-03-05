from scenarios.demos.hyq_slalom_debris_path import PathPlanner
from scenarios.hyq_contact_generator import HyqContactGenerator

class ContactGenerator(HyqContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())

    def load_limbs(self):
        super().load_limbs("fixedStep06", "ReferenceConfiguration")


    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        self.q_init[2] = self.q_ref[2]
        self.q_goal[2] = self.q_ref[2]


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()

