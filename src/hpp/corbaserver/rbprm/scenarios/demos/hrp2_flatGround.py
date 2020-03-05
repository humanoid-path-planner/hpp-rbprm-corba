from scenarios.demos.hrp2_flatGround_path import PathPlanner
from scenarios.hrp2_contact_generator import HRP2ContactGenerator

class ContactGenerator(HRP2ContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())

if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()




