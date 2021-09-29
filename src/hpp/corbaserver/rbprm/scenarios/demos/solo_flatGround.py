from hpp.corbaserver.rbprm.scenarios.demos.solo_flatGround_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.solo_contact_generator import SoloContactGenerator


class ContactGenerator(SoloContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())
        self.testReachability=False


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
