from hpp.corbaserver.rbprm.scenarios.demos.talos_flatGround_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.talos_contact_generator import TalosContactGenerator


class ContactGenerator(TalosContactGenerator, object):
    def __init__(self):
        super(ContactGenerator, self).__init__(PathPlanner())


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
