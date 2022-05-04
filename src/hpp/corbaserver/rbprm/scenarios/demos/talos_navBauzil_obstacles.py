from hpp.corbaserver.rbprm.scenarios.demos.talos_navBauzil_obstacles_path import (
    PathPlanner,
)
from hpp.corbaserver.rbprm.scenarios.talos_contact_generator import (
    TalosContactGenerator,
)


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())
        self.robustness = 2.0

    def load_fullbody(self):
        super().load_fullbody()
        self.fullbody.nbSamples = 100000
        self.fullbody.limbs_names = [
            self.fullbody.lLegId,
            self.fullbody.rLegId,
        ]  # left feet first on this scenario


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
