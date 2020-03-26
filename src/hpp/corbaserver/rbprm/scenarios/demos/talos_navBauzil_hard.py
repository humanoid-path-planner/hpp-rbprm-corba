from hpp.corbaserver.rbprm.scenarios.demos.talos_navBauzil_hard_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.talos_contact_generator import TalosContactGenerator


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())
        self.robustness = 2.

    def load_fullbody(self):
        super().load_fullbody()
        self.fullbody.nbSamples = 100000
        self.fullbody.limbs_names=[self.fullbody.lLegId, self.fullbody.rLegId] # left feet first on this scenario


    def load_limbs(self, heuristic):
        super().load_limbs(heuristic)
        # In this scenario, the arms are not used to create contact, but they may move to avoid collision.
        self.fullbody.addNonContactingLimb(self.fullbody.lArmId, self.fullbody.larm, self.fullbody.lhand, 5000)
        self.fullbody.runLimbSampleAnalysis(self.fullbody.lArmId, "ReferenceConfiguration", True)
        self.fullbody.addNonContactingLimb(self.fullbody.rArmId, self.fullbody.rarm, self.fullbody.rhand, 5000)
        self.fullbody.runLimbSampleAnalysis(self.fullbody.rArmId, "ReferenceConfiguration", True)


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
