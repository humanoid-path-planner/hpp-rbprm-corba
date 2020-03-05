from hpp.corbaserver.rbprm.scenarios.demos.talos_navBauzil_hard_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.talos_contact_generator import TalosContactGenerator

class ContactGenerator(TalosContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())

    def set_reference(self):
        super().set_reference()
        self.q_ref = self.fullbody.referenceConfig_elbowsUp[::] + [0] * self.path_planner.extra_dof


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



