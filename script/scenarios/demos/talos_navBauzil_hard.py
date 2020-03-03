from scenarios.demos.talos_navBauzil_hard_path import PathPlanner
from scenarios.talos_contact_generator import TalosContactGenerator

class ContactGenerator(TalosContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())

    def load_limbs(self, heuristic):
        super().load_limbs(heuristic)
        # In this scenario, the arms are not used to create contact, but they may move to avoid collision.
        self.fullbody.addNonContactingLimb(self.fullbody.lArmId, self.fullbody.larm, self.fullbody.lhand, 5000)
        self.fullbody.runLimbSampleAnalysis(self.fullbody.lArmId, "ReferenceConfiguration", True)
        self.fullbody.addNonContactingLimb(self.fullbody.rArmId, self.fullbody.rarm, self.fullbody.rhand, 5000)
        self.fullbody.runLimbSampleAnalysis(self.fullbody.rArmId, "ReferenceConfiguration", True)


    def run(self):
        self.load_fullbody()
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



