from hpp.corbaserver.rbprm.scenarios.memmo.talos_navBauzil_oriented_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.memmo.talos_contact_generator import TalosContactGenerator
import time


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())

    def load_fullbody(self):
        from talos_rbprm.talos import Robot
        Robot.urdfSuffix += "_safeFeet"
        self.fullbody = Robot()
        self.q_ref = self.fullbody.referenceConfig[::] + [0] * self.path_planner.extra_dof
        self.weight_postural = self.fullbody.postureWeights[::] + [0] * self.path_planner.extra_dof

    def run(self):
        super().run()
        self.write_status(50)


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
