from hpp.corbaserver.rbprm.scenarios.memmo.anymal_platform_random_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.memmo.anymal_contact_generator import AnymalContactGenerator
import time

class ContactGenerator(AnymalContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())
        self.pid = 0
        self.robustness = 1
        self.dt = 0.001


    def set_joints_bounds(self):
      super().set_joints_bounds()
      self.fullbody.setVeryConstrainedJointsBounds()


    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        if self.q_goal[1] < 0:  # goal on the right side of the circle, start motion with right leg first
            gait = [self.fullbody.rArmId, self.fullbody.rLegId, self.fullbody.lArmId, self.fullbody.lLegId]
        else:
            gait = [self.fullbody.lArmId, self.fullbody.lLegId, self.fullbody.rArmId, self.fullbody.rLegId]
        self.init_contacts = gait
        self.end_contacts = gait

    def set_reference(self):
        super().set_reference(False)

    def load_limbs(self):
        super().load_limbs("static","ReferenceConfiguration",nb_samples=100000)

    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        id_init = self.fullbody.generateStateInContact(self.q_init, [0, 0, 1])
        id_goal = self.fullbody.generateStateInContact(self.q_goal, [0, 0, 1])
        self.q_init = self.fullbody.getConfigAtState(id_init)
        self.q_goal = self.fullbody.getConfigAtState(id_goal)
        self.v(self.q_init)

    def run(self):
        super().run()
        self.fullbody.resetJointsBounds()
        self.write_status(100, False)

if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
