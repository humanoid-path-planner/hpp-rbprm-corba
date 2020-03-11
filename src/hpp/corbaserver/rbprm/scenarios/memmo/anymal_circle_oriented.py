from hpp.corbaserver.rbprm.scenarios.memmo.anymal_circle_oriented_path import PathPlanner
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
        self.fullbody.setJointBounds('LF_HAA', [-0.2, 0.2])
        self.fullbody.setJointBounds('RF_HAA', [-0.2, 0.2])
        self.fullbody.setJointBounds('LH_HAA', [-0.2, 0.2])
        self.fullbody.setJointBounds('RH_HAA', [-0.2, 0.2])

    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        if self.q_goal[1] < 0:  # goal on the right side of the circle, start motion with right leg first
            gait = [self.fullbody.rArmId, self.fullbody.rLegId, self.fullbody.lArmId, self.fullbody.lLegId]
        else:
            gait = [self.fullbody.lArmId, self.fullbody.lLegId, self.fullbody.rArmId, self.fullbody.rLegId]
        self.init_contacts = gait
        self.end_contacts = gait

    def run(self):
        super().run()
        self.fullbody.resetJointsBounds()
        self.write_status(60)


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
