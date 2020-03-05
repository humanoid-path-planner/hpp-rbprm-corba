from hpp.corbaserver.rbprm.scenarios.memmo.talos_circle_oriented_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.memmo.talos_contact_generator import TalosContactGenerator
import time
import numpy as np

class ContactGenerator(TalosContactGenerator):

    def __init__(self):
        super().__init__(PathPlanner())
        self.pid = 0
        self.dt = 0.005


    def load_fullbody(self):
        from talos_rbprm.talos import Robot
        Robot.urdfSuffix += "_safeFeet"
        self.fullbody = Robot()
        self.q_ref = self.fullbody.referenceConfig[::] + [0] * self.path_planner.extra_dof
        self.weight_postural = self.fullbody.postureWeights[::] + [0] * self.path_planner.extra_dof


    def set_joints_bounds(self):
      super().set_joints_bounds()
      self.fullbody.setConstrainedJointsBounds()


    def load_limbs(self, heuristic = "fixedStep06", analysis=None, nb_samples=None, octree_size=None):
        # heuristic used depend on the direction of the motion
        if 0.8 * np.pi > self.path_planner.alpha > 0.2 * np.pi:  # nearly straight walk
            print("use straight walk heuristic")
            heuristic = "fixedStep08"
            analysis = None
            self.fullbody.usePosturalTaskContactCreation(True)
        else:  # more complex motion. Do smaller steps and allow non-straight feet orientation
            print("use smaller steps heuristic")
            analysis = "ReferenceConfiguration"
            heuristic = "fixedStep06"

        self.fullbody.loadAllLimbs(heuristic, analysis, nbSamples= 100000)

        if self.path_planner.q_goal[1] < 0:
            print("start with right leg")
            self.init_contacts = [self.fullbody.rLegId, self.fullbody.lLegId]
            self.end_contacts = [self.fullbody.rLegId, self.fullbody.lLegId]
        else:
            print("start with left leg")
            self.init_contacts = [self.fullbody.lLegId, self.fullbody.rLegId]
            self.end_contacts = [self.fullbody.lLegId, self.fullbody.rLegId]


    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        vel_init = [0, 0, 0]
        acc_init = [0, 0, 0]
        vel_goal = [0, 0, 0]
        acc_goal = [0, 0, 0]
        configSize = self.fullbody.getConfigSize() - self.fullbody.client.robot.getDimensionExtraConfigSpace()
        self.q_init[configSize:configSize + 3] = vel_init[::]
        self.q_init[configSize + 3:configSize + 6] = acc_init[::]
        self.q_goal[configSize:configSize + 3] = vel_goal[::]
        self.q_goal[configSize + 3:configSize + 6] = acc_goal[::]

    def run(self):
        super().run()
        self.fullbody.resetJointsBounds()
        self.write_status(20)


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()

