from .abstract_contact_generator import AbstractContactGenerator
import time


class HRP2ContactGenerator(AbstractContactGenerator):
    def __init__(self, path_planner):
        super().__init__(path_planner)
        self.robustness = 1
        self.robot_node_name = "hrp2_14"

    def load_fullbody(self):
        from hpp.corbaserver.rbprm.hrp2 import Robot
        self.fullbody = Robot()
        self.q_ref = self.fullbody.referenceConfig[::] + [0] * self.path_planner.extra_dof
        self.weight_postural = self.fullbody.postureWeights[::] + [0] * self.path_planner.extra_dof
        self.fullbody.limbs_names=[self.fullbody.rLegId, self.fullbody.lLegId]


    def set_joints_bounds(self):
        super().set_joints_bounds()
        # increase min bounds on knee, required to stay in the 'comfort zone' of the stabilizer
        self.fullbody.setJointBounds("LLEG_JOINT3", [0.4, 2.61799])
        self.fullbody.setJointBounds("RLEG_JOINT3", [0.4, 2.61799])

    def init_viewer(self):
        super().init_viewer()
        self.v.addLandmark('hrp2_14/base_link', 0.3)
