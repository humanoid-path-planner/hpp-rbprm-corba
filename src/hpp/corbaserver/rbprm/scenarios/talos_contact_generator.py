from .abstract_contact_generator import AbstractContactGenerator
import time


class TalosContactGenerator(AbstractContactGenerator):
    def __init__(self, path_planner):
        super().__init__(path_planner)
        self.robustness = 2
        self.robot_node_name = "talos"

    def load_fullbody(self):
        from talos_rbprm.talos import Robot
        self.fullbody = Robot()
        self.q_ref = self.fullbody.referenceConfig[::] + [0] * self.path_planner.extra_dof
        self.weight_postural = self.fullbody.postureWeights[::] + [0] * self.path_planner.extra_dof

    def init_viewer(self):
        super().init_viewer()
        self.v.addLandmark('talos/base_link', 0.3)
