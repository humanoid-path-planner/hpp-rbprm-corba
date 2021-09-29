from .abstract_path_planner import AbstractPathPlanner


class SoloPathPlanner(AbstractPathPlanner):
    def __init__(self, context = None):
        super().__init__(context)
        # set default bounds to a large workspace on x,y with small interval around reference z value
        self.root_translation_bounds = [-5., 5., -5., 5., 0.241, 0.241]
        # set default used limbs to be both feet
        self.used_limbs = ['solo_RFleg_rom','solo_LHleg_rom','solo_LFleg_rom','solo_RHleg_rom']
        self.size_foot_x = 0.01  # size of the feet along the x axis
        self.size_foot_y = 0.01  # size of the feet along the y axis
        self.v_max = 0.3
        self.a_max = 1.
        self.extra_dof_bounds = [
            -self.v_max, self.v_max, -self.v_max, self.v_max, 0, 0, -self.a_max, self.a_max, -self.a_max, self.a_max,
            0, 0
        ]
        self.robot_node_name = "solo_trunk"

    def load_rbprm(self):
        from solo_rbprm.solo_abstract import Robot
        self.rbprmBuilder = Robot(client=self.hpp_client, clientRbprm=self.rbprm_client)

    def set_joints_bounds(self):
        super().set_joints_bounds()

    def set_configurations(self):
        super().set_configurations()
