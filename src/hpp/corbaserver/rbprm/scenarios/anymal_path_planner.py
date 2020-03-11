from .abstract_path_planner import AbstractPathPlanner


class AnymalPathPlanner(AbstractPathPlanner):
    def __init__(self):
        super().__init__()
        # set default bounds to a large workspace on x,y with small interval around reference z value
        self.root_translation_bounds = [-5., 5., -5., 5., 0.4, 0.5]
        # set default used limbs to be both feet
        self.used_limbs = ['anymal_RFleg_rom', 'anymal_LHleg_rom', 'anymal_LFleg_rom', 'anymal_RHleg_rom']
        self.size_foot_x = 0.01  # size of the feet along the x axis
        self.size_foot_y = 0.01  # size of the feet along the y axis
        self.v_max = 0.3
        self.a_max = 1.
        self.extra_dof_bounds = [
            -self.v_max, self.v_max, -self.v_max, self.v_max, 0, 0, -self.a_max, self.a_max, -self.a_max, self.a_max,
            0, 0
        ]
        self.robot_node_name = "anymal_trunk"

    def load_rbprm(self):
        from hpp.corbaserver.rbprm.anymal_abstract import Robot
        self.rbprmBuilder = Robot()

    def set_joints_bounds(self):
        super().set_joints_bounds()

    def set_configurations(self):
        super().set_configurations()
