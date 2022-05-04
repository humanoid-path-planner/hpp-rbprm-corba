from .abstract_path_planner import AbstractPathPlanner


class HRP2PathPlanner(AbstractPathPlanner):
    def __init__(self, context=None):
        super().__init__(context)
        # set default bounds to a large workspace on x,y with small interval around
        # reference z value.
        self.root_translation_bounds = [-5.0, 5.0, -5.0, 5.0, 0.5, 0.8]
        # set default used limbs to be both feet
        self.used_limbs = ["hrp2_rleg_rom", "hrp2_lleg_rom"]
        self.size_foot_x = 0.18  # size of the feet along the x axis
        self.size_foot_y = 0.1  # size of the feet along the y axis
        self.v_max = 0.2
        self.a_max = 0.1
        self.extra_dof_bounds = [
            -self.v_max,
            self.v_max,
            -self.v_max,
            self.v_max,
            0,
            0,
            -self.a_max,
            self.a_max,
            -self.a_max,
            self.a_max,
            0,
            0,
        ]
        self.robot_node_name = "hrp2_trunk_flexible"

    def load_rbprm(self):
        from hrp2_rbprm.hrp2_abstract import Robot

        self.rbprmBuilder = Robot(client=self.hpp_client, clientRbprm=self.rbprm_client)
