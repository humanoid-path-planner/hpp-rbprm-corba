from .template_path import AbstractPathPlanning


class TalosPathPlanning(AbstractPathPlanning):

    def __init__(self):
        super().__init__()
        # set default bounds to a large workspace on x,y with small interval around reference z value
        self.root_translation_bounds = [-5., 5., -5., 5., 0.95, 1.05]
        # set default used limbs to be both feet
        self.used_limbs = ['talos_lleg_rom','talos_rleg_rom']
        self.size_foot_x = 0.2 # size of the feet along the x axis
        self.size_foot_y = 0.12  # size of the feet along the y axis
        self.v_max = 0.3
        self.a_max = 0.1
        self.extra_dof_bounds = [-self.v_max, self.v_max, -self.v_max, self.v_max, 0, 0,
                                 -self.a_max, self.a_max, -self.a_max, self.a_max, 0, 0]

    def load_robot(self):
        from talos_rbprm.talos_abstract import Robot
        self.rbprmBuilder = Robot()
        self.rbprmBuilder.client.robot.setDimensionExtraConfigSpace(self.extra_dof)
        self.q_init = self.rbprmBuilder.getCurrentConfig()
        self.q_goal = self.rbprmBuilder.getCurrentConfig()
        self.q_init[2] = self.rbprmBuilder.ref_height
        self.q_goal[2] = self.rbprmBuilder.ref_height






