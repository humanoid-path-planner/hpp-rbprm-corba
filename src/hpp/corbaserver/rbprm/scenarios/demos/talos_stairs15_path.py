from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner


class PathPlanner(TalosPathPlanner):
    def load_rbprm(self):
        from talos_rbprm.talos_abstract import Robot
        Robot.urdfNameRom = [Robot.lLegId, Robot.rLegId, Robot.rArmId]
        self.used_limbs = Robot.urdfNameRom
        self.rbprmBuilder = Robot()


    def init_problem(self):
        super().init_problem()
        # greatly increase the number of loops of the random shortcut
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 100)
        self.ps.setParameter("Kinodynamic/synchronizeVerticalAxis", True)
        self.ps.setParameter("Kinodynamic/verticalAccelerationBound", 3.)

    def compute_extra_config_bounds(self):
        # bounds for the extradof : by default use v_max/a_max on x and y axis and a large value on z axis
        self.extra_dof_bounds = [
            -self.v_max, self.v_max, -self.v_max, self.v_max, -10, 10, -self.a_max, self.a_max, -self.a_max,
            self.a_max, -10, 10
        ]

    def run(self):
        self.root_translation_bounds = [-2., -1.7, 0, 2., 0.95, 2.]
        self.init_problem()

        self.q_init[:2] = [-1.9, 1.9]
        self.q_init[3:7] = [0, 0, -0.7071, 0.7071]
        self.q_init[2] = 0.98
        self.q_goal = self.q_init[::]
        self.q_goal[1] = 0.9
        self.q_goal[2] += 3 * 0.15 # 4 steps of 15cm each

        self.init_viewer("multicontact/bauzil_stairs", reduce_sizes=[0.08, 0., 0.], visualize_affordances=["Support"])
        self.init_planner()
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
