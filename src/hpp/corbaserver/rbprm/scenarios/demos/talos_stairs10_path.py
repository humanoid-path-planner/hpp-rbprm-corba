from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner


class PathPlanner(TalosPathPlanner):
    def load_rbprm(self):
        from talos_rbprm.talos_abstract import Robot
        # select ROM model with really conservative ROM shapes
        Robot.urdfName = "talos_trunk_large_reducedROM"
        self.robot_node_name = "talos_trunk_large_reducedROM"
        Robot.rLegId = 'talos_rleg_rom_reduced'
        Robot.lLegId = 'talos_lleg_rom_reduced'
        Robot.urdfNameRom = [Robot.lLegId, Robot.rLegId]
        self.used_limbs = Robot.urdfNameRom
        self.rbprmBuilder = Robot()

        # As the ROM names have changed, we need to set this values again (otherwise it's automatically done)
        self.rbprmBuilder.setReferenceEndEffector(Robot.lLegId, self.rbprmBuilder.ref_EE_lLeg)
        self.rbprmBuilder.setReferenceEndEffector(Robot.rLegId, self.rbprmBuilder.ref_EE_rLeg)

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
        self.root_translation_bounds = [-1.5, 3, 0., 3.3, 0.95, 2.]
        self.init_problem()

        self.q_init[:2] = [0., 1.2]
        self.q_goal[:2] = [1.87, 1.2]
        self.q_goal[2] += 0.6

        self.init_viewer("multicontact/bauzil_stairs", reduce_sizes=[0.08, 0., 0.], visualize_affordances=["Support"])
        self.init_planner()
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
