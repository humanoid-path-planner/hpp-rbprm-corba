from hpp.corbaserver.rbprm.scenarios.hyq_path_planner import HyqPathPlanner


class PathPlanner(HyqPathPlanner):
    def init_problem(self):
        super().init_problem()
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 100)
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)

    def run(self):
        self.v_max = 0.2
        self.a_max = 0.1
        self.root_translation_bounds = [-1.7, 2.5, -0.2, 2, 0.64, 0.64]
        self.init_problem()

        self.q_init[0:2] = [-1.5, 0]
        self.q_init[-6] = 0.05
        self.q_goal[0:2] = [2.2, 0]
        self.q_goal[-6] = 0.05

        self.init_viewer("multicontact/slalom_debris")
        self.init_planner()
        self.solve()
        self.ps.optimizePath(1)
        self.display_path()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
