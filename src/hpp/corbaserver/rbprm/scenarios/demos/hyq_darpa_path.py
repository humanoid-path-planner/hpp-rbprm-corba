from hpp.corbaserver.rbprm.scenarios.hyq_path_planner import HyqPathPlanner


class PathPlanner(HyqPathPlanner):
    def init_problem(self):
        super().init_problem()
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 100)

    def run(self):
        self.root_translation_bounds = [-2, 5, -1, 1, 0.3, 4]
        self.root_rotation_bounds = [-0.4, 0.4, -0.3, 0.3, -0.3, 0.3]
        self.init_problem()

        self.q_init[0:2] = [-2, 0]
        self.q_goal[0:2] = [3, 0]
        self.init_viewer("multicontact/darpa", visualize_affordances=["Support"], reduce_sizes=[0.06, 0, 0])
        self.init_planner(kinodynamic=False)
        self.solve()
        print("Optimize path ...")
        for i in range(1, 6):
            self.rbprmBuilder.client.problem.optimizePath(i)
            print("Optimization pass " + str(i) + "/5 done.")
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
