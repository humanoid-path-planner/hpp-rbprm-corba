from scenarios.hrp2_path_planner import HRP2PathPlanner

class PathPlanner(HRP2PathPlanner):

    def run(self):
        self.v_max = 0.3
        self.root_translation_bounds = [-5,5, -1.5, 1.5, 0.70, 0.8]
        self.init_problem()
        self.q_init[0:3] = [0.12, 0.25, 0.75]
        self.q_goal[0:3] = [1.08, 0.25, 0.75]
        self.init_viewer("multicontact/plateforme_surfaces",reduce_sizes=[0.12,0,0], visualize_affordances=["Support"])
        self.init_planner()
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()

