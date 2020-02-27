from scenarios.talos_path_planner import TalosPathPlanner

class PathPlanner(TalosPathPlanner):

    def run(self):
        self.init_problem()

        self.q_init[0:2] = [0, 0]
        self.q_goal[0:2] = [1, 0]

        self.init_viewer("multicontact/ground", visualize_affordances=["Support"])
        self.init_planner()
        self.solve()
        self.display_path()
        #self.play_path()
        self.hide_rom()

if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()