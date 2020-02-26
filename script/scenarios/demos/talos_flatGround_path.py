from scenarios.template_talos_path import TalosPathPlanning

class PathPlanning(TalosPathPlanning):

    def run(self):
        self.init_problem()

        self.q_init[0:2] = [0, 0]
        self.q_goal[0:2] = [1, 0]

        self.init_viewer("multicontact/ground", visualize_affordances=["Support"])
        self.init_planner()
        self.solve()
        self.display_path()
        self.play_path()


if __name__ == "__main__":
    PathPlanning().run()