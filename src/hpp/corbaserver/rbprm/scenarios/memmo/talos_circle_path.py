from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner
import numpy as np
import random
import sys


class PathPlanner(TalosPathPlanner):
    def __init__(self):
        super().__init__()
        self.status_filename = "/res/infos.log"
        self.v_max = 0.5
        self.a_max = 0.5
        self.radius = 0.3

    def set_random_configs(self):
        """
        randomly sample initial and goal configuration :
        """
        self.q_init[:3] = [0, 0, 1.0]
        self.q_init[3:7] = [0, 0, 0, 1]

        random.seed()
        alpha = random.uniform(0.0, 2.0 * np.pi)
        print("Test on a circle, alpha = ", alpha)
        self.q_goal = self.q_init[::]
        self.q_goal[:3] = [
            self.radius * np.sin(alpha),
            -self.radius * np.cos(alpha),
            1.0,
        ]

        print("initial root position : ", self.q_init[:3])
        print("final root position : ", self.q_goal[:3])
        self.ps.setInitialConfig(self.q_init)
        self.ps.addGoalConfig(self.q_goal)

        # write problem in files :
        with open(self.status_filename, "w") as f:
            f.write("q_init= " + str(self.q_init) + "\n")
            f.write("q_goal= " + str(self.q_goal) + "\n")

    def run(self):
        self.init_problem()
        self.init_viewer("multicontact/ground", visualize_affordances=["Support"])
        self.set_random_configs()
        self.init_planner()
        success = self.ps.client.problem.prepareSolveStepByStep()
        if not success:
            print("planning failed.")
            sys.exit(1)
        self.ps.client.problem.finishSolveStepByStep()

        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
