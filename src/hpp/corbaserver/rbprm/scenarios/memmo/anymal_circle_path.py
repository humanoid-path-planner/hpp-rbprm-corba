from hpp.corbaserver.rbprm.scenarios.anymal_path_planner import AnymalPathPlanner
from pinocchio import Quaternion
import numpy as np


class PathPlanner(AnymalPathPlanner):

    status_filename = "/res/infos.log"

    def __init__(self):
        super().__init__()
        self.v_max = 0.5
        self.a_max = 0.5
        self.radius = 0.3

    def set_random_configs(self):
        """
        randomly sample initial and goal configuration :
        """
        self.q_init[0:3] = [0, 0, 0.465]
        self.q_init[3:7] = [0, 0, 0, 1]

        import random
        random.seed()
        alpha = random.uniform(0., 2. * np.pi)
        print("Test on a circle, alpha = ", alpha)
        self.q_goal = self.q_init[::]
        self.q_goal[0:3] = [self.radius * np.sin(alpha), -self.radius * np.cos(alpha), 0.465]
        self.v(self.q_init)
        print("initial root position : ", self.q_init[0:3])
        print("final root position : ", self.q_goal[0:3])
        self.ps.setInitialConfig(self.q_init)
        self.ps.addGoalConfig(self.q_goal)

        # write problem in files :
        f = open(self.status_filename, "w")
        f.write("q_init= " + str(self.q_init) + "\n")
        f.write("q_goal= " + str(self.q_goal) + "\n")
        f.close()

    def run(self):
        self.init_problem()
        self.init_viewer("multicontact/ground")
        self.set_random_configs()
        self.init_planner(optimize=False)
        success = self.ps.client.problem.prepareSolveStepByStep()
        if not success:
            print("planning failed.")
            import sys
            sys.exit(1)
        self.ps.client.problem.finishSolveStepByStep()
        self.display_path()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
