from hpp.corbaserver.rbprm.scenarios.anymal_path_planner import AnymalPathPlanner
from pinocchio import Quaternion
import numpy as np
import random
import sys


class PathPlanner(AnymalPathPlanner):

    status_filename = "/res/infos.log"

    def __init__(self):
        super().__init__()
        self.v_max = 0.5
        self.a_max = 0.05
        self.v_init = 0.05
        self.v_goal = 0.01
        self.radius = 1.

    def init_problem(self):
        super().init_problem()
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)

    def set_random_configs(self):
        """
        randomly sample initial and goal configuration :
        """
        self.q_init[:3] = [0, 0, 0.465]
        self.q_init[3:7] = [0, 0, 0, 1]
        self.q_init[-6] = self.v_init

        random.seed()
        alpha = random.uniform(0., 2. * np.pi)
        #alpha = 4.
        print("Test on a circle, alpha = ", alpha)
        self.q_goal = self.q_init[::]
        self.q_goal[:3] = [self.radius * np.sin(alpha), -self.radius * np.cos(alpha), 0.465]
        # set final orientation to be along the circle :
        vx = np.matrix([1, 0, 0]).T
        v_goal = np.matrix([self.q_goal[0], self.q_goal[1], 0]).T
        quat = Quaternion.FromTwoVectors(vx, v_goal)
        self.q_goal[3:7] = quat.coeffs().tolist()
        # set final velocity to fix the orientation :
        self.q_goal[-6] = self.v_goal * np.sin(alpha)
        self.q_goal[-5] = -self.v_goal * np.cos(alpha)
        self.v(self.q_init)
        print("initial root position : ", self.q_init)
        print("final root position : ", self.q_goal)
        self.ps.setInitialConfig(self.q_init)
        self.ps.addGoalConfig(self.q_goal)
        self.alpha = alpha
        # write problem in files :
        with open(self.status_filename, "w") as f:
            f.write("q_init= " + str(self.q_init) + "\n")
            f.write("q_goal= " + str(self.q_goal) + "\n")

    def run(self):
        self.init_problem()
        self.init_viewer("multicontact/ground")
        self.set_random_configs()
        self.init_planner(optimize=False)
        success = self.ps.client.problem.prepareSolveStepByStep()
        if not success:
            print("planning failed.")
            sys.exit(1)
        self.ps.client.problem.finishSolveStepByStep()
        self.display_path()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
