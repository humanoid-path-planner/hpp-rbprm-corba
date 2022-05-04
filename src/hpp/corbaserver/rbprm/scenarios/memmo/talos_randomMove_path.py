from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner
import numpy as np
from pinocchio import Quaternion
import random
import sys


class PathPlanner(TalosPathPlanner):

    status_filename = "/res/infos.log"
    MIN_ROOT_DIST = 0.05
    MAX_ROOT_DIST = 0.3
    # if true : random value of yaw orientation, if false : constrained to be along the
    # direction of root_init->root_goal.
    FINAL_ORIENTATION_RANDOM = False
    ROOT_Z_MIN = 0.85
    ROOT_Z_MAX = 1.05

    def __init__(self):
        super().__init__()
        self.v_max = 0.5
        self.a_max = 0.05
        self.v_init = 0.01
        self.v_goal = 0.01
        self.root_translation_bounds = [-2, 2, -2, 2, 1.0, 1.0]

    def init_problem(self):
        super().init_problem()
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)

    def set_random_configs(self):
        """
        randomly sample initial and goal configuration :
        """
        # init position at the origin, facing x axis
        self.q_init[:3] = [0, 0, 1.0]
        self.q_init[-6] = self.v_init
        # sample random position on a circle of radius random in [MIN_ROOT_DIST;
        # MAX_ROOT_DIST].
        random.seed()
        radius = random.uniform(self.MIN_ROOT_DIST, self.MAX_ROOT_DIST)
        alpha = random.uniform(0.0, 2.0 * np.pi)
        print("Test on a circle, alpha = ", alpha)
        print("Radius = ", radius)
        self.q_goal = self.q_init[::]
        self.q_goal[:3] = [radius * np.sin(alpha), -radius * np.cos(alpha), 1.0]

        if self.FINAL_ORIENTATION_RANDOM:
            alpha = random.uniform(
                0.0, 2.0 * np.pi
            )  # sample new random yaw value for the orientation
            v_goal = np.matrix([radius * np.sin(alpha), -radius * np.cos(alpha), 0]).T
        else:
            v_goal = np.matrix(
                [self.q_goal[0], self.q_goal[1], 0]
            ).T  # direction root_init -> root_goal

        # set final orientation to be along the circle :
        vx = np.matrix([1, 0, 0]).T
        quat = Quaternion.FromTwoVectors(vx, v_goal)
        self.q_goal[3:7] = quat.coeffs().tolist()
        # set final velocity to fix the orientation :
        self.q_goal[-6] = self.v_goal * np.sin(alpha)
        self.q_goal[-5] = -self.v_goal * np.cos(alpha)
        self.ps.setInitialConfig(self.q_init)
        self.ps.addGoalConfig(self.q_goal)
        print("q_init= " + str(self.q_init))
        print("q_goal= " + str(self.q_goal))
        # write problem in files :
        with open(self.status_filename, "w") as f:
            f.write("q_init= " + str(self.q_init) + "\n")
            f.write("q_goal= " + str(self.q_goal) + "\n")

    def run(self):
        self.init_problem()
        self.set_joints_bounds()
        self.init_viewer("multicontact/ground", visualize_affordances=["Support"])
        self.set_random_configs()
        self.init_planner(optimize=False)
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
