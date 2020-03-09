from hpp.corbaserver.rbprm.scenarios.anymal_path_planner import AnymalPathPlanner
from pinocchio import Quaternion
import numpy as np
import random
import sys


class PathPlanner(AnymalPathPlanner):

    status_filename = "/res/infos.log"

    X_BOUNDS = [0.4, 3.6]
    Y_BOUNDS = [0.4, 2.]
    Z_VALUE = 0.465

    def __init__(self):
        super().__init__()
        self.v_max = 0.5
        self.a_max = 0.5
        self.v_init = 0.05  # initial / final velocity to fix the direction
        self.v_goal = 0.01

    def init_problem(self):
        super().init_problem()
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)

    def set_rom_filters(self):
        super().set_rom_filters()
        # TEMP fix, because of issue https://github.com/humanoid-path-planner/hpp-fcl/issues/134 in hpp-fcl
        # we need to disable ROM checks in this scenario with really small contact surfaces
        self.rbprmBuilder.setFilter([])

    def set_random_configs(self):
        """
        randomly sample initial and goal configuration :
        """
        random.seed()
        self.q_init[:3] = [
            random.uniform(self.X_BOUNDS[0], self.X_BOUNDS[1]),
            random.uniform(self.Y_BOUNDS[0], self.Y_BOUNDS[1]), self.Z_VALUE
        ]
        self.q_goal = self.q_init[::]
        for i in range(random.randint(0, 1000)):
            random.uniform(0., 1.)
        self.q_goal[:3] = [
            random.uniform(self.X_BOUNDS[0], self.X_BOUNDS[1]),
            random.uniform(self.Y_BOUNDS[0], self.Y_BOUNDS[1]), self.Z_VALUE
        ]

        # compute the orientation such that q_init face q_goal :
        # set final orientation to be along the circle :
        vx = np.matrix([1, 0, 0]).T
        v_init = np.matrix([self.q_goal[0] - self.q_init[0], self.q_goal[1] - self.q_init[1], 0]).T
        quat = Quaternion.FromTwoVectors(vx, v_init)
        self.q_init[3:7] = quat.coeffs().tolist()
        self.q_goal[3:7] = self.q_init[3:7]
        self.v(self.q_init)
        print("initial root position : ", self.q_init[:3])
        print("final root position : ", self.q_goal[:3])
        self.ps.setInitialConfig(self.q_init)
        self.ps.addGoalConfig(self.q_goal)

        # write problem in files :
        f = open(self.status_filename, "w")
        f.write("q_init= " + str(self.q_init) + "\n")
        f.write("q_goal= " + str(self.q_goal) + "\n")
        f.close()

    def run(self):
        self.init_problem()
        self.init_viewer("multicontact/plateforme_not_flat", reduce_sizes=[0.03, 0, 0])
        self.set_random_configs()
        self.init_planner(kinodynamic=False, optimize=False)
        success = self.ps.client.problem.prepareSolveStepByStep()
        if not success:
            print("planning failed.")
            sys.exit(1)
        self.ps.client.problem.finishSolveStepByStep()
        self.display_path()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
