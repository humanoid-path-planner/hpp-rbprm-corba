from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner
import math
import numpy as np
from numpy import random
from pinocchio import Quaternion


class PathPlanner(TalosPathPlanner):

    status_filename = "/res/infos.log"

    Y_BOUNDS = [0.3, 2.1]
    Z_VALUE = self.rbprmBuilder.ref_heigh - 0.02
    MAX_ANGLE = 0.4  # radian

    def __init__(self):
        super().__init__()
        self.v_max = 0.5
        self.a_max = 0.5
        self.v_init = 0.05
        self.v_goal = 0.01
        self.root_translation_bounds = [0.1, 4., 0.2, 2.2, 0.95, 1.05]

    def set_rom_filters(self):
        super().set_rom_filters()
        # TEMP fix, because of issue https://github.com/humanoid-path-planner/hpp-fcl/issues/134 in hpp-fcl
        # we need to disable ROM checks in this scenario with really small contact surfaces
        self.rbprmBuilder.setFilter([])

    def set_random_configs(self):
        """
        Generate random init and goal position.
        these position will be on the flat part of the environment,
        with an orientation such that they can be connected by a straight line,
        and an angle between +- 25 degree from the x axis
        """
        # select randomly the initial and final plateform, they cannot be the same
        # end plateform is always after the init plateform on the x axis
        init_plateform_id = random.randint(0, 3)
        end_plateform_id = random.randint(init_plateform_id + 1, 4)
        #if end_plateform_id >= init_plateform_id:
        #  end_plateform_id+=1

        # set x position from the plateform choosen :
        x_init = 0.16 + 0.925 * init_plateform_id
        x_goal = 0.16 + 0.925 * end_plateform_id

        # uniformly sample y position
        y_init = random.uniform(self.Y_BOUNDS[0], self.Y_BOUNDS[1])
        self.q_init[:3] = [x_init, y_init, self.Z_VALUE]

        # y_goal must be random inside Y_BOUNDS but such that the line between q_init and q_goal is between +- MAX_ANGLE radian from the x axis
        y_bound_goal = self.Y_BOUNDS[::]
        y_angle_max = math.sin(self.MAX_ANGLE) * abs(x_init - x_goal) + y_init
        y_angle_min = math.sin(-self.MAX_ANGLE) * abs(x_init - x_goal) + y_init
        y_bound_goal[0] = max(y_angle_min, y_bound_goal[0])
        y_bound_goal[1] = min(y_angle_max, y_bound_goal[1])
        y_goal = random.uniform(y_bound_goal[0], y_bound_goal[1])

        # compute the orientation such that q_init face q_goal :
        # set final orientation to be along the circle :
        vx = np.matrix([1, 0, 0]).T
        v_init = np.matrix([x_goal - x_init, y_goal - y_init, 0]).T
        quat = Quaternion.FromTwoVectors(vx, v_init)
        self.q_init[3:7] = quat.coeffs().tolist()

        self.q_goal = self.q_init[::]
        self.q_goal[:2] = [x_goal, y_goal]

        print("q_init= " + str(self.q_init))
        print("q_goal= " + str(self.q_goal))
        # write problem in files :
        with open(self.status_filename, "w") as f:
            f.write("q_init= " + str(self.q_init) + "\n")
            f.write("q_goal= " + str(self.q_goal) + "\n")

    def run(self):
        self.init_problem()
        self.init_viewer("multicontact/plateforme_not_flat",
                         reduce_sizes=[0.1, 0, 0],
                         visualize_affordances=["Support"])
        self.set_random_configs()
        self.init_planner(kinodynamic=False, optimize=False)
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
