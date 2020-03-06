from hpp.corbaserver.rbprm.tools.sample_root_config import generate_random_conf_without_orientation
from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner
from talos_rbprm.talos_abstract import Robot


class PathPlanner(TalosPathPlanner):

    Z_FLOOR = Robot.ref_height
    Z_PLATFORM = Robot.ref_height + 0.6
    BOUNDS_ALL = [-2.6, 4.7, -1.5, 3.3, Z_FLOOR, Z_PLATFORM]
    BOUNDS_ALL_PLAT = [-2.1, 2.4, -0.1, 1.6, Z_PLATFORM, Z_PLATFORM]
    BOUNDS_FLOOR = [-2.6, 4.7, -1.5, 3.3, Z_FLOOR, Z_FLOOR]
    BOUNDS_PLAT_10 = [1.9, 2.4, -0.1, 1.6, Z_PLATFORM, Z_PLATFORM]
    BOUNDS_PLAT_15 = [-2.1, -1.5, -0.1, 0.5, Z_PLATFORM, Z_PLATFORM]

    def __init__(self):
        super().__init__()
        self.status_filename = "/res/infos.log"
        self.root_translation_bounds = self.BOUNDS_ALL
        self.v_max = 0.3
        self.a_max = 0.2

    def load_rbprm(self):
        # select model with conservative collision geometry for trunk
        Robot.urdfName += "_large_reducedROM"
        # select conservative ROM for feet
        Robot.urdfNameRom += ['talos_lleg_rom_reduced', 'talos_rleg_rom_reduced']
        self.rbprmBuilder = Robot()
        self.rbprmBuilder.setReferenceEndEffector('talos_lleg_rom_reduced', self.rbprmBuilder.ref_EE_lLeg)
        self.rbprmBuilder.setReferenceEndEffector('talos_rleg_rom_reduced', self.rbprmBuilder.ref_EE_rLeg)

    def compute_extra_config_bounds(self):
        # bounds for the extradof : by default use v_max/a_max on x and y axis and a large value on z axis
        self.extra_dof_bounds = [
            -self.v_max, self.v_max, -self.v_max, self.v_max, -2, 2, -self.a_max, self.a_max, -self.a_max, self.a_max,
            -3, 3
        ]

    def init_problem(self):
        super().init_problem()
        # greatly increase the number of loops of the random shortcut
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 500)
        # force the base orientation to follow the direction of motion along the Z axis
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)
        self.ps.setParameter("Kinodynamic/synchronizeVerticalAxis", True)
        self.ps.setParameter("Kinodynamic/verticalAccelerationBound", 3.)
        self.ps.setMaxIterPathPlanning(50000)

    def set_random_configs(self):
        """
        randomly sample initial and goal configuration :
        """
        import random
        random.seed()
        # sample an initial and goal zone : either floor, right platform or left platform. With more weight on the floor
        # 0,1,2 are floor, 3 is platform10 4 is platform15
        WEIGHT_FLOOR = 3
        plat_id_init = random.randint(0, WEIGHT_FLOOR + 1)
        plat_id_goal = random.randint(0, WEIGHT_FLOOR + 1)
        while plat_id_goal >= WEIGHT_FLOOR and plat_id_goal == plat_id_init:  # cannot have init and end on the same platform (too easy)
            plat_id_goal = random.randint(0, WEIGHT_FLOOR + 1)

        print(("platform id init : ", plat_id_init))
        print(("platform id goal : ", plat_id_goal))
        #plat_id_init = 3
        #plat_id_goal = 0
        if plat_id_init < WEIGHT_FLOOR:
            print("init of flat floor")
            init_bounds = self.BOUNDS_FLOOR[::]
        elif plat_id_init == WEIGHT_FLOOR:
            print("init on 10cm platform")
            init_bounds = self.BOUNDS_PLAT_10[::]
        elif plat_id_init == WEIGHT_FLOOR + 1:
            print("init on 15cm platform")
            init_bounds = self.BOUNDS_PLAT_15[::]
        else:
            print("Unknown case")

        if plat_id_goal < WEIGHT_FLOOR:
            print("goal of flat floor")
            goal_bounds = self.BOUNDS_FLOOR[::]
        elif plat_id_goal == WEIGHT_FLOOR:
            print("goal on 10cm platform")
            goal_bounds = self.BOUNDS_PLAT_10[::]
        elif plat_id_goal == WEIGHT_FLOOR + 1:
            print("goal on 15cm platform")
            goal_bounds = self.BOUNDS_PLAT_15[::]
        else:
            print("Unknown case")

        root_bounds = [0] * 6
        for i in range(3):
            root_bounds[i * 2] = min(init_bounds[i * 2], goal_bounds[i * 2])
            root_bounds[i * 2 + 1] = max(init_bounds[i * 2 + 1], goal_bounds[i * 2 + 1])

        self.rbprmBuilder.setJointBounds("root_joint", root_bounds)
        self.q_init = generate_random_conf_without_orientation(self.rbprmBuilder, init_bounds, self.v)
        self.q_goal = generate_random_conf_without_orientation(self.rbprmBuilder, goal_bounds, self.v)

        print(("init config : ", self.q_init))
        print(("goal config : ", self.q_goal))
        # write problem in files :
        f = open(self.status_filename, "w")
        f.write("q_init= " + str(self.q_init) + "\n")
        f.write("q_goal= " + str(self.q_goal) + "\n")
        f.close()

    def run(self):
        self.init_problem()
        self.init_viewer("multicontact/bauzil_stairs", visualize_affordances=["Support"], reduce_sizes=[0.12, 0., 0.])
        self.set_random_configs()
        self.init_planner()
        self.solve()
        print("Original path length : ", self.ps.pathLength(self.ps.numberPaths() - 1))
        # optimize several time the path
        for i in range(5):
            print(("Optimize path, " + str(i + 1) + "/5 ... "))
            self.ps.optimizePath(self.ps.numberPaths() - 1)
            print(("New path length : ", self.ps.pathLength(self.ps.numberPaths() - 1)))
        # remove the very beginning and end of the path to avoid orientation discontinuities
        self.ps.extractPath(self.ps.numberPaths() - 1, 0.05, self.ps.pathLength(self.ps.numberPaths() - 1) - 0.05)
        pathId = self.ps.numberPaths() - 1
        self.q_init = self.ps.configAtParam(pathId, 0)
        self.q_goal = self.ps.configAtParam(pathId, self.ps.pathLength(pathId))
        self.display_path()
        #self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
