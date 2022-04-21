from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner
import numpy as np
import random


class PathPlanner(TalosPathPlanner):

    status_filename = "/res/infos.log"
    """
    # rectangle used to sample position of the floor
    x_floor=[-0.9,0.05]
    y_floor=[0.6,2.]
    z_floor=0.98
    # rectangle used to sample position on the plateform
    x_plat=[1.9,2.55]
    y_plat=[-0.13,1.53]
    z_plat=1.58
    """
    # rectangle used to sample position of the floor
    x_floor = [-0.9, 0.05]
    y_floor = [0.8, 1.50]
    z_floor = self.rbprmBuilder.ref_height - 0.02
    # rectangle used to sample position on the plateform
    x_plat = [1.9, 2.55]
    y_plat = [0.8, 1.50]
    z_plat = self.rbprmBuilder.ref_height + 0.6
    v_predef = 0.05
    vx = np.matrix([1, 0, 0]).T

    def __init__(self):
        super().__init__()
        self.v_max = 0.3
        self.a_max = 0.05

    def load_rbprm(self):
        from talos_rbprm.talos_abstract import Robot
        # select ROM model with really conservative ROM shapes
        Robot.urdfName = "talos_trunk_large_reducedROM"
        self.robot_node_name = "talos_trunk_large_reducedROM"
        Robot.rLegId = 'talos_rleg_rom_reduced'
        Robot.lLegId = 'talos_lleg_rom_reduced'
        Robot.urdfNameRom = [Robot.lLegId, Robot.rLegId]
        self.used_limbs = Robot.urdfNameRom
        self.rbprmBuilder = Robot()
        # As the ROM names have changed, we need to set this values again (otherwise it's automatically done)
        self.rbprmBuilder.setReferenceEndEffector(Robot.lLegId, self.rbprmBuilder.ref_EE_lLeg)
        self.rbprmBuilder.setReferenceEndEffector(Robot.rLegId, self.rbprmBuilder.ref_EE_rLeg)

    def set_random_configs(self):
        from hpp.corbaserver.rbprm.tools.sampleRotation import sampleRotationForConfig
        q_up = self.q_init[::]
        q_down = q_up[::]
        # generate a random problem : (q_init, q_goal)
        random.seed()
        go_up = random.randint(0, 1)
        if go_up:
            print("go upstair")
            alphaBounds = [np.pi / 4., 3. * np.pi / 4.]
        else:
            print("go downstair")
            alphaBounds = [5. * np.pi / 4., 7. * np.pi / 4.]

        # sample random valid position on the floor :
        while not self.rbprmBuilder.isConfigValid(q_down)[0]:
            q_down[0] = random.uniform(self.x_floor[0], self.x_floor[1])
            q_down[1] = random.uniform(self.y_floor[0], self.y_floor[1])
            q_down[2] = self.z_floor
            # sample random orientation :
            q_down = sampleRotationForConfig(alphaBounds, q_down, self.v_predef)
            self.v(q_down)
        print("q_down found : ", q_down)
        # sample random valid position on the platform :
        while not self.rbprmBuilder.isConfigValid(q_up)[0]:
            q_up[0] = random.uniform(self.x_plat[0], self.x_plat[1])
            q_up[1] = random.uniform(self.y_plat[0], self.y_plat[1])
            q_up[2] = self.z_plat
            # sample random orientation :
            q_up = sampleRotationForConfig(alphaBounds, q_up, self.v_predef)
            self.v(q_up)
        print("q_up found : ", q_up)
        if go_up:
            self.q_init = q_down
            self.q_goal = q_up
        else:
            self.q_init = q_up
            self.q_goal = q_down
        print("q_init= " + str(self.q_init))
        print("q_goal= " + str(self.q_goal))
        # write problem in files :
        with open(self.status_filename, "w") as f:
            f.write("q_init= " + str(self.q_init) + "\n")
            f.write("q_goal= " + str(self.q_goal) + "\n")

    def init_problem(self):
        super().init_problem()
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)
        # greatly increase the number of loops of the random shortcut
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 100)
        self.ps.setParameter("Kinodynamic/synchronizeVerticalAxis", True)
        self.ps.setParameter("Kinodynamic/verticalAccelerationBound", 3.)

    def compute_extra_config_bounds(self):
        # bounds for the extradof : by default use v_max/a_max on x and y axis and a large value on z axis
        self.extra_dof_bounds = [
            -self.v_max, self.v_max, -self.v_max, self.v_max, -10, 10, -self.a_max, self.a_max, -self.a_max,
            self.a_max, -10, 10
        ]

    def run(self):
        self.root_translation_bounds = [-0.9, 2.55, -0.13, 2., z_floor - 0.05, z_plat + 0.05]
        self.init_problem()
        self.init_viewer("multicontact/bauzil_stairs", reduce_sizes=[0.11, 0., 0.], visualize_affordances=["Support"])
        self.set_random_configs()
        self.init_planner()
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
