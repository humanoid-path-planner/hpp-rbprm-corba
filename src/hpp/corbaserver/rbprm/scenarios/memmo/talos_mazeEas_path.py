from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner
import numpy as np


class PathPlanner(TalosPathPlanner):
    def __init__(self):
        super().__init__()
        self.status_filename = "/res/infos.log"
        self.v_max = 0.3
        self.a_max = 0.2
        self.root_translation_bounds = [0, 18.5, 0., 24., 0.98, 0.98]

    def load_rbprm(self):
        from talos_rbprm.talos_abstract import Robot
        # select model with conservative collision geometry for trunk
        Robot.urdfName += "_large"
        self.rbprmBuilder = Robot()

    def init_problem(self):
        super().init_problem()
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 500)
        self.ps.setMaxIterPathPlanning(100000)

    def set_random_configs(self):
        """
        randomly sample initial and goal configuration :
        """
        from hpp.corbaserver.rbprm.tools.sample_root_config import generate_random_conf_without_orientation
        self.q_init = generate_random_conf_without_orientation(self.rbprmBuilder, self.root_translation_bounds)
        self.q_goal = generate_random_conf_without_orientation(self.rbprmBuilder, self.root_translation_bounds)
        print("q_init= " + str(self.q_init))
        print("q_goal= " + str(self.q_goal))
        # write problem in files :
        f = open(self.status_filename, "w")
        f.write("q_init= " + str(self.q_init) + "\n")
        f.write("q_goal= " + str(self.q_goal) + "\n")
        f.close()

    def run(self):
        self.init_problem()
        self.init_viewer("multicontact/maze_easy", visualize_affordances=["Support"])
        self.set_random_configs()
        self.init_planner()
        self.solve()
        for i in range(10):
            print("Optimize path, " + str(i + 1) + "/10 ... ")
            self.ps.optimizePath(self.ps.numberPaths() - 1)

        self.ps.extractPath(self.ps.numberPaths() - 1, 0.1, self.ps.pathLength(self.ps.numberPaths() - 1) - 0.1)
        pathId = self.ps.numberPaths() - 1
        self.q_init = self.ps.configAtParam(pathId, 0)
        self.q_goal = self.ps.configAtParam(pathId, self.ps.pathLength(pathId))
        self.display_path()
        #self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
