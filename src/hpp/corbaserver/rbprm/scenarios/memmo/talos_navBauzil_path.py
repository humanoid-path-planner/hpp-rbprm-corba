from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner
import numpy as np
class PathPlanner(TalosPathPlanner):

    status_filename = "/res/infos.log"

    def load_rbprm(self):
        from talos_rbprm.talos_abstract import Robot
        Robot.urdfName += "_large" # load the model with conservative bounding boxes for trunk
        self.rbprmBuilder = Robot()

    def init_problem(self):
        super().init_problem()
        # greatly increase the number of loops of the random shortcut
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 100)
        # force the base orientation to follow the direction of motion along the Z axis
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)

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
        self.root_translation_bounds = [-1.5,3,0.,3.3, self.rbprmBuilder.ref_height, self.rbprmBuilder.ref_height]
        self.set_joints_bounds()
        self.init_viewer("multicontact/floor_bauzil", visualize_affordances=["Support"])
        self.set_random_configs()
        self.init_planner()
        self.solve()
        self.display_path()
        #self.play_path()
        self.hide_rom()

if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
