from hpp.corbaserver.rbprm.scenarios.solo_path_planner import SoloPathPlanner


class PathPlanner(SoloPathPlanner):

    def init_problem(self):
        self.a_max = 0.1
        super().init_problem()
        # greatly increase the number of loops of the random shortcut
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 100)
        # force the base orientation to follow the direction of motion along the Z axis
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)


    def run(self):
        self.init_problem()

        self.q_init[:2] = [-0.85, 0.]
       # self.q_goal[:2] = [1, 0.]
        self.q_goal[:2] = [0., 0.]


        self.init_viewer("ori/modular_palet_low", visualize_affordances=["Support"], reduce_sizes=[0.06, 0, 0],
                         min_area = [['Support', 0.05]])
        self.init_planner(True, False)
        self.ps.directPath(self.q_init, self.q_goal, False)


        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
