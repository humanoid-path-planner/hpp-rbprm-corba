from hpp.corbaserver.rbprm.scenarios.solo_path_planner import SoloPathPlanner


class PathPlanner(SoloPathPlanner):

    def init_problem(self):
        self.a_max = 0.1
        super().init_problem()
        # greatly increase the number of loops of the random shortcut
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 50)
        # force the base orientation to follow the direction of motion along the Z axis
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)


    def run(self):
        self.init_problem()

        self.q_init[:2] = [0, 0]
        self.q_goal[:2] = [0.5, 0]

        # Constraint the initial orientation when forceYawOrientation = True, expressed as a 3D vector (x,y,z)
        #self.q_init[-6:-3] = [0.1, 0, 0]
        #self.q_goal[-6:-3] = [0, -0.1, 0]

        self.init_viewer("multicontact/ground", visualize_affordances=["Support"])
        self.init_planner(True, False)
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
