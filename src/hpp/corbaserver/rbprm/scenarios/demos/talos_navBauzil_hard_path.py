from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner


class PathPlanner(TalosPathPlanner):
    def load_rbprm(self):
        from talos_rbprm.talos_abstract import Robot
        Robot.urdfName += "_large"  # load the model with conservative bounding boxes for trunk
        self.rbprmBuilder = Robot()

    def init_problem(self):
        super().init_problem()
        # greatly increase the number of loops of the random shortcut
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 50)
        # force the base orientation to follow the direction of motion along the Z axis
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)

    def run(self):
        self.init_problem()
        self.root_translation_bounds = [
            -2.3, 4.6, -1.5, 3.3, self.rbprmBuilder.ref_height, self.rbprmBuilder.ref_height
        ]
        self.set_joints_bounds()

        self.q_init[:2] = [-0.7, 2]
        self.q_goal[:2] = [0, -1]

        self.init_viewer("multicontact/floor_bauzil", visualize_affordances=["Support"])
        self.init_planner()
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
