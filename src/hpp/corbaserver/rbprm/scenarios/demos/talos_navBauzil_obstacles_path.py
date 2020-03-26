from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner


class PathPlanner(TalosPathPlanner):
    def load_rbprm(self):
        from talos_rbprm.talos_abstract import Robot
        Robot.urdfName = "talos_trunk_large"  # load the model with conservative bounding boxes for trunk
        self.robot_node_name = "talos_trunk_large"
        self.rbprmBuilder = Robot()

    def init_problem(self):
        super().init_problem()
        # greatly increase the number of loops of the random shortcut
        self.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 100)
        # force the base orientation to follow the direction of motion along the Z axis
        self.ps.setParameter("Kinodynamic/forceYawOrientation", True)

    def run(self):
        self.init_problem()
        self.root_translation_bounds = [-1.5, 4, 0., 3.3, self.rbprmBuilder.ref_height, self.rbprmBuilder.ref_height]
        self.set_joints_bounds()

        self.q_init[:2] = [-0.9, 1.7]
        # Constraint the initial orientation when forceYawOrientation = True, expressed as a 3D vector (x,y,z)
        self.q_init[-6:-3] = [0.07, 0, 0]
        self.q_goal[:2] = [2, 2.6]

        self.init_viewer("multicontact/floor_bauzil_obstacles", visualize_affordances=["Support"],
                         reduce_sizes=[0.02, 0., 0.])
        self.init_planner()
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
