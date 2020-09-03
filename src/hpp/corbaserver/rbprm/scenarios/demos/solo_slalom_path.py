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

        self.q_init[:2] = [-0.7, 0.95]
        #self.q_goal[:2] = [1.05, 0.95]
        self.q_goal[:2] = [-0.2, 0.95]

        # Constraint the initial orientation when forceYawOrientation = True, expressed as a 3D vector (x,y,z)
        #self.q_init[-6:-3] = [0.1, 0, 0]
        #self.q_goal[-6:-3] = [0, -0.1, 0]

        self.init_viewer("multicontact/slalom_debris", visualize_affordances=["Support"], reduce_sizes=[0.05, 0, 0])
        self.init_planner(True, False)
        self.solve()

        """
        ps = self.ps
        q_init = self.rbprmBuilder.getCurrentConfig();
        q_init[0:3] = [-1.8, 0., self.rbprmBuilder.ref_height];
        self.v(q_init)
        q_init[-6:-3] = [0.1, 0, 0]
        q_goal = q_init[::]
        q_goal[0:3] = [-0.9, 0.95, self.rbprmBuilder.ref_height];
        self.v(q_goal)
        q_goal[-6:-3] = [0.1, 0, 0]
        ps.setInitialConfig(q_init)
        ps.addGoalConfig(q_goal)
        self.v(q_goal)

        q_init_0 = q_init[::]
        t = ps.solve()
        print("done planning, optimize path ...")
        # v.solveAndDisplay('rm',2,0.005)
        # for i in range(5):
        #  ps.optimizePath(ps.numberPaths() -1)

        pId_begin = ps.numberPaths() - 1
        ### END BEGIN up to the rubbles #####
        ps.resetGoalConfigs()
        ### BEGIN rubbles #####
        ps.setParameter("Kinodynamic/velocityBound", 0.4)
        ps.setParameter("Kinodynamic/accelerationBound", 0.1)
        q_init = self.rbprmBuilder.getCurrentConfig();
        q_init = q_goal[::];
        self.v(q_init)
        # q_init[-6:-3] = [0.,0,0]
        q_goal[0:3] = [1.05, 0.95, self.rbprmBuilder.ref_height];
        self.v(q_goal)
        q_goal[-6:-3] = [0.1, 0, 0]
        ps.setInitialConfig(q_init)
        ps.addGoalConfig(q_goal)
        self.v(q_goal)

        t = ps.solve()
        print("done planning, optimize path ...")
        # v.solveAndDisplay('rm',2,0.005)
        for i in range(10):
          ps.optimizePath(ps.numberPaths() -1)

        pId_rubbles = ps.numberPaths() - 1
        ### END rubbles #####
        ps.resetGoalConfigs()
        ### BEGIN after rubbles #####
        ps.setParameter("Kinodynamic/velocityBound", 0.15)
        ps.setParameter("Kinodynamic/accelerationBound", 0.1)
        q_init = self.rbprmBuilder.getCurrentConfig();
        q_init = q_goal[::];
        self.v(q_init)
        q_goal[0:3] = [2.2, 0, self.rbprmBuilder.ref_height];
        self.v(q_goal)
        q_goal[-6:-3] = [0.05, 0, 0]
        ps.setInitialConfig(q_init)
        ps.addGoalConfig(q_goal)
        self.v(q_goal)

        t = ps.solve()
        print("done planning, optimize path ...")
        # v.solveAndDisplay('rm',2,0.005)
        # for i in range(5):
        #  ps.optimizePath(ps.numberPaths() -1)

        pId_end = ps.numberPaths() - 1
        ### END after rubbles #####
        self.pathId = pId_begin
        ps.concatenatePath(self.pathId, pId_rubbles)
        ps.concatenatePath(self.pathId, pId_end)
        """

        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
