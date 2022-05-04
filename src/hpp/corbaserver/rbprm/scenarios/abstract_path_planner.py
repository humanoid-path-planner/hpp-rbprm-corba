from abc import abstractmethod

from hpp.corbaserver import Client, ProblemSolver, createContext, loadServerPlugin
from hpp.corbaserver.affordance.affordance import AffordanceTool
from hpp.corbaserver.rbprm import Client as RbprmClient
from hpp.gepetto import PathPlayer, ViewerFactory


class AbstractPathPlanner:

    rbprmBuilder = None
    ps = None
    v = None
    afftool = None
    pp = None
    extra_dof_bounds = None
    robot_node_name = None  # name of the robot in the node list of the viewer

    def __init__(self, context=None):
        """
        Constructor
        :param context: An optional string that give a name to a corba context instance
        """
        # bounds on the linear velocity for the root, negative values mean unused
        self.v_max = -1
        # bounds on the linear acceleration for the root, negative values mean unused
        self.a_max = -1
        self.root_translation_bounds = [
            # bounds on the root translation position (-x, +x, -y, +y, -z, +z)
            0
        ] * 6
        # bounds on the rotation of the root (-z, z, -y, y, -x, x)
        self.root_rotation_bounds = [
            -3.14,
            3.14,
            -0.01,
            0.01,
            -0.01,
            0.01,
        ]
        # The rotation bounds are only used during the random sampling, they are not
        # enforced along the path.

        # number of extra config appended after the joints configuration, 6 to store
        # linear root velocity and acceleration.
        self.extra_dof = 6
        # friction coefficient between the robot and the environment
        self.mu = 0.5
        # names of the limbs that must be in contact during all the motion
        self.used_limbs = []
        self.size_foot_x = 0  # size of the feet along the x axis
        self.size_foot_y = 0  # size of the feet along the y axis
        self.q_init = []
        self.q_goal = []
        self.context = context
        if context:
            createContext(context)
            loadServerPlugin(context, "rbprm-corba.so")
            loadServerPlugin(context, "affordance-corba.so")
            self.hpp_client = Client(context=context)
            self.hpp_client.problem.selectProblem(context)
            self.rbprm_client = RbprmClient(context=context)
        else:
            self.hpp_client = None
            self.rbprm_client = None

    @abstractmethod
    def load_rbprm(self):
        """
        Build an rbprmBuilder instance for the correct robot and initialize it's extra
        config size.
        """
        pass

    def set_configurations(self):
        self.rbprmBuilder.client.robot.setDimensionExtraConfigSpace(self.extra_dof)
        self.q_init = self.rbprmBuilder.getCurrentConfig()
        self.q_goal = self.rbprmBuilder.getCurrentConfig()
        self.q_init[2] = self.rbprmBuilder.ref_height
        self.q_goal[2] = self.rbprmBuilder.ref_height

    def compute_extra_config_bounds(self):
        """
        Compute extra dof bounds from the current values of v_max and a_max
        By default, set symmetrical bounds on x and y axis and bounds z axis values to
        0.
        """
        # bounds for the extradof : by default use v_max/a_max on x and y axis and 0 on
        # z axis.
        self.extra_dof_bounds = [
            -self.v_max,
            self.v_max,
            -self.v_max,
            self.v_max,
            0,
            0,
            -self.a_max,
            self.a_max,
            -self.a_max,
            self.a_max,
            0,
            0,
        ]

    def set_joints_bounds(self):
        """
        Set the root translation and rotation bounds as well as the the extra dofs
        bounds.
        """
        self.rbprmBuilder.setJointBounds("root_joint", self.root_translation_bounds)
        self.rbprmBuilder.boundSO3(self.root_rotation_bounds)
        self.rbprmBuilder.client.robot.setExtraConfigSpaceBounds(self.extra_dof_bounds)

    def set_rom_filters(self):
        """
        Define which ROM must be in collision at all time and with which kind of
        affordances.
        By default it set all the roms in used_limbs to be in contact with 'support'
        affordances.
        """
        self.rbprmBuilder.setFilter(self.used_limbs)
        for limb in self.used_limbs:
            self.rbprmBuilder.setAffordanceFilter(limb, ["Support"])

    def init_problem(self):
        """
        Load the robot, set the bounds and the ROM filters and then
        Create a ProblemSolver instance and set the default parameters.
        The values of v_max, a_max, mu, size_foot_x and size_foot_y must be defined
        before calling this method.
        """
        self.load_rbprm()
        self.set_configurations()
        self.compute_extra_config_bounds()
        self.set_joints_bounds()
        self.set_rom_filters()
        self.ps = ProblemSolver(self.rbprmBuilder)
        # define parameters used by various methods :
        if self.v_max >= 0:
            self.ps.setParameter("Kinodynamic/velocityBound", self.v_max)
        if self.a_max >= 0:
            self.ps.setParameter("Kinodynamic/accelerationBound", self.a_max)
        if self.size_foot_x > 0:
            self.ps.setParameter("DynamicPlanner/sizeFootX", self.size_foot_x)
        if self.size_foot_y > 0:
            self.ps.setParameter("DynamicPlanner/sizeFootY", self.size_foot_y)
        self.ps.setParameter("DynamicPlanner/friction", 0.5)
        # sample only configuration with null velocity and acceleration :
        self.ps.setParameter("ConfigurationShooter/sampleExtraDOF", False)

    def init_viewer(
        self,
        env_name,
        env_package="hpp_environments",
        reduce_sizes=[0, 0, 0],
        visualize_affordances=[],
        min_area=None,
    ):
        """
        Build an instance of hpp-gepetto-viewer from the current problemSolver
        :param env_name: name of the urdf describing the environment
        :param env_package: name of the package containing this urdf (default to
        hpp_environments).
        :param reduce_sizes: Distance used to reduce the affordances plan toward the
        center of the plane.
        (in order to avoid putting contacts closes to the edges of the surface)
        :param visualize_affordances: list of affordances type to visualize, default to
        none.
        :param min_area: list of couple [affordanceType, size]. If provided set the
        minimal area for each affordance.
        """
        vf = ViewerFactory(self.ps)
        if self.context:
            self.afftool = AffordanceTool(context=self.context)
            # FIXME: this should be called in afftool constructor
            self.afftool.client.affordance.affordance.resetAffordanceConfig()
        else:
            self.afftool = AffordanceTool()
        self.afftool.setAffordanceConfig("Support", [0.5, 0.03, 0.00005])
        if min_area is not None:
            for (aff_type, min_size) in min_area:
                self.afftool.setMinimumArea(aff_type, min_size)
        self.afftool.loadObstacleModel(
            "package://" + env_package + "/urdf/" + env_name + ".urdf",
            "planning",
            vf,
            reduceSizes=reduce_sizes,
        )
        self.v = vf.createViewer(ghost=True, displayArrows=True)
        self.pp = PathPlayer(self.v)
        for aff_type in visualize_affordances:
            self.afftool.visualiseAffordances(aff_type, self.v, self.v.color.lightBrown)

    def init_planner(self, kinodynamic=True, optimize=True):
        """
        Select the rbprm methods, and the kinodynamic ones if required
        :param kinodynamic: if True, also select the kinodynamic methods
        :param optimize: if True, add randomShortcut path optimizer (or
        randomShortcutDynamic if kinodynamic is also True).
        """
        self.ps.selectConfigurationShooter("RbprmShooter")
        self.ps.selectPathValidation("RbprmPathValidation", 0.05)
        if kinodynamic:
            self.ps.selectSteeringMethod("RBPRMKinodynamic")
            self.ps.selectDistance("Kinodynamic")
            self.ps.selectPathPlanner("DynamicPlanner")
        if optimize:
            if kinodynamic:
                self.ps.addPathOptimizer("RandomShortcutDynamic")
            else:
                self.ps.addPathOptimizer("RandomShortcut")

    def solve(self, display_roadmap=False):
        """
        Solve the path planning problem.
        q_init and q_goal must have been defined before calling this method
        """
        if len(self.q_init) != self.rbprmBuilder.getConfigSize():
            raise ValueError("Initial configuration vector do not have the right size")
        if len(self.q_goal) != self.rbprmBuilder.getConfigSize():
            raise ValueError("Goal configuration vector do not have the right size")
        self.ps.setInitialConfig(self.q_init)
        self.ps.addGoalConfig(self.q_goal)
        self.v(self.q_init)
        if display_roadmap and self.v.client.gui.getNodeList() is not None:
            t = self.v.solveAndDisplay("roadmap", 5, 0.001)
        else:
            t = self.ps.solve()
        print("Guide planning time : ", t)

    def display_path(self, path_id=-1, dt=0.1):
        """
        Display the path in the viewer, if no path specified display the last one
        :param path_id: the Id of the path specified, default to the most recent one
        :param dt: discretization step used to display the path (default to 0.1)
        """
        if self.pp is not None:
            if path_id < 0:
                path_id = self.ps.numberPaths() - 1
            self.pp.dt = dt
            self.pp.displayVelocityPath(path_id)

    def play_path(self, path_id=-1, dt=0.01):
        """
        play the path in the viewer, if no path specified display the last one
        :param path_id: the Id of the path specified, default to the most recent one
        :param dt: discretization step used to display the path (default to 0.01)
        """
        self.show_rom()
        if self.pp is not None:
            if path_id < 0:
                path_id = self.ps.numberPaths() - 1
            self.pp.dt = dt
            self.pp(path_id)

    def hide_rom(self):
        """
        Remove the current robot from the display
        """
        self.v.client.gui.setVisibility(self.robot_node_name, "OFF")

    def show_rom(self):
        """
        Add the current robot to the display
        """
        self.v.client.gui.setVisibility(self.robot_node_name, "ON")

    @abstractmethod
    def run(self):
        """
        Must be defined in the child class to run all the methods with the correct
        arguments.
        """
        # example of definition:
        """
        self.init_problem()
        # define initial and goal position
        self.q_init[:2] = [0, 0]
        self.q_goal[:2] = [1, 0]

        self.init_viewer("multicontact/ground", visualize_affordances=["Support"])
        self.init_planner()
        self.solve()
        self.display_path()
        self.play_path()
        """
        pass
