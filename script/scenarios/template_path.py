from abc import abstractmethod
from hpp.gepetto import ViewerFactory, PathPlayer
from hpp.corbaserver.affordance.affordance import AffordanceTool
from hpp.corbaserver import ProblemSolver
import time

class AbstractPathPlanning:

  rbprmBuilder = None
  ps = None
  v = None
  afftool = None
  pp = None
  q_init = []
  q_goal = []

  def __init__(self):
    self.v_max = -1 # bounds on the linear velocity for the root, negative values mean unused
    self.a_max = -1 # bounds on the linear acceleration for the root, negative values mean unused
    self.root_translation_bounds = [0] * 6 # bounds on the root translation position (-x, +x, -y, +y, -z, +z)
    self.root_rotation_bounds = [-3.14, 3.14, -0.01, 0.01, -0.01, 0.01] # bounds on the rotation of the root (-z, z, -y, y, -x, x)
    # The rotation bounds are only used during the random sampling, they are not enforced along the path
    self.extra_dof = 6 # number of extra config appended after the joints configuration, 6 to store linear root velocity and acceleration
    self.mu = 0.5 # friction coefficient between the robot and the environment
    self.used_limbs = [] # names of the limbs that must be in contact during all the motion
    self.size_foot_x = 0 # size of the feet along the x axis
    self.size_foot_y = 0 # size of the feet along the y axis
    # bounds for the extradof : by default use v_max/a_max on x and y axis and 0 on z axis
    self.extra_dof_bounds = [-self.v_max,self.v_max,-self.v_max,self.v_max,0,0,
                             -self.a_max,self.a_max,-self.a_max,self.a_max,0,0]


  @abstractmethod
  def load_robot(self):
    """
    Build an rbprmBuilder instance for the correct robot and initialize it's extra config size
    """
    pass

  def set_joints_bounds(self):
    """
    Set the root translation and rotation bounds as well as the the extra dofs bounds
    """
    self.rbprmBuilder.setJointBounds ("root_joint", self.root_translation_bounds)
    self.rbprmBuilder.boundSO3(self.root_rotation_bounds)
    self.rbprmBuilder.client.robot.setExtraConfigSpaceBounds(self.extra_dof_bounds)

  def set_rom_filters(self):
    """
    Define which ROM must be in collision at all time and with which kind of affordances
    By default it set all the roms in used_limbs to be in contact with 'support' affordances
    """
    self.rbprmBuilder.setFilter(self.used_limbs)
    for limb in self.used_limbs:
      self.rbprmBuilder.setAffordanceFilter(limb, ['Support'])

  def init_problem(self):
    """
    Load the robot, set the bounds and the ROM filters and then
    Create a ProblemSolver instance and set the default parameters.
    The values of v_max, a_max, mu, size_foot_x and size_foot_y must be defined before calling this method
    """
    self.load_robot()
    self.set_joints_bounds()
    self.set_rom_filters()
    self.ps = ProblemSolver( self.rbprmBuilder)
    # define parameters used by various methods :
    if self.v_max >= 0:
      self.ps.setParameter("Kinodynamic/velocityBound", self.v_max)
    if self.a_max >= 0:
      self.ps.setParameter( "Kinodynamic/accelerationBound", self.a_max)
    if self.size_foot_x > 0:
      self.ps.setParameter("DynamicPlanner/sizeFootX", self.size_foot_x)
    if self.size_foot_y > 0:
      self.ps.setParameter("DynamicPlanner/sizeFootY", self.size_foot_y)
    self.ps.setParameter("DynamicPlanner/friction", 0.5)
    # sample only configuration with null velocity and acceleration :
    self.ps.setParameter("ConfigurationShooter/sampleExtraDOF", False)

  def init_viewer(self, env_name, env_package = "hpp_environments", visualize_affordances = []):
    """
    Build an instance of hpp-gepetto-viewer from the current problemSolver
    :param env_name: name of the urdf describing the environment
    :param env_package: name of the package containing this urdf (default to hpp_environments)
    :param visualize_affordances: list of affordances type to visualize, default to none
    """
    vf = ViewerFactory(self.ps)
    self.afftool = AffordanceTool ()
    self.afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
    self.afftool.loadObstacleModel (env_package, env_name, "planning", vf)
    try :
        self.v = vf.createViewer(displayArrows = True)
    except Exception:
        print("No viewer started !")
        class FakeViewer():
            def __init__(self):
                return
            def __call__(self,q):
                return
            def addLandmark(self,a,b):
                return
        self.v = FakeViewer()
    self.pp = PathPlayer(self.v)
    for aff_type in visualize_affordances:
      self.afftool.visualiseAffordances(aff_type, self.v, self.v.color.lightBrown)



  def init_planner(self, kinodynamic = True):
    """
    Select the rbprm methods, and the kinodynamic ones if required
    :param kinodynamic: if True, also select the kinodynamic methods
    """
    self.ps.selectConfigurationShooter("RbprmShooter")
    self.ps.selectPathValidation("RbprmPathValidation", 0.05)
    if kinodynamic:
      self.ps.addPathOptimizer("RandomShortcutDynamic")
      self.ps.selectSteeringMethod("RBPRMKinodynamic")
      self.ps.selectDistance("Kinodynamic")
      self.ps.selectPathPlanner("DynamicPlanner")


  def solve(self):
    """
    Solve the path planning problem.
    q_init and q_goal must have been defined before calling this method
    """
    if len(self.q_init) != self.rbprmBuilder.getConfigSize():
      raise ValueError("Initial configuration vector do not have the right size")
    if len(self.q_goal) != self.rbprmBuilder.getConfigSize():
      raise ValueError("Goal configuration vector do not have the right size")
    self.ps.setInitialConfig (self.q_init)
    self.ps.addGoalConfig (self.q_goal)
    self.v(self.q_init)
    t = self.ps.solve ()
    print("Guide planning time : ",t)

  def display_path(self, path_id = -1, dt = 0.1):
    """
    Display the path in the viewer, if no path specified display the last one
    :param path_id: the Id of the path specified, default to the most recent one
    :param dt: discretization step used to display the path (default to 0.1)
    """
    if path_id < 0:
      path_id = self.ps.numberPaths()-1
    self.pp.dt = dt
    self.pp.displayVelocityPath(path_id)

  def play_path(self, path_id = -1, dt = 0.01):
    """
    play the path in the viewer, if no path specified display the last one
    :param path_id: the Id of the path specified, default to the most recent one
    :param dt: discretization step used to display the path (default to 0.01)
    """
    if path_id < 0:
      path_id = self.ps.numberPaths()-1
    self.pp.dt = dt
    self.pp(path_id)

  def hide_rom(self):
    """
    Move visual ROM far above the meshs, as we cannot just delete it.
    """
    q_far = self.q_init[::]
    q_far[2] = -3
    self.v(q_far)