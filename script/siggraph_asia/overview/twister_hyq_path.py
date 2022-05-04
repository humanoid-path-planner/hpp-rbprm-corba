# Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.rbprmbuilder import Builder

from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent


class Robot(Parent):
    rootJointType = "freeflyer"
    packageName = "hpp-rbprm-corba"
    meshPackageName = "hpp-rbprm-corba"
    # URDF file describing the trunk of the robot HyQ
    urdfName = "hyq_trunk_large"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, robotName, load=True):
        Parent.__init__(self, robotName, self.rootJointType, load)
        self.tf_root = "base_footprint"
        self.client.basic = Client()
        self.load = load


cl = Client()
cl.problem.selectProblem("robot1")

# Importing Gepetto viewer helper class
from hpp.gepetto import Viewer

rootJointType = "freeflyer"
packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
# URDF file describing the trunk of the robot HyQ
urdfName = "hyq_trunk_large"
# URDF files describing the reachable workspace of each limb of HyQ
urdfNameRom = ["hyq_lhleg_rom", "hyq_lfleg_rom", "hyq_rfleg_rom", "hyq_rhleg_rom"]
urdfSuffix = ""
srdfSuffix = ""

# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Builder()
rbprmBuilder.loadModel(
    urdfName,
    urdfNameRom,
    rootJointType,
    meshPackageName,
    packageName,
    urdfSuffix,
    srdfSuffix,
)
rbprmBuilder.setJointBounds("base_joint_xyz", [-2, 5, -1, 1, 0.3, 4])
# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter(
    ["hyq_rhleg_rom", "hyq_lfleg_rom", "hyq_rfleg_rom", "hyq_lhleg_rom"]
)
rbprmBuilder.setAffordanceFilter("hyq_rhleg_rom", ["Support"])
rbprmBuilder.setAffordanceFilter(
    "hyq_rfleg_rom",
    [
        "Support",
    ],
)
rbprmBuilder.setAffordanceFilter("hyq_lhleg_rom", ["Support"])
rbprmBuilder.setAffordanceFilter(
    "hyq_lfleg_rom",
    [
        "Support",
    ],
)
# We also bound the rotations of the torso.
rbprmBuilder.boundSO3([-0.4, 0.4, -3, 3, -3, 3])

# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver(rbprmBuilder)

# Choosing a path optimizer
ps.addPathOptimizer("RandomShortcut")

from hpp.corbaserver.affordance.affordance import AffordanceTool

afftool = AffordanceTool()
afftool.setAffordanceConfig("Support", [0.5, 0.03, 0.00005])
afftool.setAffordanceConfig("Lean", [0.5, 0.03, 0.00005])

r = None
try:
    r = Viewer(ps)
    afftool.loadObstacleModel(packageName, "twister", "planning", r)
except:
    pass

# Choosing RBPRM shooter and path validation methods.
# Note that the standard RRT algorithm is used.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
# ~ ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)

q_init = rbprmBuilder.getCurrentConfig()
# ~ q_init [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]
q_init[0:3] = [0.19999999999999996, -0.82, 1.0]

# Playing the computed path
from hpp.gepetto import PathPlayer

# ~ pp = PathPlayer (rbprmBuilder.client.basic, r)
# ~
# ~ q_far = q_init [::]
# ~ q_far [0:3] = [-2, -3, 0.63];
# ~ r(q_far)
# ~
# ~ for i in range(1,10):
# ~ rbprmBuilder.client.basic.problem.optimizePath(i)
